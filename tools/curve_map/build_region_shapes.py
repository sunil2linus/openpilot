#!/usr/bin/env python3
"""
Build starpilot/common/map_region_shapes.json: simplified outlines of the US states and of Canada and Mexico, used to
tell which 2 x 2 degree map folders a selected map region really touches (a bounding box is far too coarse: Michigan's
box covers all of Wisconsin).

Dev-only (needs shapely); the car side reads the JSON with the standard library. Source: Natural Earth 1:10m
(public domain), https://github.com/nvkelso/natural-earth-vector:
  geojson/ne_10m_admin_1_states_provinces_lakes.geojson   (land: the Great Lakes are cut out)
  geojson/ne_10m_admin_0_countries_lakes.geojson

  git clone --depth 1 --filter=blob:none --sparse https://github.com/nvkelso/natural-earth-vector
  cd natural-earth-vector && git sparse-checkout set --no-cone geojson/ne_10m_admin_1_states_provinces_lakes.geojson \
     geojson/ne_10m_admin_0_countries_lakes.geojson
  python build_region_shapes.py natural-earth-vector/geojson

Every simplified shape COVERS its original (checked, hard failure otherwise): a folder holding any part of a state is
never missed, a folder near the line may be counted in addition. Per polygon part: tiny parts (islands) become their
bounding box; the rest is simplified (Douglas-Peucker, tolerance TOL), its holes dropped, then buffered by 1.5 x TOL
(the simplification moves the outline by at most TOL, so the buffer restores coverage).
"""
import json
import os
import sys

from shapely.geometry import MultiPolygon, Polygon, box, shape
from shapely.ops import unary_union

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "starpilot", "common", "map_region_shapes.json")
STATE_TOL, STATE_SMALL = 0.03, 0.01      # degrees, square degrees
NATION_TOL, NATION_SMALL = 0.1, 0.1
NATIONS = {"CAN": "CA", "MEX": "MX"}


def parts(g):
  return list(g.geoms) if isinstance(g, MultiPolygon) else [g]


def cover(g, tol, small):
  out = []
  for p in parts(g):
    if p.area < small:
      out.append(box(*p.bounds).buffer(tol / 2, join_style=2))
      continue
    s = Polygon(p.simplify(tol, preserve_topology=True).exterior)
    out.append(s.buffer(tol * 1.5, join_style=2))
  return unary_union(out)


def rings(g):
  return [[[round(x, 3), round(y, 3)] for x, y in p.exterior.coords] for p in parts(g)]


def checked(code, original, simplified):
  r = rings(simplified)
  back = unary_union([Polygon(ring) for ring in r])
  if not back.covers(original):
    sys.exit(f"{code}: the simplified outline does not cover the original ({original.difference(back).area:.6f} deg^2)")
  return r


def main(src):
  states = json.load(open(os.path.join(src, "ne_10m_admin_1_states_provinces_lakes.geojson")))
  out = {"source": "Natural Earth 1:10m (public domain), simplified to cover the original", "states": {}, "nations": {}}
  for f in states["features"]:
    p = f["properties"]
    if p.get("adm0_a3") != "USA":
      continue
    code = p["iso_3166_2"][3:]
    g = shape(f["geometry"])
    out["states"][code] = checked(code, g, cover(g, STATE_TOL, STATE_SMALL))
  nations = json.load(open(os.path.join(src, "ne_10m_admin_0_countries_lakes.geojson")))
  for f in nations["features"]:
    code = NATIONS.get(f["properties"].get("ADM0_A3"))
    if code:
      g = shape(f["geometry"])
      out["nations"][code] = checked(code, g, cover(g, NATION_TOL, NATION_SMALL))
  with open(OUT, "w") as fh:
    json.dump(out, fh, separators=(",", ":"))
  pts = sum(len(r) for d in (out["states"], out["nations"]) for rs in d.values() for r in rs)
  print(f"{len(out['states'])} states, {len(out['nations'])} nations, {pts} points, {os.path.getsize(OUT) // 1000} kB -> {OUT}")


if __name__ == "__main__":
  main(sys.argv[1] if len(sys.argv) > 1 else "natural-earth-vector/geojson")
