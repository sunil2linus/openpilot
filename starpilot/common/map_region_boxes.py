"""
Where a map region of StarPilot's map settings is on the globe: bounding boxes per MapsSelected token.

The map download (the mapd binary) knows the real shape of every region, but it keeps that to itself: this repo
only sends it the token CSV. The curve map builder (starpilot/system/curve_mapd.py) needs to know which 2 x 2 degree
map folders belong to the states the owner selected, so the folders left over from an older whole-US download are
never built, and so a ticked state is built in full. A bounding box per state is plenty for that: a folder is
built when its box overlaps the region's box (with MARGIN_DEG around the region), so the worst a loose box can do is
build a neighbouring folder too, never miss one.

Boxes are (lat_min, lat_max, lon_min, lon_max) in degrees, from the usual US state extents, rounded outwards to
0.01. A region can have several boxes (Alaska crosses the antimeridian). Countries: only the ones a US driver can
drive to (and the US itself, as the union of its states); any other token has no box, and the caller says so.

No openpilot imports: curve_mapd loads this file by path, so it can be unit tested without the rest of openpilot.
"""

COUNTRY_PREFIX = "nation."
STATE_PREFIX = "us_state."
MARGIN_DEG = 0.1

US_STATE_BOXES = {
  "AL": ((30.22, 35.01, -88.48, -84.88),),
  "AK": ((51.21, 71.39, -180.0, -129.97), (51.21, 53.0, 172.4, 180.0)),       # the Aleutians cross 180
  "AZ": ((31.33, 37.01, -114.82, -109.04),),
  "AR": ((33.0, 36.5, -94.62, -89.64),),
  "CA": ((32.53, 42.01, -124.42, -114.13),),
  "CO": ((36.99, 41.01, -109.06, -102.04),),
  "CT": ((40.98, 42.06, -73.73, -71.78),),
  "DE": ((38.45, 39.84, -75.79, -75.04),),
  "DC": ((38.79, 39.0, -77.12, -76.9),),
  "FL": ((24.5, 31.01, -87.64, -80.03),),
  "GA": ((30.35, 35.01, -85.61, -80.84),),
  "HI": ((18.9, 22.24, -160.25, -154.8),),                                    # the main islands (the rest has no roads)
  "ID": ((41.99, 49.01, -117.25, -111.04),),
  "IL": ((36.97, 42.51, -91.52, -87.49),),
  "IN": ((37.77, 41.77, -88.1, -84.78),),
  "IA": ((40.37, 43.51, -96.64, -90.14),),
  "KS": ((36.99, 40.01, -102.06, -94.58),),
  "KY": ((36.49, 39.15, -89.58, -81.96),),
  "LA": ((28.92, 33.02, -94.05, -88.81),),
  "ME": ((43.05, 47.46, -71.09, -66.94),),
  "MD": ((37.91, 39.73, -79.49, -75.04),),
  "MA": ((41.23, 42.89, -73.51, -69.92),),
  "MI": ((41.69, 48.31, -90.42, -82.41),),
  "MN": ((43.49, 49.39, -97.24, -89.48),),
  "MS": ((30.17, 35.01, -91.66, -88.09),),
  "MO": ((35.99, 40.62, -95.78, -89.09),),
  "MT": ((44.35, 49.01, -116.05, -104.03),),
  "NE": ((39.99, 43.01, -104.06, -95.3),),
  "NV": ((35.0, 42.01, -120.01, -114.03),),
  "NH": ((42.69, 45.31, -72.56, -70.7),),
  "NJ": ((38.92, 41.36, -75.56, -73.88),),
  "NM": ((31.33, 37.01, -109.06, -103.0),),
  "NY": ((40.49, 45.02, -79.77, -71.85),),
  "NC": ((33.84, 36.59, -84.33, -75.45),),
  "ND": ((45.93, 49.01, -104.05, -96.55),),
  "OH": ((38.4, 41.98, -84.82, -80.51),),
  "OK": ((33.61, 37.01, -103.01, -94.43),),
  "OR": ((41.99, 46.3, -124.57, -116.46),),
  "PA": ((39.71, 42.27, -80.52, -74.69),),
  "RI": ((41.14, 42.02, -71.87, -71.12),),
  "SC": ((32.03, 35.22, -83.36, -78.54),),
  "SD": ((42.47, 45.95, -104.06, -96.43),),
  "TN": ((34.98, 36.68, -90.31, -81.64),),
  "TX": ((25.83, 36.51, -106.65, -93.5),),
  "UT": ((36.99, 42.01, -114.06, -109.04),),
  "VT": ((42.72, 45.02, -73.44, -71.46),),
  "VA": ((36.54, 39.47, -83.68, -75.24),),
  "WA": ((45.54, 49.01, -124.77, -116.91),),
  "WV": ((37.2, 40.64, -82.65, -77.71),),
  "WI": ((42.49, 47.09, -92.89, -86.75),),
  "WY": ((40.99, 45.01, -111.06, -104.05),),
  # territories
  "PR": ((17.88, 18.53, -67.96, -65.22),),
  "VI": ((17.67, 18.42, -65.1, -64.55),),
  "GU": ((13.23, 13.66, 144.61, 144.96),),
  "AS": ((-14.39, -14.15, -170.86, -169.41),),
  "MP": ((14.1, 20.56, 144.88, 146.08),),
}

US_MAIN_CODES = tuple(c for c in US_STATE_BOXES if c not in ("PR", "VI", "GU", "AS", "MP"))

COUNTRY_BOXES = {
  "US": tuple(b for c in US_MAIN_CODES for b in US_STATE_BOXES[c]),
  "CA": ((41.67, 83.12, -141.01, -52.61),),
  "MX": ((14.53, 32.72, -118.41, -86.7),),
}


def region_boxes(token):
  """The boxes of a MapsSelected token ('us_state.MI', 'nation.CA'), or None when this file cannot place it."""
  if not isinstance(token, str):
    return None
  token = token.strip()
  if token.startswith(STATE_PREFIX):
    return US_STATE_BOXES.get(token[len(STATE_PREFIX):])
  if token.startswith(COUNTRY_PREFIX):
    return COUNTRY_BOXES.get(token[len(COUNTRY_PREFIX):])
  return None


def folder_box(folder):
  """(lat0, lat1, lon0, lon1) of a map folder name '42/-84': lat 42..44, lon -84..-82. None for another name."""
  try:
    la, lo = (int(x) for x in str(folder).split("/"))
  except ValueError:
    return None
  return la, la + 2, lo, lo + 2


def box_overlaps(folder, boxes, margin=MARGIN_DEG):
  """True when the map folder's box overlaps any of the region boxes, each grown by `margin` degrees."""
  fb = folder_box(folder)
  if fb is None or not boxes:
    return False
  la0, la1, lo0, lo1 = fb
  for lat_min, lat_max, lon_min, lon_max in boxes:
    if la0 < lat_max + margin and la1 > lat_min - margin and lo0 < lon_max + margin and lo1 > lon_min - margin:
      return True
  return False


def place_tokens(tokens):
  """Split tokens into {token: boxes} for the ones this file can place and [tokens] it cannot."""
  placed, unknown = {}, []
  for t in tokens:
    boxes = region_boxes(t)
    if boxes:
      placed[t] = boxes
    else:
      unknown.append(t)
  return placed, unknown


def folders_in_regions(folders, boxes_by_token, margin=MARGIN_DEG):
  """{folder: [token, ...]} for the folders (names) that overlap at least one region, tokens in the given order."""
  out = {}
  for f in folders:
    hits = [t for t, boxes in boxes_by_token.items() if box_overlaps(f, boxes, margin)]
    if hits:
      out[f] = hits
  return out


def folders_of_box(boxes, margin=MARGIN_DEG):
  """Every 2 x 2 degree folder name a region's boxes touch (downloaded or not): what a full build of it may need."""
  names = set()
  for lat_min, lat_max, lon_min, lon_max in boxes:
    la = int((lat_min - margin) // 2) * 2
    while la < lat_max + margin:
      lo = int((lon_min - margin) // 2) * 2
      while lo < lon_max + margin:
        if -90 <= la < 90 and -180 <= lo < 180:
          names.add(f"{la}/{lo}")
        lo += 2
      la += 2
  return sorted(names)
