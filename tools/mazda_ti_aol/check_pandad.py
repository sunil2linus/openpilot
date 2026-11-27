#!/usr/bin/env python3
"""Check a pandad built on the device (tools/mazda_ti_aol/pandad_device_build.sh) before committing it.

  python3 tools/mazda_ti_aol/check_pandad.py /path/to/aolbuild.tar [--repo .]

Needs aarch64-linux-gnu binutils and gcc, and qemu-aarch64. Nothing from the tar is executed: only the bytes of one
function (mazda_aol_param, no calls or memory loads) are run in a small harness under qemu-aarch64 for every uint16
param. Prints PASS / FAIL per check and, when all pass, the two files to copy into the repo.
"""
import argparse
import hashlib
import re
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

PANDAD = "openpilot/selfdrive/pandad/pandad"
PANDAD_O = "openpilot/selfdrive/pandad/pandad.o"
FUNCTION = "_Z15mazda_aol_paramt"
EXPECTED_NEW = [0, 1032, 1288, 1544, 1800]
EXPECTED_OLD = [0]
# The only tracked files the new source may change; every other change must also be in the unchanged-source build
COMMIT_FILES = (PANDAD, PANDAD_O)
results = []


def check(name, ok, detail=""):
  results.append(bool(ok))
  print(f"{'PASS' if ok else 'FAIL'}  {name}{(': ' + detail) if detail else ''}")
  return ok


def run(*cmd, data=None):
  return subprocess.run(cmd, input=data, capture_output=True, check=True, text=data is None or isinstance(data, str)).stdout


def git_blob(repo, rev, path):
  return subprocess.run(["git", "-C", str(repo), "show", f"{rev}:{path}"], capture_output=True, check=True).stdout


def sha(b):
  return hashlib.sha256(b).hexdigest()


def needed(path):
  return [l.split("[", 1)[1].rstrip("]") for l in run("aarch64-linux-gnu-readelf", "-dW", path).splitlines() if "(NEEDED)" in l]


def header(path):
  text = run("aarch64-linux-gnu-readelf", "-hW", path)
  return {k: re.search(rf"{k}:\s+(.*)", text).group(1).strip() for k in ("Class", "Machine", "Type")}


def interpreter(path):
  m = re.search(r"Requesting program interpreter: (\S+)\]", run("aarch64-linux-gnu-readelf", "-lW", path))
  return m.group(1) if m else None


def glibc_versions(path):
  text = run("aarch64-linux-gnu-readelf", "-VW", path)
  return sorted({tuple(int(x) for x in v.split(".")) for v in re.findall(r"GLIBC_(\d+\.\d+(?:\.\d+)?)", text)})


def sections(path):
  out = {}
  for line in run("aarch64-linux-gnu-readelf", "-SW", path).splitlines():
    m = re.match(r"\s*\[\s*(\d+)\]\s+(\S+)\s+\S+\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)", line)
    if m:
      out[int(m.group(1))] = (m.group(2), int(m.group(3), 16), int(m.group(4), 16), int(m.group(5), 16))
  return out


def defined_symbols(path):
  names = set()
  for line in run("aarch64-linux-gnu-nm", "--defined-only", path).splitlines():
    parts = line.split()
    if len(parts) == 3:
      names.add(parts[2])
  return names


def function_bytes(path, name=FUNCTION):
  secs = sections(path)
  for line in run("aarch64-linux-gnu-readelf", "-sW", path).splitlines():
    parts = line.split()
    if len(parts) >= 8 and parts[-1] == name and parts[3] == "FUNC" and parts[6].isdigit():
      value, size, index = int(parts[1], 16), int(parts[2]), int(parts[6])
      _, addr, offset, _ = secs[index]
      data = Path(path).read_bytes()
      return data[offset + value - addr: offset + value - addr + size]
  return None


def disassembly_is_self_contained(path, name=FUNCTION):
  text = run("aarch64-linux-gnu-objdump", "-d", f"--disassemble={name}", path)
  body = text.split(f"<{name}>:", 1)[1] if f"<{name}>:" in text else ""
  bad = re.findall(r"\t(adrp|adr|bl|blr|br|ldr\w*|str\w*)\s", body)
  return bool(body.strip()) and not bad, body


def accepted(code, harness):
  with tempfile.NamedTemporaryFile(suffix=".bin") as f:
    f.write(code)
    f.flush()
    out = subprocess.run(["qemu-aarch64", harness, f.name], capture_output=True, text=True, timeout=120, check=True).stdout
  return [int(x) for x in out.split()]


def build_harness(repo, workdir):
  harness = str(Path(workdir) / "harness")
  subprocess.run(["aarch64-linux-gnu-gcc", "-O1", "-static", "-o", harness,
                  str(Path(repo) / "tools/mazda_ti_aol/profile_harness.c")], check=True)
  return harness


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("tar")
  parser.add_argument("--repo", default=".")
  args = parser.parse_args()
  repo = Path(args.repo).resolve()
  with tempfile.TemporaryDirectory() as tmp:
    with tarfile.open(args.tar) as tar:
      tar.extractall(tmp, filter="data")
    root = Path(tmp)
    env = dict(l.split(" ", 1) for l in (root / "report/env.txt").read_text().splitlines() if " " in l)
    base, new = env["base"], env["new"]
    print(f"device: {env.get('agnos')} | {env.get('clang')} | base {base[:10]} new {new[:10]}")
    head = run("git", "-C", str(repo), "rev-parse", "HEAD").strip()
    check("built from this branch's commits", subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", new, head]).returncode == 0
          and run("git", "-C", str(repo), "log", "-1", "--format=%H", "--", "openpilot/starpilot/car/mazda/aol_policy.h").strip() ==
          run("git", "-C", str(repo), "log", "-1", "--format=%H", new, "--", "openpilot/starpilot/car/mazda/aol_policy.h").strip(),
          f"new {new[:10]} is in HEAD {head[:10]} with the same profile source")
    harness = build_harness(repo, tmp)

    committed = Path(tmp) / "committed_pandad"
    committed.write_bytes(git_blob(repo, base, PANDAD))
    new_bin, base_bin = root / "out/new" / PANDAD, root / "out/base" / PANDAD
    new_obj = root / "out/new" / PANDAD_O

    # 1. the build of the UNCHANGED source (equivalence of the device build with the committed one)
    base_identical = sha(base_bin.read_bytes()) == sha(committed.read_bytes())
    print(f"INFO  unchanged-source pandad {'is BYTE-IDENTICAL to' if base_identical else 'differs from'} the committed one")
    for label, path in (("unchanged-source build", base_bin), ("new build", new_bin)):
      h, hc = header(path), header(committed)
      check(f"{label}: ELF64 AArch64 PIE like the committed binary", h == hc and h["Machine"] == "AArch64", str(h))
      check(f"{label}: same interpreter", interpreter(path) == interpreter(committed), str(interpreter(path)))
      check(f"{label}: same linked libraries", needed(path) == needed(committed), ", ".join(needed(path)))
      gv, gc = glibc_versions(path), glibc_versions(committed)
      check(f"{label}: no newer glibc symbol version", (gv[-1] if gv else ()) <= (gc[-1] if gc else ()),
            f"max {'.'.join(map(str, gv[-1])) if gv else '-'} (committed {'.'.join(map(str, gc[-1])) if gc else '-'})")
      s, sc = sections(path), sections(committed)
      check(f"{label}: same sections", [v[0] for v in s.values()] == [v[0] for v in sc.values()])
      text = next(v[3] for v in s.values() if v[0] == ".text")
      text_c = next(v[3] for v in sc.values() if v[0] == ".text")
      check(f"{label}: .text size within 1 %", abs(text - text_c) <= text_c / 100, f"{text} vs {text_c}")
      check(f"{label}: same defined symbols", defined_symbols(path) == defined_symbols(committed),
            "only in new: %s; only in committed: %s" % (sorted(defined_symbols(path) - defined_symbols(committed))[:5],
                                                       sorted(defined_symbols(committed) - defined_symbols(path))[:5]))
    # 2. the profile function itself, run for every param
    for label, path, expected in (("committed", committed, EXPECTED_OLD), ("unchanged-source build", base_bin, EXPECTED_OLD),
                                  ("new build", new_bin, EXPECTED_NEW), ("new pandad.o", new_obj, EXPECTED_NEW)):
      code = function_bytes(path)
      ok, body = disassembly_is_self_contained(path) if code else (False, "")
      check(f"{label}: {FUNCTION} found, no calls or loads", code is not None and ok, f"{len(code or b'')} bytes")
      if code and ok:
        got = accepted(code, harness)
        check(f"{label}: Mazda AOL profile accepts exactly {expected}", got == expected, str(got[:8]))
    check("new pandad and pandad.o have the same profile code", function_bytes(new_bin) == function_bytes(new_obj))
    # 3. every other AOL profile function is unchanged
    others = sorted(n for n in defined_symbols(committed) if re.fullmatch(r"_Z\d+\w+_aol_param\w*", n) and n != FUNCTION)
    same = [n for n in others if function_bytes(new_bin, n) == function_bytes(committed, n)]
    check("every other AOL profile function is byte-identical", same == others and others, f"{len(same)} of {len(others)}")
    # 4. tracked files: only pandad and pandad.o may differ because of the new source
    def changed(phase):
      rows = [l.split() for l in (root / f"report/{phase}.sha256").read_text().splitlines() if l.strip()]
      return {path: built for built, committed_sha, path in rows if built != committed_sha}
    base_changed, new_changed = changed("base"), changed("new")
    extra = {p for p in new_changed if p not in COMMIT_FILES and base_changed.get(p) != new_changed[p]}
    check("no other tracked file changes because of the new source", not extra, ", ".join(sorted(extra)))
    env_only = sorted(p for p in new_changed if p not in COMMIT_FILES)
    if env_only:
      print("INFO  changed by the device toolchain alone (the same in the unchanged-source build; NOT to commit): " + ", ".join(env_only))
    check("pandad.o is an AArch64 relocatable object", header(new_obj)["Type"].startswith("REL") and header(new_obj)["Machine"] == "AArch64")
    if all(results):
      print("\nALL PASS. Copy exactly these two files into the repo, then commit and refresh prebuilt.json:")
      for p in COMMIT_FILES:
        print(f"  out/new/{p} (in the tar) -> {p}  sha256 {sha((root / 'out/new' / p).read_bytes())}")
      print(f"  tar -xf {args.tar} -C <scratch dir> out/new/{PANDAD} out/new/{PANDAD_O}, then copy both")
      return 0
    print("\nFAILED: do not commit the binary.")
    return 1


if __name__ == "__main__":
  sys.exit(main())
