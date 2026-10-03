#!/usr/bin/env python3
"""Decode an app/prof.c capture and rank code by time spent per byte.

  prof_report.py report ELF CAPTURE [options]
      CAPTURE is either a serial log containing the "PROF ..." dump or a
      binary dump of g_prof. Prints where time goes, the frame counters, the
      hot-code footprint, and which functions / object files are worth moving
      into ITCM (most busy time per KB first).

  prof_report.py cmd ELF
      Prints the debugger commands that dump g_prof or restart a capture.

  prof_report.py compare A.json B.json ...
      Side-by-side frame/footprint numbers from summaries saved with
      "report --summary-json", e.g. build A vs B vs C.

The toolchain's readelf is found on PATH, under ~/.vcpkg/artifacts, or via
--toolchain DIR.
"""

import argparse
import bisect
import csv
import glob
import json
import os
import re
import shutil
import struct
import subprocess
import sys

# Must match prof_capture_t in app/prof.c
HEADER_FMT = "<20I8I16I4I5Q"
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 232
HEADER_NAMES = ["magic", "version", "header_size", "state", "gen", "rearm", "rate_hz", "tick_hz",
                "cpu_hz", "shift", "n_windows", "n_buckets", "max_buckets", "samples", "unmapped",
                "start_tick", "elapsed_ticks", "capture_ticks", "isr_addr", "self_addr"]
FRAME32 = ["frames", "empty_frames", "min_cycles", "max_cycles"]
FRAME64 = ["cycles", "i_refill", "fe_stall", "d_refill", "be_stall"]
MAGIC = 0x464F5250
VERSION = 1
REARM_OFFSET = 20
STATE_OFFSET = 12
STATES = {0: "waiting", 1: "sampling", 2: "done"}

REGIONS = [
    ("ITCM", 0x00000000, 0x00040000),
    ("SRAM0", 0x02000000, 0x02400000),
    ("SRAM1", 0x02400000, 0x02800000),
    ("DTCM", 0x20000000, 0x20100000),
    ("MRAM", 0x80000000, 0x80600000),
]
CACHED_REGIONS = {"SRAM0", "SRAM1", "MRAM"}
ICACHE_KB = 32

DEFAULT_IDLE = {"prvIdleTask", "vPortSuppressTicksAndSleep", "vApplicationIdleHook",
                "pm_core_enter_normal_sleep", "pm_core_enter_deep_sleep"}


def die(msg):
    sys.exit("error: " + msg)


def warn(msg):
    print("warning: " + msg, file=sys.stderr)


def region_of(addr):
    for name, lo, hi in REGIONS:
        if lo <= addr < hi:
            return name
    return "other"


# ---------------------------------------------------------------------------
# Toolchain / ELF
# ---------------------------------------------------------------------------
def find_tool(name, toolchain_dir):
    exe = name + (".exe" if os.name == "nt" else "")
    if toolchain_dir:
        path = os.path.join(toolchain_dir, exe)
        if os.path.exists(path):
            return path
        die("%s not found in %s" % (exe, toolchain_dir))
    found = shutil.which(name)
    if found:
        return found
    pattern = os.path.expanduser("~/.vcpkg/artifacts/*/compilers.arm.arm.none.eabi.gcc/*/bin/" + exe)
    hits = sorted(glob.glob(pattern))
    if hits:
        return hits[-1]
    die("cannot find %s; pass --toolchain <gcc bin dir>" % name)


def run(tool, *args):
    return subprocess.run([tool] + list(args), check=True, capture_output=True, text=True).stdout


class Func:
    __slots__ = ("name", "addr", "size", "glob", "obj", "isec", "samples", "hot")

    def __init__(self, name, addr, size, is_global):
        self.name, self.addr, self.size, self.glob = name, addr, size, is_global
        self.obj, self.isec = "", ""
        self.samples, self.hot = 0.0, 0

    @property
    def end(self):
        return self.addr + self.size

    @property
    def region(self):
        return region_of(self.addr)


def parse_int(tok):
    return int(tok, 16) if tok.lower().startswith("0x") else int(tok)


def load_symbols(readelf, elf):
    """Returns (functions sorted by address, {object name: (addr, size)})"""
    funcs_by_addr, objects = {}, {}
    sym_re = re.compile(r"^\s*\d+:\s+([0-9a-fA-F]+)\s+(\S+)\s+(\w+)\s+(\w+)\s+\w+\s+(\S+)\s+(\S+)")
    for line in run(readelf, "-sW", elf).splitlines():
        m = sym_re.match(line)
        if not m:
            continue
        value, size, typ, bind, ndx, name = m.groups()
        if not ndx.isdigit():
            continue
        addr, size = int(value, 16), parse_int(size)
        if typ == "OBJECT":
            objects.setdefault(name, (addr, size))
            continue
        if typ != "FUNC":
            continue
        addr &= ~1
        cur = funcs_by_addr.get(addr)
        # Aliases share an address: keep the global name and the known size
        if cur is None or (bind == "GLOBAL" and not cur.glob) or (cur.size == 0 and size):
            funcs_by_addr[addr] = Func(name, addr, size, bind == "GLOBAL")
    funcs = sorted(funcs_by_addr.values(), key=lambda f: f.addr)
    # Size-less symbols (hand-written asm) run up to the next function
    for f, nxt in zip(funcs, funcs[1:] + [None]):
        if f.size == 0:
            f.size = min(nxt.addr - f.addr, 4096) if nxt else 4
    return funcs, objects


def load_sections(readelf, elf):
    secs = []
    sec_re = re.compile(r"^\s*\[\s*(\d+)\]\s+(\S+)\s+(\S+)\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+\S+\s+([A-Za-z]*)")
    for line in run(readelf, "-SW", elf).splitlines():
        m = sec_re.match(line)
        if not m or m.group(1) == "0":
            continue
        _, name, typ, addr, size, flags = m.groups()
        if "A" in flags:
            secs.append((name, int(addr, 16), int(size, 16)))
    return secs


def parse_map(path):
    """Input sections from a GNU ld map: sorted [(addr, size, section, object)]"""
    one = re.compile(r"^ (\.\S+)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
    name_only = re.compile(r"^ (\.\S+)\s*$")
    cont = re.compile(r"^\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
    out, pending = [], None
    with open(path, errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = one.match(line)
            if m:
                sec, addr, size, obj = m.group(1), int(m.group(2), 16), int(m.group(3), 16), m.group(4)
                pending = None
            elif pending and cont.match(line):
                m = cont.match(line)
                sec, addr, size, obj = pending, int(m.group(1), 16), int(m.group(2), 16), m.group(3)
                pending = None
            else:
                m = name_only.match(line)
                pending = m.group(1) if m else None
                continue
            if size and region_of(addr) != "other" and region_of(addr) != "DTCM":
                out.append((addr, size, sec, re.split(r"[\\/]", obj.strip())[-1]))
    out.sort()
    return out


def attach_objects(funcs, map_entries):
    starts = [e[0] for e in map_entries]
    found = 0
    for f in funcs:
        i = bisect.bisect_right(starts, f.addr) - 1
        if i >= 0:
            addr, size, sec, obj = map_entries[i]
            if addr <= f.addr < addr + size:
                f.obj, f.isec = obj, sec
                found += 1
    return found


# ---------------------------------------------------------------------------
# Capture parsing
# ---------------------------------------------------------------------------
def parse_binary(data):
    if len(data) < HEADER_SIZE:
        die("binary dump is shorter than the %d-byte header" % HEADER_SIZE)
    v = struct.unpack_from(HEADER_FMT, data, 0)
    h = dict(zip(HEADER_NAMES, v[:20]))
    if h["magic"] != MAGIC:
        die("bad magic 0x%08x: not a g_prof dump" % h["magic"])
    if h["version"] != VERSION or h["header_size"] != HEADER_SIZE:
        die("capture layout v%d/%d bytes does not match this script (v%d/%d)"
            % (h["version"], h["header_size"], VERSION, HEADER_SIZE))
    windows = []
    for i in range(h["n_windows"]):
        base, size, first, n = v[28 + 4 * i: 32 + 4 * i]
        windows.append({"base": base, "size": size, "first": first, "n": n})
    frame = dict(zip(FRAME32, v[44:48]))
    frame.update(zip(FRAME64, v[48:53]))
    need = HEADER_SIZE + 4 * h["n_buckets"]
    if len(data) < need:
        die("dump has %d bytes, needs %d for %d buckets" % (len(data), need, h["n_buckets"]))
    counts = struct.unpack_from("<%dI" % h["n_buckets"], data, HEADER_SIZE)
    if h["state"] != 2:
        warn("capture state is '%s', not 'done': numbers are partial" % STATES.get(h["state"], h["state"]))
    return {
        "source": "binary",
        "header": h,
        "windows": windows,
        "frame": frame,
        "unmapped_pc": list(v[20:28])[:min(h["unmapped"], 8)],
        "buckets": {i: c for i, c in enumerate(counts) if c},
    }


def kv(tokens):
    out = {}
    for t in tokens:
        if "=" in t:
            k, val = t.split("=", 1)
            out[k] = int(val, 16)
    return out


def parse_text(text, gen):
    blocks, cur = [], None
    for raw in text.splitlines():
        i = raw.find("PROF ")
        if i < 0:
            continue
        parts = raw[i + 5:].split()
        if not parts:
            continue
        tag, rest = parts[0], parts[1:]
        if tag == "BEGIN":
            cur = {"H": {}, "W": [], "F": {}, "U": [], "B": {}, "end": None}
        elif cur is None:
            continue
        elif tag == "H":
            cur["H"] = kv(rest)
        elif tag == "W":
            cur["W"].append(kv(rest))
        elif tag == "F":
            cur["F"] = kv(rest)
        elif tag == "U":
            cur["U"] = [int(t, 16) for t in rest]
        elif tag == "B":
            if len(rest) % 2:
                warn("odd-length bucket line dropped (garbled serial data?)")
                continue
            for j in range(0, len(rest), 2):
                cur["B"][int(rest[j], 16)] = int(rest[j + 1], 16)
        elif tag == "END":
            cur["end"] = kv(rest)
            blocks.append(cur)
            cur = None
    if not blocks:
        die("no complete PROF BEGIN ... PROF END block in the log")
    if gen is not None:
        blocks = [b for b in blocks if b["H"].get("gen") == gen]
        if not blocks:
            die("no capture with gen=%d in the log" % gen)
    b = blocks[-1]
    h = b["H"]
    for key in ("samples", "unmapped", "shift", "n_buckets"):
        if key not in h:
            die("PROF H line is missing '%s'" % key)
    total = sum(b["B"].values())
    if total != b["end"].get("sum") or len(b["B"]) != b["end"].get("nonzero"):
        warn("bucket lines do not add up to PROF END (lost serial data?): %d/%d buckets, %d/%d samples"
             % (len(b["B"]), b["end"].get("nonzero", -1), total, b["end"].get("sum", -1)))
    windows = [{"base": w["base"], "size": w["size"], "first": w["first"], "n": w["n"]} for w in b["W"]]
    return {"source": "uart", "header": h, "windows": windows, "frame": b["F"],
            "unmapped_pc": b["U"], "buckets": b["B"]}


def load_capture(path, gen):
    with open(path, "rb") as fh:
        data = fh.read()
    # The magic is the ASCII "PROF", so a log starting with "PROF BEGIN" matches it too;
    # a real dump also carries the binary header size right after the version
    if len(data) >= 12 and struct.unpack_from("<III", data, 0)[0::2] == (MAGIC, HEADER_SIZE):
        return parse_binary(data)
    return parse_text(data.decode("utf-8", errors="replace"), gen)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def bucket_addr(cap, idx):
    shift = cap["header"]["shift"]
    for w in cap["windows"]:
        if w["first"] <= idx < w["first"] + w["n"]:
            return w["base"] + ((idx - w["first"]) << shift)
    return None


def whole_buckets(cap, f):
    """Number of buckets lying entirely inside function f"""
    bsize = 1 << cap["header"]["shift"]
    for w in cap["windows"]:
        if w["base"] <= f.addr < w["base"] + w["size"]:
            k0 = -((w["base"] - f.addr) // bsize)       # ceil((addr - base) / bsize)
            k1 = (min(f.end, w["base"] + w["size"]) - w["base"]) // bsize
            return max(k1 - k0, 0)
    return 0


def attribute(cap, funcs, idle_names):
    """Spread each bucket's count over the functions it overlaps.

    A bucket straddling a function boundary is split by each function's
    sample density in the buckets lying wholly inside it, so a cold neighbour
    does not inherit time from a hot loop at the edge of the next function.
    A function with no whole bucket is weighted by the straddling bucket's
    own density. Returns ({region: samples with no function},
    [(bucket addr, busy samples in it)])"""
    bsize = 1 << cap["header"]["shift"]
    starts = [f.addr for f in funcs]
    nosym, placed, interior = {}, [], {}
    for idx, cnt in cap["buckets"].items():
        lo = bucket_addr(cap, idx)
        if lo is None:
            warn("bucket %d is outside every window" % idx)
            continue
        hi = lo + bsize
        hits = []
        j = max(bisect.bisect_right(starts, lo) - 1, 0)
        while j < len(funcs) and funcs[j].addr < hi:
            f = funcs[j]
            ov = min(hi, f.end) - max(lo, f.addr)
            if ov > 0:
                hits.append((f, ov))
            j += 1
        if not hits:
            nosym[region_of(lo)] = nosym.get(region_of(lo), 0) + cnt
        elif len(hits) == 1 and hits[0][1] == bsize:
            interior[id(hits[0][0])] = interior.get(id(hits[0][0]), 0) + cnt
        placed.append((lo, cnt, hits))

    density = {}
    for f in funcs:
        n = whole_buckets(cap, f)
        if n:
            density[id(f)] = interior.get(id(f), 0) / float(n * bsize)

    per_bucket = []
    for lo, cnt, hits in placed:
        if not hits:
            per_bucket.append((lo, cnt))
            continue
        weights = [ov * density.get(id(f), cnt / float(bsize)) for f, ov in hits]
        tot = sum(weights)
        if tot <= 0:
            weights = [float(ov) for _, ov in hits]
            tot = sum(weights)
        busy = 0.0
        for (f, ov), wgt in zip(hits, weights):
            share = cnt * wgt / tot
            f.samples += share
            if share > 0:
                f.hot += ov
            if f.name not in idle_names:
                busy += share
        per_bucket.append((lo, busy))
    return nosym, per_bucket


def footprint(per_bucket, regions=None, line=32, fractions=(0.5, 0.9, 0.95, 0.99)):
    """KB of the hottest cache lines needed to cover each fraction of busy samples"""
    lines = {}
    for lo, busy in per_bucket:
        if busy > 0 and (regions is None or region_of(lo) in regions):
            lines[lo // line] = lines.get(lo // line, 0.0) + busy
    total = sum(lines.values())
    out, acc, n = {}, 0.0, 0
    targets = list(fractions)
    for c in sorted(lines.values(), reverse=True):
        acc += c
        n += 1
        while targets and acc >= targets[0] * total - 1e-9:
            out[targets.pop(0)] = n * line / 1024.0
    for t in targets:
        out[t] = n * line / 1024.0
    return out, total


def fmt_kb(nbytes):
    return "%.1f" % (nbytes / 1024.0)


def trunc(s, n):
    return s if len(s) <= n else s[:n - 1] + "~"


def report(args):
    readelf = find_tool("arm-none-eabi-readelf", args.toolchain)
    cap = load_capture(args.capture, args.gen)
    h = cap["header"]
    funcs, objects = load_symbols(readelf, args.elf)
    sections = load_sections(readelf, args.elf)
    idle_names = DEFAULT_IDLE | set(args.idle or [])

    # Make sure the capture came from this ELF
    isr = next((f for f in funcs if f.name == "prof_sample_isr"), None)
    if isr and h.get("isr_addr") is not None and (h["isr_addr"] & ~1) != isr.addr:
        warn("prof_sample_isr is at 0x%08x in the ELF but 0x%08x in the capture: "
             "this capture is from a different build" % (isr.addr, h["isr_addr"] & ~1))
    if "g_prof" in objects and h.get("self_addr") is not None and objects["g_prof"][0] != h["self_addr"]:
        warn("g_prof address differs between ELF and capture: different build")

    have_map = False
    if args.map:
        entries = parse_map(args.map)
        found = attach_objects(funcs, entries)
        code_funcs = sum(1 for f in funcs if f.region != "other")
        have_map = True
        if code_funcs and found < 0.8 * code_funcs:
            warn("only %d of %d functions found in the map: map and ELF may be from different builds"
                 % (found, code_funcs))

    nosym, per_bucket = attribute(cap, funcs, idle_names)
    bsize = 1 << h["shift"]
    samples, unmapped = h["samples"], h["unmapped"]
    idle = sum(f.samples for f in funcs if f.name in idle_names)
    busy = samples - idle
    busy_mapped = samples - unmapped - idle
    tick_hz = h.get("tick_hz") or 100000000
    secs = h.get("elapsed_ticks", 0) / float(tick_hz)
    cpu_hz = args.cpu_mhz * 1e6 if args.cpu_mhz else (h.get("cpu_hz") or 400e6)

    # ---- capture overview
    print("Capture: %s, gen %d, %.1f s at %d Hz, %d samples, bucket %d B"
          % (cap["source"], h.get("gen", 0), secs, h.get("rate_hz", 0), samples, bsize))
    if samples == 0:
        print("  no PC samples (PROF_RATE_HZ = 0, or the sampler never fired)")
    else:
        print("  CPU busy %.1f%%   idle %.1f%%   outside windows %.2f%%"
              % (100.0 * busy / samples, 100.0 * idle / samples, 100.0 * unmapped / samples))
    print("\nCode windows:")
    for w in cap["windows"]:
        secs_in = [s[0] for s in sections if s[2] and w["base"] <= s[1] < w["base"] + w["size"]]
        print("  %-5s 0x%08x  %7s KB  %s" % (region_of(w["base"]), w["base"], fmt_kb(w["size"]), " ".join(secs_in)))
    if cap["unmapped_pc"]:
        names = []
        for pc in cap["unmapped_pc"]:
            f = next((f for f in funcs if f.addr <= pc < f.end), None)
            names.append("0x%08x%s" % (pc, " (%s)" % f.name if f else ""))
        print("  last PCs outside the windows: " + ", ".join(names))

    # ---- where busy time goes, by memory
    by_region = {}
    for f in funcs:
        if f.samples and f.name not in idle_names:
            by_region[f.region] = by_region.get(f.region, 0) + f.samples
    for r, c in nosym.items():
        by_region[r] = by_region.get(r, 0) + c
    if busy_mapped > 0:
        print("\nBusy time by memory:  " + "   ".join(
            "%s %.1f%%" % (r, 100.0 * c / busy_mapped) for r, c in sorted(by_region.items(), key=lambda x: -x[1])))

    # ---- frame counters
    fr = cap["frame"]
    frame_summary = {}
    print("\nFrame counter (LVGL refresh, rendering tasks only, flush waits excluded):")
    if not fr or fr.get("frames", 0) == 0:
        print("  no frames recorded: check prof_attach_display() and that the screen refreshed during the capture")
    elif fr.get("cycles", 0) == 0:
        print("  %d frames but zero cycles: the PMU is not counting (secure non-invasive debug disabled?)" % fr["frames"])
    else:
        n = fr["frames"]
        avg = fr["cycles"] / n
        frame_summary = {
            "frames": n,
            "cycles_per_frame": avg,
            "ms_per_frame": 1000.0 * avg / cpu_hz,
            "min_cycles": fr["min_cycles"],
            "max_cycles": fr["max_cycles"],
            "i_refill_per_frame": fr["i_refill"] / n,
            "i_refill_per_kcycle": 1000.0 * fr["i_refill"] / fr["cycles"],
            "fe_stall_pct": 100.0 * fr["fe_stall"] / fr["cycles"],
            "d_refill_per_frame": fr["d_refill"] / n,
            "be_stall_pct": 100.0 * fr["be_stall"] / fr["cycles"],
        }
        s = frame_summary
        print("  frames            %d rendered, %d empty" % (n, fr.get("empty_frames", 0)))
        print("  cycles/frame      avg %.0f (%.2f ms at %.0f MHz)   min %d   max %d"
              % (avg, s["ms_per_frame"], cpu_hz / 1e6, fr["min_cycles"], fr["max_cycles"]))
        print("  I-cache refills   %.0f/frame   %.2f per 1k cycles" % (s["i_refill_per_frame"], s["i_refill_per_kcycle"]))
        print("  front-end stall   %.1f%% of cycles" % s["fe_stall_pct"])
        print("  D-cache refills   %.0f/frame" % s["d_refill_per_frame"])
        print("  back-end stall    %.1f%% of cycles" % s["be_stall_pct"])

    # ---- hot-code footprint vs the I-cache
    fp_all, busy_all = footprint(per_bucket)
    fp_cached, busy_cached = footprint(per_bucket, CACHED_REGIONS)
    if busy_all > 0:
        print("\nHot-code footprint (KB of hottest 32-byte lines covering X%% of busy samples; I-cache is %d KB):"
              % ICACHE_KB)
        print("  all code            " + "   ".join("%d%%: %.1f" % (k * 100, v) for k, v in fp_all.items()))
        if busy_cached > 0:
            print("  cached (MRAM/SRAM)  " + "   ".join("%d%%: %.1f" % (k * 100, v) for k, v in fp_cached.items())
                  + "   (%.1f%% of busy samples)" % (100.0 * busy_cached / busy_all))

    # ---- top functions
    hot = sorted((f for f in funcs if f.samples > 0 and f.name not in idle_names), key=lambda f: -f.samples)
    denom = max(busy_mapped, 1)

    def pct_per_kb(f):
        return (100.0 * f.samples / denom) / (f.size / 1024.0)

    col_obj = "  object" if have_map else ""
    print("\nTop %d functions by busy time:" % args.top)
    print("  %6s %6s %7s %7s %8s  %-5s  %-40s%s" % ("busy%", "cum%", "size", "hot", "%/KB", "mem", "function", col_obj))
    cum = 0.0
    for f in hot[:args.top]:
        p = 100.0 * f.samples / denom
        cum += p
        print("  %6.2f %6.1f %7d %7d %8.2f  %-5s  %-40s%s"
              % (p, cum, f.size, f.hot, pct_per_kb(f), f.region, trunc(f.name, 40), "  " + f.obj if have_map else ""))
    for r, c in sorted(nosym.items(), key=lambda x: -x[1]):
        print("  %6.2f        (samples in %s with no function symbol)" % (100.0 * c / denom, r))

    # ---- ITCM candidates
    itcm_used = sum(s[2] for s in sections if region_of(s[1]) == "ITCM")
    budget = args.budget_kb * 1024 if args.budget_kb is not None else max(0x40000 - itcm_used, 0)
    print("\nITCM candidates, most busy time per KB first (budget %s KB; ITCM holds %s KB now):"
          % (fmt_kb(budget), fmt_kb(itcm_used)))
    print("  %8s %6s %7s %8s %6s  %-5s  %-40s%s" % ("%/KB", "busy%", "size", "cum KB", "cum%", "mem", "function", col_obj))
    cand = sorted((f for f in hot if f.region != "ITCM"), key=lambda f: -pct_per_kb(f))
    used = cum = 0.0
    chosen = []
    for f in cand:
        if used + f.size > budget:
            continue
        used += f.size
        cum += 100.0 * f.samples / denom
        chosen.append(f)
        if len(chosen) <= args.top:
            print("  %8.2f %6.2f %7d %8s %6.1f  %-5s  %-40s%s"
                  % (pct_per_kb(f), 100.0 * f.samples / denom, f.size, fmt_kb(used), cum, f.region,
                     trunc(f.name, 40), "  " + f.obj if have_map else ""))
    if len(chosen) > args.top:
        print("  ... %d more; all %d fill %s KB and cover %.1f%% of busy time" % (len(chosen) - args.top, len(chosen), fmt_kb(used), cum))
    elif chosen:
        print("  %d functions, %s KB, %.1f%% of busy time" % (len(chosen), fmt_kb(used), cum))

    # ---- by object file: linker rules place whole files
    objs = {}
    if have_map:
        for f in funcs:
            if not f.obj or f.region == "other":
                continue
            o = objs.setdefault(f.obj, {"samples": 0.0, "size": 0, "regions": set()})
            o["size"] += f.size
            o["regions"].add(f.region)
            if f.name not in idle_names:
                o["samples"] += f.samples
        ranked = sorted(((k, v) for k, v in objs.items() if v["samples"] > 0),
                        key=lambda kv_: -(kv_[1]["samples"] / max(kv_[1]["size"], 1)))
        print("\nObject files, most busy time per KB of code first (for EXCLUDE_FILE / per-file rules):")
        print("  %8s %6s %8s %8s  %-10s  %s" % ("%/KB", "busy%", "code KB", "cum KB", "mem", "object"))
        acc = 0
        for name, o in ranked[:args.top]:
            p = 100.0 * o["samples"] / denom
            acc += o["size"]
            print("  %8.2f %6.2f %8s %8s  %-10s  %s"
                  % (p / (o["size"] / 1024.0), p, fmt_kb(o["size"]), fmt_kb(acc), "/".join(sorted(o["regions"])), name))

    # ---- files for comparing builds
    if args.csv:
        os.makedirs(args.csv, exist_ok=True)
        with open(os.path.join(args.csv, "functions.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["function", "addr", "size", "hot_bytes", "samples", "busy_pct", "pct_per_kb", "region", "object", "input_section"])
            for f in hot:
                w.writerow([f.name, "0x%08x" % f.addr, f.size, f.hot, "%.1f" % f.samples,
                            "%.4f" % (100.0 * f.samples / denom), "%.4f" % pct_per_kb(f), f.region, f.obj, f.isec])
        if objs:
            with open(os.path.join(args.csv, "objects.csv"), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["object", "code_bytes", "samples", "busy_pct", "regions"])
                for name, o in sorted(objs.items(), key=lambda kv_: -kv_[1]["samples"]):
                    w.writerow([name, o["size"], "%.1f" % o["samples"], "%.4f" % (100.0 * o["samples"] / denom),
                                "/".join(sorted(o["regions"]))])
        print("\nCSV written to %s" % args.csv)

    if args.summary_json:
        summary = {
            "label": args.label or os.path.splitext(os.path.basename(args.capture))[0],
            "elf": os.path.abspath(args.elf),
            "capture_s": secs,
            "samples": samples,
            "busy_pct": 100.0 * busy / samples if samples else None,
            "busy_by_region_pct": {r: 100.0 * c / denom for r, c in by_region.items()},
            "footprint_kb": {str(k): v for k, v in fp_all.items()},
            "footprint_cached_kb": {str(k): v for k, v in fp_cached.items()},
            "frame": frame_summary,
        }
        with open(args.summary_json, "w") as fh:
            json.dump(summary, fh, indent=2)
        print("Summary written to %s" % args.summary_json)


def cmd(args):
    readelf = find_tool("arm-none-eabi-readelf", args.toolchain)
    _, objects = load_symbols(readelf, args.elf)
    if "g_prof" not in objects:
        die("g_prof not found in %s (built with PROF_ENABLE=0?)" % args.elf)
    addr, size = objects["g_prof"]
    print("g_prof is at 0x%08x, %d bytes (0x%x)\n" % (addr, size, size))
    print("J-Link Commander (the core can keep running):")
    print("  mem32 0x%08x 1                 capture state: 0 waiting, 1 sampling, 2 done" % (addr + STATE_OFFSET))
    print("  savebin prof.bin, 0x%08x, 0x%x" % (addr, size))
    print("  w4 0x%08x 1                    start a new capture" % (addr + REARM_OFFSET))
    print("\nGDB:")
    print("  print g_prof.state")
    print("  dump binary value prof.bin g_prof")
    print("  set var g_prof.rearm = 1")


def compare(args):
    rows = []
    for path in args.summaries:
        with open(path) as fh:
            rows.append(json.load(fh))
    metrics = [
        ("ms/frame", lambda s: s["frame"].get("ms_per_frame"), "%.2f"),
        ("cycles/frame", lambda s: s["frame"].get("cycles_per_frame"), "%.0f"),
        ("min cycles", lambda s: s["frame"].get("min_cycles"), "%.0f"),
        ("I-refill/frame", lambda s: s["frame"].get("i_refill_per_frame"), "%.0f"),
        ("I-refill/kcycle", lambda s: s["frame"].get("i_refill_per_kcycle"), "%.2f"),
        ("front-end stall %", lambda s: s["frame"].get("fe_stall_pct"), "%.1f"),
        ("D-refill/frame", lambda s: s["frame"].get("d_refill_per_frame"), "%.0f"),
        ("back-end stall %", lambda s: s["frame"].get("be_stall_pct"), "%.1f"),
        ("CPU busy %", lambda s: s.get("busy_pct"), "%.1f"),
        ("footprint 90% KB", lambda s: s.get("footprint_kb", {}).get("0.9"), "%.1f"),
        ("footprint 99% KB", lambda s: s.get("footprint_kb", {}).get("0.99"), "%.1f"),
    ]
    width = max(12, max(len(r["label"]) for r in rows) + 2)
    print("%-20s" % "" + "".join("%*s" % (width, r["label"]) for r in rows) + ("   vs first" if len(rows) > 1 else ""))
    for name, get, fmt in metrics:
        vals = [get(r) for r in rows]
        line = "%-20s" % name + "".join("%*s" % (width, (fmt % v) if v is not None else "-") for v in vals)
        if len(rows) > 1 and vals[0]:
            line += "  " + " ".join("%+.1f%%" % (100.0 * (v - vals[0]) / vals[0]) if v is not None else "-"
                                    for v in vals[1:])
        print(line)
    regions = sorted({k for r in rows for k in r.get("busy_by_region_pct", {})})
    for reg in regions:
        vals = [r.get("busy_by_region_pct", {}).get(reg) for r in rows]
        print("%-20s" % ("busy in %s %%" % reg) + "".join("%*s" % (width, "%.1f" % v if v is not None else "-") for v in vals))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    r = sub.add_parser("report", help="decode a capture against its ELF")
    r.add_argument("elf")
    r.add_argument("capture", help="serial log with the PROF dump, or binary dump of g_prof")
    r.add_argument("--map", help="GNU ld map file, adds the object file per function")
    r.add_argument("--toolchain", help="directory containing arm-none-eabi-readelf")
    r.add_argument("--top", type=int, default=30, help="rows per table (default 30)")
    r.add_argument("--budget-kb", type=float, help="ITCM budget for candidates (default: free ITCM)")
    r.add_argument("--idle", action="append", help="extra function name to count as idle (repeatable)")
    r.add_argument("--cpu-mhz", type=float, help="core clock for ms/frame (default: value in capture, else 400)")
    r.add_argument("--gen", type=lambda s: int(s, 0), help="capture generation to use from a log (default: last)")
    r.add_argument("--csv", metavar="DIR", help="write functions.csv / objects.csv")
    r.add_argument("--summary-json", metavar="FILE", help="write a summary for 'compare'")
    r.add_argument("--label", help="name for this build in the summary (default: capture file name)")
    r.set_defaults(func=report)

    c = sub.add_parser("cmd", help="print debugger commands for dumping g_prof")
    c.add_argument("elf")
    c.add_argument("--toolchain")
    c.set_defaults(func=cmd)

    m = sub.add_parser("compare", help="compare summaries from several builds")
    m.add_argument("summaries", nargs="+")
    m.set_defaults(func=compare)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
