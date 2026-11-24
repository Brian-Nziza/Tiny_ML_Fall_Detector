#!/usr/bin/env python3
"""
fall_logger.py - live plot + labelled csv from the nucleo

    python fall_logger.py --label fall
    python fall_logger.py --port COM5 --label walk --last 4 --window 15
    python fall_logger.py --demo
    python fall_logger.py --replay recordings/raw.log --label fall
    python fall_logger.py --replay raw.log --no-plot
    python fall_logger.py --list-ports
    python fall_logger.py --dump --erase
    python fall_logger.py --dump --all
    python fall_logger.py --info
    python fall_logger.py --dump-file raw_dump.bin

needs: pip install pyserial matplotlib
"""
import argparse
import csv
import math
import os
import sys
import io
import struct
import threading
import time
import zlib
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

MG_PER_COUNT = 72
COUNT_CLIP = 127
GAP_US = 6000
WRAP = 1 << 32
SAMPLE_HZ = 400
STALE_S = 1.0
BREAK = "\x00BREAK"
PAUSE = "\x00PAUSE"
FC_HZ = 35.0
LABEL_NAMES = ["fall", "set_down", "tilt", "slide", "walk", "stairs", "sit", "other8", "other9"]


class Biquad:
    def __init__(self, fc, fs):
        k = math.tan(math.pi * fc / fs)
        c = math.sqrt(2.0)
        d = 1.0 + c * k + k * k
        self.b0 = k * k / d
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (k * k - 1.0) / d
        self.a2 = (1.0 - c * k + k * k) / d
        self.z1 = 0.0
        self.z2 = 0.0

    def settle(self, x0):
        self.z2 = (self.b2 - self.a2) * x0
        self.z1 = (self.b1 - self.a1) * x0 + self.z2

    def step(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


class Filter3:
    def __init__(self, fc, fs, first):
        self.axes = [Biquad(fc, fs) for _ in range(3)]
        for b, v in zip(self.axes, first):
            b.settle(v)

    def step(self, x, y, z):
        a = self.axes
        return a[0].step(x), a[1].step(y), a[2].step(z)


class Session:

    def __init__(self, label, outdir, stamp, last_s, fc=FC_HZ):
        self.label, self.outdir, self.stamp, self.last_s = label, outdir, stamp, last_s
        self.fc = fc
        self.filt = None
        self.index = 0
        self.events = []
        self.samples = []
        self.t_prev_raw = None
        self.t_us = 0
        self.gaps = 0

    def add_sample(self, t_raw, x, y, z):
        if self.t_prev_raw is None:
            self.t_us = 0
            self.filt = Filter3(self.fc, SAMPLE_HZ, (x, y, z))
        else:
            dt = (t_raw - self.t_prev_raw) % WRAP
            self.t_us += dt
            if dt > GAP_US:
                self.gaps += 1
        self.t_prev_raw = t_raw
        f = self.filt.step(x, y, z)
        self.samples.append((self.t_us / 1e6, x, y, z) + f)
        return f

    def end_event(self, rec_id, label=None):
        s = self.samples
        self.samples = []
        trimmed = 0.0
        if self.last_s and s:
            t_end = s[-1][0]
            kept = [p for p in s if p[0] >= t_end - self.last_s]
            trimmed = kept[0][0] - s[0][0]
            s = kept
        if not s:
            print(f"recording {rec_id:03d}: no samples (ignored)")
            return None
        peak = max(math.sqrt(x * x + y * y + z * z) for _, x, y, z, *_f in s) * MG_PER_COUNT / 1000
        fpeak = max(math.sqrt(a * a + b * b + c * c) for *_r, a, b, c in s) * MG_PER_COUNT / 1000
        clipped = sum(1 for _, x, y, z, *_f in s if max(abs(x), abs(y), abs(z)) >= COUNT_CLIP)
        dur = s[-1][0] - s[0][0]
        self.events.append((rec_id, s, dur, peak, clipped, label or self.label))
        warn = f"  <-- {clipped} samples clipped at the +-8 g limit" if clipped else ""
        cut = f"  (dropped first {trimmed:.1f} s)" if trimmed > 0.01 else ""
        print(f"recording {rec_id:03d} [{label or self.label}]: {len(s)} samples, {dur:.2f} s, "
              f"peak {peak:.2f} g (filtered {fpeak:.2f} g){warn}{cut}")
        return self.events[-1]

    def write_csv(self):
        if not self.events:
            self.samples = []
            self.t_prev_raw = None
            return None
        self.index += 1
        self.outdir.mkdir(parents=True, exist_ok=True)
        base = f"{self.label}_{self.stamp}_s{self.index}"
        path_raw = self.outdir / f"{base}.csv"
        path_filt = self.outdir / f"{base}_filt.csv"
        longest = max(len(e[1]) for e in self.events)
        header = (["recording", "label", "n_samples", "duration_s", "peak_g", "clipped"]
                  + [f"{axis}{i}" for i in range(longest) for axis in "xyz"])
        for path, cols in ((path_raw, (1, 2, 3)), (path_filt, (4, 5, 6))):
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                for rec_id, s, dur, peak, clipped, lab in self.events:
                    vals = []
                    for p in s:
                        vals += [round(p[c] * MG_PER_COUNT / 1000, 3) for c in cols]
                    pk = max(math.sqrt(sum(p[c] ** 2 for c in cols)) for p in s) * MG_PER_COUNT / 1000
                    w.writerow([rec_id, lab, len(s), round(dur, 3),
                                round(pk, 3), clipped] + vals)
        print(f"--> wrote {len(self.events)} rows to {path_raw}")
        print(f"    and the {self.fc:g} Hz Butterworth-filtered copy {path_filt.name}")
        if self.gaps:
            print(f"    note: {self.gaps} gaps in the sample stream (dt > {GAP_US} us)")
        self.events, self.samples, self.gaps = [], [], 0
        self.t_prev_raw = None
        return path_raw


class Shared:
    def __init__(self, window_s, baud):
        self.lock = threading.Lock()
        self.baud = baud
        self.pts = deque(maxlen=int(window_s * SAMPLE_HZ * 1.3) + 800)
        self.rx = deque(maxlen=4000)
        self.marks = deque(maxlen=100)
        self.port_open = False
        self.conn_note = "starting"
        self.last_sample_mono = None
        self.last_g = None
        self.n_samples = 0
        self.bad = 0
        self.gaps = 0
        self.rec_id = 0
        self.last_event = "-"
        self.last_csv = None
        self.sensor_err = None
        self.fatal = None
        self.label = "unlabeled"
        self.cmds = deque()
        self.dumps = deque()

    def say(self, text):
        with self.lock:
            self.last_event = text
        print(text)

    def set_conn(self, is_open, note):
        with self.lock:
            changed = (note != self.conn_note) or (is_open != self.port_open)
            self.port_open, self.conn_note = is_open, note
        if changed:
            print(f"[link] {note}")

    def add_break(self):
        with self.lock:
            self.pts.append((time.time(), float("nan"), float("nan")))

    def add_point(self, wall, g, gf):
        with self.lock:
            self.pts.append((wall, g, gf))
            self.rx.append(time.monotonic())
            self.last_sample_mono = self.rx[-1]
            self.last_g = g
            self.n_samples += 1

    def add_event(self, wall, rec_id, ev):
        with self.lock:
            self.marks.append((wall, "E", f"REC {rec_id:03d}"))
            self.rec_id = rec_id
            if ev:
                _, s, dur, peak, clipped, _lab = ev
                self.last_event = (f"{rec_id:03d}: {len(s)} samples, peak {peak:.2f} g"
                                   + ("  CLIPPED" if clipped else ""))
            else:
                self.last_event = f"{rec_id:03d}: empty (no samples)"

    def add_reset(self, wall, path):
        with self.lock:
            self.marks.append((wall, "R", "RESET"))
            self.rec_id = 0
            self.last_csv = path
            self.last_event = f"saved {path.name}" if path else "reset (nothing to save)"

    def bump(self, field, n=1):
        with self.lock:
            setattr(self, field, getattr(self, field) + n)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            rate = sum(1 for t in self.rx if now - t <= 1.0)
            age = None if self.last_sample_mono is None else now - self.last_sample_mono
            if self.fatal:
                state = "ERROR"
            elif not self.port_open:
                state = "NO CONNECTION"
            elif age is not None and age <= STALE_S:
                state = "CONNECTED"
            elif self.sensor_err:
                state = "SENSOR ERROR"
            else:
                state = "NO DATA"
            return dict(state=state, rate=rate, age=age, pts=list(self.pts),
                        marks=list(self.marks), note=self.conn_note, last_g=self.last_g,
                        n=self.n_samples, bad=self.bad, gaps=self.gaps, rec=self.rec_id,
                        last_event=self.last_event, last_csv=self.last_csv,
                        sensor_err=self.sensor_err, fatal=self.fatal, baud=self.baud)


def list_candidate_ports():
    return [p.device for p in serial.tools.list_ports.comports()
            if p.vid == 0x0483 or "STLink" in (p.description or "")
            or "STM" in (p.description or "")]


def short_err(e):
    text = str(e).strip().splitlines()[0] if str(e).strip() else type(e).__name__
    low = text.lower()
    if "access is denied" in low or "permission" in low or "busy" in low:
        return "port busy - close PuTTY / CubeIDE console / any other serial program"
    if ("no such file" in low or "errno 2" in low or "cannot find" in low or "readiness" in low
            or "disconnected" in low or "input/output error" in low or "errno 5" in low):
        return "device gone - USB cable unplugged or board powered off?"
    return text if len(text) < 90 else text[:87] + "..."


def parse_slot_header(raw):
    f = raw.decode("ascii", errors="replace").strip().split(",")
    try:
        slot, label, n, t0, t1 = (int(v) for v in f[1:6])
        crc = int(f[6], 16)
    except (ValueError, IndexError):
        return None
    if len(f) != 7 or not (0 <= slot <= 9 and 1 <= n <= 4000):
        return None
    return dict(slot=slot, label=label, n=n, t0=t0, t1=t1, crc=crc)


def serial_lines(args, shared, stop, on_dump=None):
    while not stop.is_set():
        port = args.port
        if not port:
            found = list_candidate_ports()
            if len(found) == 1:
                port = found[0]
            elif not found:
                shared.set_conn(False, "no ST-LINK COM port found - is the Nucleo's USB cable plugged in?")
                stop.wait(1.0)
                continue
            else:
                shared.set_conn(False, "several candidate ports (" + ", ".join(found)
                                + ") - pick one with --port")
                stop.wait(1.0)
                continue
        try:
            ser = serial.Serial(port, args.baud, timeout=0.2)
        except (serial.SerialException, OSError) as e:
            shared.set_conn(False, f"cannot open {port}: {short_err(e)}")
            stop.wait(1.0)
            continue
        shared.set_conn(True, f"{port} open")
        yield BREAK
        buf = b""
        pending = []
        last_rx = last_cmd = time.monotonic()
        try:
            ser.write(b"L")
            while not stop.is_set():
                while shared.cmds:
                    ser.write(shared.cmds.popleft())
                    last_cmd = time.monotonic()
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    if time.monotonic() - last_cmd > 1.5:
                        ser.write(b"L")
                        last_cmd = time.monotonic()
                    continue
                last_rx = time.monotonic()
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    g = parse_slot_header(raw) if raw.startswith(b"B,") else None
                    if g:
                        need = 3 * g["n"] + 1
                        t_wait = time.monotonic()
                        while len(buf) < need and time.monotonic() - t_wait < 3.0:
                            more = ser.read(need - len(buf))
                            if more:
                                buf += more
                                t_wait = time.monotonic()
                        g["data"], buf = buf[:3 * g["n"]], buf[need:]
                        g["ok"] = (len(g["data"]) == 3 * g["n"]
                                   and (zlib.crc32(g["data"]) & 0xFFFFFFFF) == g["crc"])
                        print(f"slot {g['slot']}: {g['n']} samples, crc "
                              f"{'OK' if g['ok'] else 'MISMATCH - not saved, stays on the board'}")
                        pending.append(g)
                        yield PAUSE
                        continue
                    line = raw.decode("ascii", errors="replace").strip()
                    if line.startswith("#DUMPDONE"):
                        for g in (on_dump(pending) if on_dump else []):
                            ser.write(b"K")
                            time.sleep(0.005)
                            ser.write(b"%d" % g["slot"])
                            time.sleep(0.03)
                        pending = []
                    yield line
        except (serial.SerialException, OSError) as e:
            shared.set_conn(False, f"lost connection on {port}: {short_err(e)}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
        stop.wait(1.0)


def replay_lines(path, pace, stop):
    t_prev, t_dev, start = None, 0, time.monotonic()
    with open(path, encoding="ascii", errors="replace") as f:
        for raw in f:
            if stop.is_set():
                return
            line = raw.strip()
            if pace and line.startswith("S,"):
                try:
                    t = int(line.split(",")[1])
                    if t_prev is not None:
                        t_dev += (t - t_prev) % WRAP
                    t_prev = t
                    ahead = t_dev / 1e6 - (time.monotonic() - start)
                    if ahead > 0.005:
                        stop.wait(ahead)
                except (ValueError, IndexError):
                    pass
            yield line


def demo_xyz(c, rnd):
    n = lambda: rnd.randint(-1, 1)
    if c < 3.0:
        sway = 3 * math.sin(2 * math.pi * 1.8 * c)
        return n(), n() + int(sway * 0.3), 14 + n() + int(sway)
    if c < 3.4:
        return n(), n(), 2 + n()
    if c < 3.45:
        return 25 + n() * 3, n() * 3, 70 + rnd.randint(-10, 10)
    if c < 3.9:
        k = c - 3.45
        return 12 + n(), n(), int(22 * math.exp(-k * 7) * math.sin(45 * k)) + 3 + n()
    return 14 + n(), n(), n()


def demo_lines(stop):
    import random
    rnd = random.Random(1)
    yield "#BOOT,demo (simulated board),400Hz,8g"
    start, emitted, rec, t0 = time.monotonic(), 0, 0, 1_000_000
    while not stop.is_set():
        due = int((time.monotonic() - start) * SAMPLE_HZ) - emitted
        for _ in range(min(due, 400)):
            k = emitted % (9 * SAMPLE_HZ)
            x, y, z = demo_xyz(k / SAMPLE_HZ, rnd)
            t = (t0 + emitted * 2500) & 0xFFFFFFFF
            yield f"S,{t},{x},{y},{z}"
            if k == 7 * SAMPLE_HZ:
                rec += 1
                yield f"E,{rec},{t}"
            elif k == 7 * SAMPLE_HZ + 100 and rec == 3:
                rec = 0
                yield f"R,{t}"
            emitted += 1
        time.sleep(0.005)


def run_reader(source, sess, shared, raw_log, stop):
    anchor, last_wall, last_flush = None, 0.0, time.monotonic()
    for line in source:
        if stop.is_set():
            break
        if raw_log:
            raw_log.write({BREAK: "#LINK,reopened", PAUSE: "#DUMP,slot received"}.get(line, line) + "\n")
            if time.monotonic() - last_flush > 1.0:
                raw_log.flush()
                last_flush = time.monotonic()

        if line == PAUSE:
            if sess.samples:
                print("dump arrived - discarding the unfinished on-wire recording")
            sess.samples, sess.t_prev_raw, anchor = [], None, None
            shared.add_break()
            continue
        if line == BREAK or line.startswith(("#BOOT", "#LINK")):
            if line.startswith("#BOOT"):
                print(line)
                shared.sensor_err = None
            if sess.samples:
                print("link/board restarted - discarding the unfinished recording")
            if sess.events:
                print("link/board restarted - saving the session so far")
                sess.label = shared.label
                path = sess.write_csv()
                shared.add_reset(last_wall or time.time(), path)
            sess.samples, sess.t_prev_raw, anchor = [], None, None
            shared.add_break()
            continue
        if not line:
            continue
        if line.startswith("#"):
            print(line)
            if line.startswith("#ERR"):
                shared.sensor_err = line[1:]
            elif line.startswith("#ERASED"):
                shared.say("board store erased" if line.endswith("1") else "ERASE FAILED")
            continue

        parts = line.split(",")
        try:
            if parts[0] == "S" and len(parts) == 5:
                t, x, y, z = (int(p) for p in parts[1:])
                fresh = sess.t_prev_raw is None
                gaps_before = sess.gaps
                fx, fy, fz = sess.add_sample(t, x, y, z)
                if sess.gaps > gaps_before:
                    shared.bump("gaps")
                if fresh or anchor is None:
                    anchor = max(time.time(), last_wall + 1.0 / SAMPLE_HZ) - sess.t_us / 1e6
                last_wall = anchor + sess.t_us / 1e6
                g = math.sqrt(x * x + y * y + z * z) * MG_PER_COUNT / 1000
                gf = math.sqrt(fx * fx + fy * fy + fz * fz) * MG_PER_COUNT / 1000
                shared.add_point(last_wall, g, gf)
            elif parts[0] == "E" and len(parts) == 3:
                rec_id = int(parts[1])
                ev = sess.end_event(rec_id, shared.label)
                shared.add_event(last_wall or time.time(), rec_id, ev)
            elif parts[0] == "R" and len(parts) == 2:
                print("session reset button -> writing CSV")
                sess.label = shared.label
                path = sess.write_csv()
                anchor = None
                shared.add_reset(last_wall or time.time(), path)
            else:
                shared.bump("bad")
        except ValueError:
            shared.bump("bad")


def reader_main(source, sess, shared, raw_log, stop):
    try:
        run_reader(source, sess, shared, raw_log, stop)
    except Exception as e:
        shared.fatal = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        if not stop.is_set():
            shared.set_conn(False, "stream ended (replay finished)")


INK, MUTED, GRID, GRID_MINOR = "#1f2328", "#656d76", "#d0d7de", "#ebeef1"
GREEN = "#16a34a"
STATUS_STYLE = {
    "CONNECTED":     ("●", "#15803d"),
    "NO DATA":       ("▲", "#b45309"),
    "NO CONNECTION": ("✕", "#b91c1c"),
    "SENSOR ERROR":  ("✕", "#b91c1c"),
    "ERROR":         ("✕", "#b91c1c"),
}


def run_gui(args, shared, stop, outdir):
    import matplotlib
    if args.snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.ticker import AutoMinorLocator, FuncFormatter

    plt.ioff()
    matplotlib.rcParams["path.simplify_threshold"] = 1.0
    base = math.floor(time.time())
    fig = plt.figure(figsize=(11, 6.4), facecolor="white")
    canvas = fig.canvas
    try:
        canvas.manager.set_window_title("Fall recorder - live accelerometer")
    except Exception:
        pass

    ax = fig.add_axes([0.075, 0.235, 0.9, 0.595], facecolor="white")
    ax.set_title(f"Total acceleration |a|      grey = raw      green = Butterworth {args.fc:g} Hz      dashed = 1 g",
                 loc="left", fontsize=10, color=MUTED, pad=8)
    ax.axhline(1.0, color="#9aa4af", lw=0.9, ls=(0, (4, 3)), zorder=1)
    ax.grid(True, which="major", axis="y", color=GRID, lw=0.8)
    ax.grid(True, which="minor", axis="y", color=GRID_MINOR, lw=0.6)
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _p: f"{v:g}"))
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(axis="y", colors=MUTED, labelsize=9)
    ax.tick_params(axis="x", which="both", bottom=False, labelbottom=False)
    ax.set_ylabel("|a|  (g)", color=MUTED, fontsize=10)
    fig.text(0.5, 0.165, "Time (local clock, HH:MM:SS)", ha="center", va="center",
             fontsize=10, color=MUTED)
    ax.set_xlim(0, args.window)
    ax.set_ylim(0, 2)

    xt = ax.get_xaxis_transform()
    anim = []

    def moving(artist):
        artist.set_animated(True)
        anim.append(artist)
        return artist

    (vgrid,) = ax.plot([], [], color=GRID, lw=0.8, transform=xt, zorder=1)
    moving(vgrid)
    tick_txt = [moving(ax.text(0, -0.015, "", transform=xt, ha="center", va="top",
                               fontsize=9, color=MUTED, clip_on=False)) for _ in range(14)]
    mk_line = [moving(ax.axvline(0, lw=1.1, zorder=2, visible=False)) for _ in range(12)]
    mk_text = [moving(ax.text(0, 0.985, "", transform=xt, ha="right", va="top", fontsize=9,
                              fontweight="bold", zorder=4, visible=False,
                              bbox=dict(facecolor="white", edgecolor="none", pad=1.5)))
               for _ in range(12)]
    (line,) = ax.plot([], [], color="#b4bcc5", lw=1.0, solid_joinstyle="round", zorder=2)
    moving(line)
    (fline,) = ax.plot([], [], color=GREEN, lw=1.7, solid_joinstyle="round", zorder=3)
    moving(fline)
    overlay = moving(ax.text(0.5, 0.80, "", transform=ax.transAxes, ha="center", va="center",
                             fontsize=20, fontweight="bold", zorder=5,
                             bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=6)))
    head = moving(fig.text(0.075, 0.945, "", fontsize=18, fontweight="bold", va="center"))
    detail = moving(fig.text(0.075, 0.895, "", fontsize=10, color=MUTED, va="center"))
    clock = moving(fig.text(0.975, 0.945, "", fontsize=12, color=INK, ha="right", va="center"))
    foot1 = moving(fig.text(0.075, 0.122, "", fontsize=10, color=INK, va="center"))
    fig.text(0.075, 0.022, "Left button = end recording   \u00b7   Right button: short = save CSV and reset, "
             "held 5 s = fetch what the board saved   \u00b7   closing this window also saves",
             fontsize=9, color=MUTED, va="center")
    fig.text(0.60, 0.068, f"CSV folder: {outdir.resolve()}", fontsize=8, color=MUTED, va="center")

    from matplotlib.widgets import Button
    from matplotlib.patches import Rectangle
    BOX = (0.125, 0.048, 0.15, 0.042)
    fig.text(BOX[0] - 0.008, BOX[1] + BOX[3] / 2, "Label", ha="right", va="center", fontsize=10, color=INK)
    box_rect = moving(Rectangle(BOX[:2], BOX[2], BOX[3], transform=fig.transFigure, figure=fig,
                                facecolor="white", edgecolor=MUTED, lw=1.0))
    fig.patches.append(box_rect)
    box_txt = moving(fig.text(BOX[0] + 0.006, BOX[1] + BOX[3] / 2, shared.label, ha="left", va="center",
                              fontsize=10, color=INK, family="monospace"))
    edit = {"on": False, "text": shared.label, "keys": {}}

    def box_show():
        blink = "|" if edit["on"] and int(time.monotonic() * 2) % 2 == 0 else " "
        box_txt.set_text(edit["text"][-22:] + (blink if edit["on"] else ""))
        box_rect.set_edgecolor("#d9480f" if edit["on"] else MUTED)
        box_rect.set_linewidth(1.8 if edit["on"] else 1.0)

    def box_focus(on):
        if on == edit["on"]:
            return
        edit["on"] = on
        if on:
            edit["keys"] = {k: matplotlib.rcParams[k] for k in matplotlib.rcParams if k.startswith("keymap.")}
            for k in edit["keys"]:
                matplotlib.rcParams[k] = []
        else:
            for k, v in edit["keys"].items():
                matplotlib.rcParams[k] = v
            if not edit["text"]:
                edit["text"] = "unlabeled"
            on_label(edit["text"])

    def on_click(e):
        if e.x is None:
            return
        fx, fy = e.x / fig.bbox.width, e.y / fig.bbox.height
        box_focus(BOX[0] <= fx <= BOX[0] + BOX[2] and BOX[1] <= fy <= BOX[1] + BOX[3])

    def on_key(e):
        if not edit["on"] or e.key is None:
            return
        if e.key in ("enter", "escape", "tab"):
            box_focus(False)
        elif e.key == "backspace":
            edit["text"] = edit["text"][:-1]
        elif e.key == "delete":
            edit["text"] = ""
        elif len(e.key) == 1 and len(edit["text"]) < 40:
            edit["text"] += e.key if (e.key.isalnum() or e.key in "_-") else "_"
        if edit["on"] and edit["text"]:
            on_label(edit["text"])

    canvas.mpl_connect("button_press_event", on_click)
    canvas.mpl_connect("key_press_event", on_key)
    b_fetch = Button(fig.add_axes([0.295, 0.048, 0.14, 0.042]), "Fetch from board")
    b_erase = Button(fig.add_axes([0.445, 0.048, 0.14, 0.042]), "Erase board")
    for b in (b_fetch, b_erase):
        b.label.set_fontsize(9)
    arm = {"t": 0.0}

    def on_label(text):
        clean = "".join(c if (c.isalnum() or c in "_-") else "_" for c in text.strip()) or "unlabeled"
        shared.label = clean

    def on_fetch(_e):
        shared.cmds.append(b"D")
        shared.say("asked the board for its saved recordings ...")

    def on_erase(_e):
        if time.monotonic() - arm["t"] < 3.0:
            arm["t"] = 0.0
            b_erase.label.set_text("Erase board")
            shared.cmds.append(b"X")
            shared.say("erasing the board's store ...")
        else:
            arm["t"] = time.monotonic()
            b_erase.label.set_text("Click again = erase")
        canvas.draw_idle()

    b_fetch.on_clicked(on_fetch)
    b_erase.on_clicked(on_erase)
    fig._widgets_keepalive = (b_fetch, b_erase)

    bg = [None]

    def on_draw(_event=None):
        bg[0] = canvas.copy_from_bbox(fig.bbox)
        for art in anim:
            fig.draw_artist(art)

    canvas.mpl_connect("draw_event", on_draw)
    canvas.mpl_connect("close_event", lambda _e: stop.set())

    st = {"ytop": 2, "err": None, "t_prev": None, "fps": 0.0}
    step = max(1, round(args.window / 10))

    def update():
        if bg[0] is None:
            return
        s = shared.snapshot()
        now = time.time()
        tp = time.perf_counter()
        if st["t_prev"] is not None and tp > st["t_prev"]:
            st["fps"] = 0.9 * st["fps"] + 0.1 / (tp - st["t_prev"])
        st["t_prev"] = tp
        arr = np.array(s["pts"], dtype=float) if s["pts"] else np.empty((0, 3))
        right = max(now, arr[-1, 0]) if len(arr) else now
        left = right - args.window
        ax.set_xlim(left - base, right - base)

        vis_max = 0.0
        if len(arr):
            a = arr[arr[:, 0] >= left - 0.5]
            line.set_data(a[:, 0] - base, a[:, 1])
            fline.set_data(a[:, 0] - base, a[:, 2])
            if np.isfinite(a[:, 1]).any():
                vis_max = float(np.nanmax(a[:, 1]))
        else:
            line.set_data([], [])
            fline.set_data([], [])
        ytop = max(2, math.ceil(vis_max * 1.1))
        if ytop != st["ytop"]:
            st["ytop"] = ytop
            ax.set_ylim(0, ytop)
            canvas.draw()

        first = math.ceil(left / step) * step
        ticks = [first + i * step for i in range(len(tick_txt)) if first + i * step <= right]
        gx = [v for t in ticks for v in (t - base, t - base, float("nan"))]
        gy = [v for _ in ticks for v in (0.0, 1.0, float("nan"))]
        vgrid.set_data(gx, gy)
        for i, txt in enumerate(tick_txt):
            if i < len(ticks):
                txt.set_position((ticks[i] - base, -0.015))
                txt.set_text(time.strftime("%H:%M:%S", time.localtime(ticks[i])))
                txt.set_visible(True)
            else:
                txt.set_visible(False)

        marks = [m for m in s["marks"] if left <= m[0] <= right][-len(mk_line):]
        for i, (ln, tx) in enumerate(zip(mk_line, mk_text)):
            if i < len(marks):
                wall, kind, text = marks[i]
                x = wall - base
                col = INK if kind == "E" else MUTED
                ln.set_xdata([x, x])
                ln.set_color(col)
                ln.set_linestyle((0, (5, 3)) if kind == "E" else (0, (1, 2)))
                tx.set_position((x, 0.985 if kind == "E" else 0.90))
                tx.set_text(text)
                tx.set_color(col)
                ln.set_visible(True)
                tx.set_visible(True)
            else:
                ln.set_visible(False)
                tx.set_visible(False)

        state = s["state"]
        icon, color = STATUS_STYLE[state]
        head.set_text(f"{icon}  {state}")
        head.set_color(color)
        if state == "CONNECTED":
            txt = (f"{s['note'].replace(' open', '')}  ·  {s['baud']} baud  ·  "
                   f"{s['rate']} samples/s  ·  {s['n']:,} samples  ·  now {s['last_g']:.2f} g")
            overlay.set_text("")
        elif state == "NO DATA":
            txt = (f"{s['note']}, but no valid samples for {s['age']:.0f} s"
                   if s["age"] else f"{s['note']}, waiting for the first sample")
            txt += "  -  board unpowered, reset, not flashed, or wrong baud?"
            overlay.set_text("NO DATA")
        elif state == "SENSOR ERROR":
            txt = f"{s['note']}  -  board says: {s['sensor_err']}"
            overlay.set_text("SENSOR ERROR")
        elif state == "ERROR":
            txt = f"reader stopped: {s['fatal']}"
            overlay.set_text("ERROR")
        else:
            txt = s["note"] + ("" if "ended" in s["note"] else "  -  retrying every second")
            overlay.set_text("NO CONNECTION")
        overlay.set_color(color)
        extra = []
        if s["gaps"]:
            extra.append(f"{s['gaps']} gaps")
        if s["bad"]:
            extra.append(f"{s['bad']} bad lines")
        detail.set_text(txt + (("  ·  " + ", ".join(extra)) if extra else "")
                        + f"  ·  display {st['fps']:.0f} fps")
        box_show()
        clock.set_text(datetime.now().strftime("%a %Y-%m-%d   %H:%M:%S"))
        foot1.set_text(f"Recordings this session: {s['rec']:03d}     last: {s['last_event']}")

        canvas.restore_region(bg[0])
        for art in anim:
            fig.draw_artist(art)
        canvas.blit(fig.bbox)

    def safe_update():
        try:
            if arm["t"] and time.monotonic() - arm["t"] >= 3.0:
                arm["t"] = 0.0
                b_erase.label.set_text("Erase board")
                canvas.draw_idle()
            while shared.dumps:
                events, png = shared.dumps.popleft()
                f2 = make_dump_fig(plt, events, args.fc)
                f2.savefig(png, dpi=110, facecolor="white")
                try:
                    f2.canvas.manager.set_window_title(f"Fetched from the board - {png.name}")
                except Exception:
                    pass
                f2.show()
            update()
        except Exception as e:
            msg = f"{type(e).__name__}: {e}"
            if msg != st["err"]:
                st["err"] = msg
                traceback.print_exc()

    if args.snapshot:
        canvas.draw()
        t_start = time.monotonic()
        tt, nfr, worst = 0.0, 0, 0.0
        if os.environ.get("TYPE_LABEL"):
            from matplotlib.backend_bases import KeyEvent, MouseEvent
            px, py = (BOX[0] + 0.02) * fig.bbox.width, (BOX[1] + 0.02) * fig.bbox.height
            canvas.callbacks.process("button_press_event", MouseEvent("button_press_event", canvas, px, py, 1))
            for ch in ["delete"] + list(os.environ["TYPE_LABEL"]):
                canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", canvas, ch))
            n_draws = [0]
            canvas.mpl_connect("draw_event", lambda _e: n_draws.__setitem__(0, n_draws[0] + 1))
            canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", canvas, "x"))
            canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", canvas, "backspace"))
            print(f"TYPE_LABEL: label is now '{shared.label}', full redraws caused by typing: {n_draws[0]}")
        while not stop.is_set() and time.monotonic() - t_start < args.snapshot_after:
            time.sleep(0.02)
            a0 = time.perf_counter()
            safe_update()
            dt = time.perf_counter() - a0
            tt, nfr, worst = tt + dt, nfr + 1, max(worst, dt)
        if os.environ.get("BENCH") and nfr:
            print(f"BENCH frames={nfr}  per-frame={tt / nfr * 1000:.1f} ms  worst={worst * 1000:.1f} ms")
        for art in anim:
            art.set_animated(False)
        fig.savefig(args.snapshot, dpi=110, facecolor="white")
        stop.set()
        return

    timer = canvas.new_timer(interval=33)
    timer.add_callback(safe_update)
    timer.start()
    fig._timer_keepalive = timer
    plt.show()


class FileSer:

    def __init__(self, path):
        self.buf = io.BytesIO(Path(path).read_bytes())
        self.in_waiting = 0

    def write(self, b):
        return len(b)

    def readline(self):
        return self.buf.readline()

    def read(self, n=1):
        return self.buf.read(n)

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class TeeSer:

    def __init__(self, ser):
        self.ser, self.rx = ser, bytearray()

    def write(self, b):
        return self.ser.write(b)

    def readline(self):
        d = self.ser.readline()
        self.rx += d
        return d

    def read(self, n=1):
        d = self.ser.read(n)
        self.rx += d
        return d

    def reset_input_buffer(self):
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()


def read_exact(ser, n, patience_s=3.0):
    data = b""
    last = time.monotonic()
    while len(data) < n and time.monotonic() - last < patience_s:
        chunk = ser.read(n - len(data))
        if chunk:
            data += chunk
            last = time.monotonic()
    return data


def receive_dump(ser, want_all, patience_s=15.0):
    ser.write(b"A" if want_all else b"D")
    slots = []
    last = time.monotonic()
    while time.monotonic() - last < patience_s:
        raw = ser.readline()
        if not raw:
            continue
        last = time.monotonic()
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("B,"):
            f = line.split(",")
            try:
                slot, label, n, t0, t1 = (int(v) for v in f[1:6])
                crc = int(f[6], 16)
            except (ValueError, IndexError):
                print(f"unreadable slot header: {line}")
                continue
            data = read_exact(ser, 3 * n)
            ser.read(1)
            ok = len(data) == 3 * n and (zlib.crc32(data) & 0xFFFFFFFF) == crc
            slots.append(dict(slot=slot, label=label, n=n, t0=t0, t1=t1, crc=crc, data=data, ok=ok))
            print(f"slot {slot}: {n} samples, label {label}, crc {'OK' if ok else 'MISMATCH - not saved, not acked'}")
            last = time.monotonic()
        elif line.startswith("#DUMPDONE"):
            return slots
        elif line:
            print(line)
    print("timed out waiting for the end of the dump")
    return slots


def dump_into_session(sess, slots, names, force_label=None):
    for g in slots:
        sess.t_prev_raw = None
        n = g["n"]
        dt = ((g["t1"] - g["t0"]) % WRAP) / max(1, n - 1)
        if abs(dt - 1e6 / SAMPLE_HZ) > 0.03 * 1e6 / SAMPLE_HZ:
            print(f"slot {g['slot']}: average sample spacing {dt:.0f} us instead of {1e6 / SAMPLE_HZ:.0f} - samples missed?")
        for i in range(n):
            x, y, z = struct.unpack_from("bbb", g["data"], 3 * i)
            sess.add_sample((g["t0"] + round(i * dt)) & 0xFFFFFFFF, x, y, z)
        lab = force_label or (names[g["label"] - 1] if 1 <= g["label"] <= len(names) else f"label{g['label']}")
        sess.end_event(g["slot"] + 1, lab)


def make_dump_fig(plt, events, fc):
    rows = len(events)
    fig, axs = plt.subplots(rows, 1, figsize=(10, 2.2 * rows + 0.8), squeeze=False, facecolor="white")
    for ax, (rec_id, s, dur, peak, clipped, lab) in zip(axs[:, 0], events):
        t = [p[0] for p in s]
        raw = [math.sqrt(p[1] ** 2 + p[2] ** 2 + p[3] ** 2) * MG_PER_COUNT / 1000 for p in s]
        flt = [math.sqrt(p[4] ** 2 + p[5] ** 2 + p[6] ** 2) * MG_PER_COUNT / 1000 for p in s]
        ax.plot(t, raw, color="#b4bcc5", lw=1.0, label="raw")
        ax.plot(t, flt, color=GREEN, lw=1.6, label=f"Butterworth {fc:g} Hz")
        ax.axhline(1.0, color="#9aa4af", lw=0.8, ls=(0, (4, 3)))
        ax.set_title(f"slot {rec_id - 1}  \u00b7  {lab}  \u00b7  {len(s)} samples  \u00b7  peak {peak:.2f} g",
                     loc="left", fontsize=9, color=MUTED)
        ax.set_ylabel("|a| (g)", fontsize=8)
        ax.grid(True, axis="y", color=GRID, lw=0.7)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axs[0, 0].legend(loc="upper right", fontsize=8, frameon=False)
    axs[-1, 0].set_xlabel("time since the start of the recording (s)")
    fig.tight_layout()
    return fig


def plot_dump(events, path, show, fc):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = make_dump_fig(plt, events, fc)
    fig.savefig(path, dpi=110, facecolor="white")
    print(f"plot saved to {path}")
    if show:
        plt.show()


def gui_dump_saver(args, shared, outdir):
    def save(slots):
        good = [g for g in slots if g["ok"]]
        if not slots:
            shared.say("no new recordings on the board (all fetched already, or none saved)")
            return []
        label = shared.label
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ds = Session(f"{label}_dump", outdir, stamp, 0.0, args.fc)
        dump_into_session(ds, good, None, label)
        events = list(ds.events)
        path = ds.write_csv()
        if not path:
            shared.say(f"dump failed: {len(slots)} slots arrived, none passed the CRC check")
            return []
        for g in good:
            (outdir / f"slot{g['slot']}_{stamp}.bin").write_bytes(g["data"])
        bad = len(slots) - len(good)
        shared.say(f"data found and saved as: {path.name}  ({len(good)} recordings, label '{label}'"
                   + (f", {bad} FAILED the CRC and stay on the board" if bad else "") + ")")
        with shared.lock:
            shared.last_csv = path
        shared.dumps.append((events, outdir / f"{label}_dump_{stamp}.png"))
        return good
    return save


def pick_port(args):
    if args.port:
        return args.port
    found = list_candidate_ports()
    if len(found) == 1:
        return found[0]
    raise SystemExit("no single ST-LINK port found (" + (", ".join(found) or "none")
                     + ") - use --list-ports and --port")


def dump_main(args, outdir, stamp):
    names = [n.strip() for n in args.names.split(",")]
    sess = Session("dump", outdir, stamp, args.last, args.fc)
    tee = None
    if args.dump_file:
        ser = FileSer(args.dump_file)
    else:
        port = pick_port(args)
        tee = TeeSer(serial.Serial(port, args.baud, timeout=0.5))
        ser = tee
        print(f"connected to {port}")
        ser.write(b"l")
        time.sleep(0.4)
        ser.reset_input_buffer()
        if args.info:
            ser.write(b"I")
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line.startswith("#STORE"):
                    _, used, total, pending = line.split(",")
                    print(f"board store: {used}/{total} slots used, {pending} not fetched yet")
                    break
            else:
                print("no answer from the board")
            if not args.dump:
                ser.close()
                return 0

    slots = receive_dump(ser, args.all)
    good = [g for g in slots if g["ok"]]
    if not slots:
        print("the board sent no recordings (nothing saved, or all fetched already - try --all)")
    outdir.mkdir(parents=True, exist_ok=True)
    if tee is not None and tee.rx:
        (outdir / f"dumpraw_{stamp}.bin").write_bytes(bytes(tee.rx))
    for g in good:
        (outdir / f"slot{g['slot']}_{stamp}.bin").write_bytes(g["data"])

    dump_into_session(sess, good, names)
    events = list(sess.events)
    path = sess.write_csv()
    if events:
        plot_dump(events, outdir / f"dump_{stamp}.png", not args.no_plot, args.fc)

    if path and tee is not None:
        for g in good:
            ser.write(b"K")
            time.sleep(0.005)
            ser.write(b"%d" % g["slot"])
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                if ser.readline().decode("ascii", errors="replace").startswith("#ACK"):
                    break
        print(f"{len(good)} slots saved to disk and marked as fetched on the board")
        if args.erase:
            if len(good) == len(slots) and slots:
                ser.write(b"X")
                t_end = time.monotonic() + 8.0
                while time.monotonic() < t_end:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("#ERASED"):
                        print("board store erased" if line.endswith("1") else "ERASE FAILED")
                        break
            else:
                print("not erasing: at least one slot did not verify")
    if tee is not None:
        ser.close()
    return 0 if len(good) == len(slots) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (COM5, /dev/ttyACM0 ...); auto-detected if omitted")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--label", default="unlabeled", help="label written into every row of this run")
    ap.add_argument("--outdir", default="recordings")
    ap.add_argument("--last", type=float, default=None,
                    help="keep only the last N seconds before each press (on the wire: default 5; "
                         "pump and dump keeps all; 0 = keep everything, but long rows exceed "
                         "Excel's 16384-column limit)")
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history shown in the graph (default 10)")
    ap.add_argument("--no-plot", action="store_true", help="no graph window, console + CSV only")
    ap.add_argument("--replay", help="parse a saved raw log instead of opening a serial port")
    ap.add_argument("--demo", action="store_true", help="simulated board, no hardware needed")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--dump", action="store_true", help="pump and dump: fetch the recordings saved in the board's flash")
    ap.add_argument("--all", action="store_true", help="with --dump: also fetch slots that were fetched before")
    ap.add_argument("--erase", action="store_true", help="with --dump: wipe the board's store after everything verified")
    ap.add_argument("--info", action="store_true", help="ask the board how many slots are used")
    ap.add_argument("--dump-file", help="parse a saved raw dump (dumpraw_*.bin) instead of opening a port")
    ap.add_argument("--fc", type=float, default=FC_HZ, help="Butterworth corner in Hz (default 35)")
    ap.add_argument("--names", default=",".join(LABEL_NAMES),
                    help="label names for the board's label button 1..9, comma separated")
    ap.add_argument("--snapshot", help=argparse.SUPPRESS)
    ap.add_argument("--snapshot-after", type=float, default=6.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(p.device, "-", p.description)
        return 0
    if args.demo and args.replay:
        print("Use either --demo or --replay, not both.")
        return 1

    outdir = Path(args.outdir)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.last is None:
        args.last = 0.0 if (args.dump or args.dump_file) else 5.0
    if not 0 < args.fc < SAMPLE_HZ / 2:
        print(f"--fc must be between 0 and {SAMPLE_HZ // 2} Hz")
        return 1
    if args.dump or args.dump_file or args.info:
        return dump_main(args, outdir, stamp)
    sess = Session(args.label, outdir, stamp, args.last, args.fc)
    shared = Shared(args.window, args.baud)
    shared.label = args.label
    stop = threading.Event()
    raw_log = None

    if args.replay:
        source = replay_lines(args.replay, not args.no_plot, stop)
        shared.set_conn(True, f"replay of {Path(args.replay).name}")
    elif args.demo:
        source = demo_lines(stop)
        shared.set_conn(True, "demo (simulated board)")
    else:
        print(f"Label = '{args.label}'. Close the window or press Ctrl+C to stop and save.")
        source = serial_lines(args, shared, stop, gui_dump_saver(args, shared, outdir))
        outdir.mkdir(parents=True, exist_ok=True)
        raw_log = open(outdir / f"raw_{stamp}.log", "w", encoding="ascii")

    worker = threading.Thread(target=reader_main, args=(source, sess, shared, raw_log, stop),
                              daemon=True)
    try:
        if args.no_plot:
            reader_main(source, sess, shared, raw_log, stop)
        else:
            worker.start()
            run_gui(args, shared, stop, outdir)
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        stop.set()
        if worker.is_alive():
            worker.join(timeout=3)
        sess.label = shared.label
        sess.write_csv()
        if raw_log:
            raw_log.close()
        if shared.bad:
            print(f"({shared.bad} unparseable lines skipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
