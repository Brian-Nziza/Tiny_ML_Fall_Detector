#!/usr/bin/env python3
"""
fall_logger.py - STM32 fall recorder: live graph + labelled CSV.

What it does
    * Reads the accelerometer stream from the Nucleo (ST-LINK virtual COM port).
    * Draws a live graph of the total acceleration |a| (white background, grid,
      green line, local-time axis) with a connection status banner.
    * Each press of the "end of recording" button becomes ONE ROW in the CSV:
      all samples since the previous press. The graph marks every press.
    * The "end of session" button writes the CSV and resets the counter.
      Closing the window or Ctrl+C also writes whatever is pending.
    * If the USB cable is pulled the script keeps running, shows NO CONNECTION
      and reconnects by itself when the board is back.

Usage
    python fall_logger.py --label fall
    python fall_logger.py --port COM5 --label walk --last 4 --window 15
    python fall_logger.py --demo                      # simulated board, no hardware
    python fall_logger.py --replay recordings/raw_20261106_101500.log --label fall
    python fall_logger.py --replay raw.log --no-plot  # convert a log to CSV, fast
    python fall_logger.py --list-ports

Needs:  pip install pyserial matplotlib
"""
import argparse
import csv
import math
import os
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

MG_PER_COUNT = 72          # LIS35DE typical sensitivity at +-8 g (FS = 1)
COUNT_CLIP = 127           # int8 limit; |count| >= this means the sensor saturated
GAP_US = 6000              # sample spacing above this = missed samples (nominal 2500)
WRAP = 1 << 32             # the firmware timestamp is a 32-bit microsecond counter
SAMPLE_HZ = 400            # nominal output data rate set in the firmware
STALE_S = 1.0              # no valid sample for this long -> "NO DATA"
BREAK = "\x00BREAK"        # internal marker: the serial link was (re)opened


# ---------------------------------------------------------------------------
# CSV session (unchanged behaviour)
# ---------------------------------------------------------------------------
class Session:
    """Collects samples and finished events until a CSV is written."""

    def __init__(self, label, outdir, stamp, last_s):
        self.label, self.outdir, self.stamp, self.last_s = label, outdir, stamp, last_s
        self.index = 0                 # which session this run (for the file name)
        self.events = []               # finished rows
        self.samples = []              # (t_seconds, x, y, z) of the event in progress
        self.t_prev_raw = None         # for unwrapping the 32-bit timestamp
        self.t_us = 0                  # unwrapped time in microseconds
        self.gaps = 0

    def add_sample(self, t_raw, x, y, z):
        if self.t_prev_raw is None:
            self.t_us = 0
        else:
            dt = (t_raw - self.t_prev_raw) % WRAP
            self.t_us += dt
            if dt > GAP_US:
                self.gaps += 1
        self.t_prev_raw = t_raw
        self.samples.append((self.t_us / 1e6, x, y, z))

    def end_event(self, rec_id):
        """Close the current recording. Returns the stored event, or None if empty."""
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
        peak = max(math.sqrt(x * x + y * y + z * z) for _, x, y, z in s) * MG_PER_COUNT / 1000
        clipped = sum(1 for _, x, y, z in s if max(abs(x), abs(y), abs(z)) >= COUNT_CLIP)
        dur = s[-1][0] - s[0][0]
        self.events.append((rec_id, s, dur, peak, clipped))
        warn = f"  <-- {clipped} samples clipped at the +-8 g limit" if clipped else ""
        cut = f"  (dropped first {trimmed:.1f} s)" if trimmed > 0.01 else ""
        print(f"recording {rec_id:03d}: {len(s)} samples, {dur:.2f} s, peak {peak:.2f} g{warn}{cut}")
        return self.events[-1]

    def write_csv(self):
        if not self.events:
            self.samples = []
            self.t_prev_raw = None
            return None
        self.index += 1
        self.outdir.mkdir(parents=True, exist_ok=True)
        path = self.outdir / f"{self.label}_{self.stamp}_s{self.index}.csv"
        longest = max(len(e[1]) for e in self.events)
        header = (["recording", "label", "n_samples", "duration_s", "peak_g", "clipped"]
                  + [f"{axis}{i}" for i in range(longest) for axis in "xyz"])
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for rec_id, s, dur, peak, clipped in self.events:
                vals = []
                for _, x, y, z in s:
                    vals += [round(x * MG_PER_COUNT / 1000, 3),
                             round(y * MG_PER_COUNT / 1000, 3),
                             round(z * MG_PER_COUNT / 1000, 3)]
                w.writerow([rec_id, self.label, len(s), round(dur, 3),
                            round(peak, 3), clipped] + vals)
        print(f"--> wrote {len(self.events)} rows to {path}")
        if self.gaps:
            print(f"    note: {self.gaps} gaps in the sample stream (dt > {GAP_US} us)")
        self.events, self.samples, self.gaps = [], [], 0
        self.t_prev_raw = None
        return path


# ---------------------------------------------------------------------------
# State shared between the reader thread and the plot
# ---------------------------------------------------------------------------
class Shared:
    def __init__(self, window_s, baud):
        self.lock = threading.Lock()
        self.baud = baud
        self.pts = deque(maxlen=int(window_s * SAMPLE_HZ * 1.3) + 800)   # (wall_s, g)
        self.rx = deque(maxlen=4000)        # monotonic arrival time of each valid sample
        self.marks = deque(maxlen=100)      # (wall_s, "E" | "R", text)
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

    # -- called from the reader thread --------------------------------------
    def set_conn(self, is_open, note):
        with self.lock:
            changed = (note != self.conn_note) or (is_open != self.port_open)
            self.port_open, self.conn_note = is_open, note
        if changed:
            print(f"[link] {note}")

    def add_break(self):
        with self.lock:
            self.pts.append((time.time(), float("nan")))

    def add_point(self, wall, g):
        with self.lock:
            self.pts.append((wall, g))
            self.rx.append(time.monotonic())
            self.last_sample_mono = self.rx[-1]
            self.last_g = g
            self.n_samples += 1

    def add_event(self, wall, rec_id, ev):
        with self.lock:
            self.marks.append((wall, "E", f"REC {rec_id:03d}"))
            self.rec_id = rec_id
            if ev:
                _, s, dur, peak, clipped = ev
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

    # -- called from the plot ------------------------------------------------
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


# ---------------------------------------------------------------------------
# Line sources: serial (auto-reconnect), replay file, built-in demo
# ---------------------------------------------------------------------------
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


def serial_lines(args, shared, stop):
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
        try:
            while not stop.is_set():
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    yield raw.decode("ascii", errors="replace").strip()
        except (serial.SerialException, OSError) as e:
            shared.set_conn(False, f"lost connection on {port}: {short_err(e)}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
        stop.wait(1.0)


def replay_lines(path, pace, stop):
    """Yield lines from a raw log. With pace=True, play back in real time."""
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
    """Fake accelerometer counts for second c of a 9 s cycle: stand, fall at 3 s, lie."""
    n = lambda: rnd.randint(-1, 1)
    if c < 3.0:                                   # standing, gentle sway
        sway = 3 * math.sin(2 * math.pi * 1.8 * c)
        return n(), n() + int(sway * 0.3), 14 + n() + int(sway)
    if c < 3.4:                                   # free fall: near 0 g
        return n(), n(), 2 + n()
    if c < 3.45:                                  # impact
        return 25 + n() * 3, n() * 3, 70 + rnd.randint(-10, 10)
    if c < 3.9:                                   # bounce, decaying
        k = c - 3.45
        return 12 + n(), n(), int(22 * math.exp(-k * 7) * math.sin(45 * k)) + 3 + n()
    return 14 + n(), n(), n()                     # lying on the side


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
            if k == 7 * SAMPLE_HZ:                # event button, 4 s after the fall
                rec += 1
                yield f"E,{rec},{t}"
            elif k == 7 * SAMPLE_HZ + 100 and rec == 3:   # reset button after 3 recordings
                rec = 0
                yield f"R,{t}"
            emitted += 1
        time.sleep(0.005)


# ---------------------------------------------------------------------------
# Reader: parse lines, feed the CSV session and the plot state
# ---------------------------------------------------------------------------
def run_reader(source, sess, shared, raw_log, stop):
    anchor, last_wall, last_flush = None, 0.0, time.monotonic()
    for line in source:
        if stop.is_set():
            break
        if raw_log:
            raw_log.write(("#LINK,reopened" if line == BREAK else line) + "\n")
            if time.monotonic() - last_flush > 1.0:
                raw_log.flush()
                last_flush = time.monotonic()

        if line == BREAK or line.startswith(("#BOOT", "#LINK")):   # link reopened / board restarted
            if line.startswith("#BOOT"):
                print(line)
                shared.sensor_err = None
            if sess.samples:
                print("link/board restarted - discarding the unfinished recording")
            if sess.events:                                 # the board's counter restarts at 0,
                print("link/board restarted - saving the session so far")   # so close the session
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
            continue

        parts = line.split(",")
        try:
            if parts[0] == "S" and len(parts) == 5:
                t, x, y, z = (int(p) for p in parts[1:])
                fresh = sess.t_prev_raw is None
                gaps_before = sess.gaps
                sess.add_sample(t, x, y, z)
                if sess.gaps > gaps_before:
                    shared.bump("gaps")
                if fresh or anchor is None:                 # map device time to the PC clock
                    anchor = max(time.time(), last_wall + 1.0 / SAMPLE_HZ) - sess.t_us / 1e6
                last_wall = anchor + sess.t_us / 1e6
                g = math.sqrt(x * x + y * y + z * z) * MG_PER_COUNT / 1000
                shared.add_point(last_wall, g)
            elif parts[0] == "E" and len(parts) == 3:
                rec_id = int(parts[1])
                ev = sess.end_event(rec_id)
                shared.add_event(last_wall or time.time(), rec_id, ev)
            elif parts[0] == "R" and len(parts) == 2:
                print("session reset button -> writing CSV")
                path = sess.write_csv()
                anchor = None
                shared.add_reset(last_wall or time.time(), path)
            else:
                shared.bump("bad")
        except ValueError:
            shared.bump("bad")                              # truncated / garbled line


def reader_main(source, sess, shared, raw_log, stop):
    try:
        run_reader(source, sess, shared, raw_log, stop)
    except Exception as e:                                  # show it in the window, too
        shared.fatal = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        if not stop.is_set():                               # source ran out (replay finished)
            shared.set_conn(False, "stream ended (replay finished)")


# ---------------------------------------------------------------------------
# Live plot
# ---------------------------------------------------------------------------
INK, MUTED, GRID, GRID_MINOR = "#1f2328", "#656d76", "#d0d7de", "#ebeef1"
GREEN = "#16a34a"
STATUS_STYLE = {                     # symbol + word + colour, never colour alone
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
    matplotlib.rcParams["path.simplify_threshold"] = 1.0    # merge points < 1 px apart: faster, invisible
    base = math.floor(time.time())          # plot x = seconds since start (keeps the numbers small)
    fig = plt.figure(figsize=(11, 6.4), facecolor="white")
    canvas = fig.canvas
    try:
        canvas.manager.set_window_title("Fall recorder - live accelerometer")
    except Exception:
        pass

    # ---- static parts: drawn once, then cached as a bitmap -------------------
    ax = fig.add_axes([0.075, 0.235, 0.9, 0.595], facecolor="white")
    ax.set_title("Total acceleration |a| = sqrt(x²+y²+z²)      dashed line = 1 g (board at rest)",
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
    ax.tick_params(axis="x", which="both", bottom=False, labelbottom=False)   # x labels are drawn by hand
    ax.set_ylabel("|a|  (g)", color=MUTED, fontsize=10)
    fig.text(0.5, 0.165, "Time (local clock, HH:MM:SS)", ha="center", va="center",
             fontsize=10, color=MUTED)
    ax.set_xlim(0, args.window)
    ax.set_ylim(0, 2)

    # ---- moving parts: redrawn every frame on top of the cached bitmap ------
    xt = ax.get_xaxis_transform()           # x in data units, y as a fraction of the axes height
    anim = []

    def moving(artist):
        artist.set_animated(True)
        anim.append(artist)
        return artist

    (vgrid,) = ax.plot([], [], color=GRID, lw=0.8, transform=xt, zorder=1)   # one line, NaN-separated
    moving(vgrid)
    tick_txt = [moving(ax.text(0, -0.015, "", transform=xt, ha="center", va="top",
                               fontsize=9, color=MUTED, clip_on=False)) for _ in range(14)]
    mk_line = [moving(ax.axvline(0, lw=1.1, zorder=2, visible=False)) for _ in range(12)]
    mk_text = [moving(ax.text(0, 0.985, "", transform=xt, ha="right", va="top", fontsize=9,
                              fontweight="bold", zorder=4, visible=False,
                              bbox=dict(facecolor="white", edgecolor="none", pad=1.5)))
               for _ in range(12)]
    (line,) = ax.plot([], [], color=GREEN, lw=1.5, solid_joinstyle="round", zorder=3)
    moving(line)
    overlay = moving(ax.text(0.5, 0.80, "", transform=ax.transAxes, ha="center", va="center",
                             fontsize=20, fontweight="bold", zorder=5,
                             bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=6)))
    head = moving(fig.text(0.075, 0.945, "", fontsize=18, fontweight="bold", va="center"))
    detail = moving(fig.text(0.075, 0.895, "", fontsize=10, color=MUTED, va="center"))
    clock = moving(fig.text(0.975, 0.945, "", fontsize=12, color=INK, ha="right", va="center"))
    foot1 = moving(fig.text(0.075, 0.115, "", fontsize=10, color=INK, va="center"))
    fig.text(0.075, 0.075, f"Label '{args.label}'   \u00b7   CSV folder: {outdir.resolve()}",
             fontsize=9, color=MUTED, va="center")          # constant, so part of the cached bitmap
    fig.text(0.075, 0.04, "Event button = end recording   \u00b7   Reset button = save CSV and reset "
             "to 000   \u00b7   closing this window also saves", fontsize=9, color=MUTED, va="center")

    bg = [None]                              # cached bitmap of everything static

    def on_draw(_event=None):                # after every full redraw (start, resize, y-scale change)
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
            st["fps"] = 0.9 * st["fps"] + 0.1 / (tp - st["t_prev"])      # smoothed frames per second
        st["t_prev"] = tp
        arr = np.array(s["pts"], dtype=float) if s["pts"] else np.empty((0, 2))
        right = max(now, arr[-1, 0]) if len(arr) else now
        left = right - args.window
        ax.set_xlim(left - base, right - base)

        vis_max = 0.0
        if len(arr):
            a = arr[arr[:, 0] >= left - 0.5]
            line.set_data(a[:, 0] - base, a[:, 1])
            if np.isfinite(a[:, 1]).any():
                vis_max = float(np.nanmax(a[:, 1]))
        else:
            line.set_data([], [])
        ytop = max(2, math.ceil(vis_max * 1.1))
        if ytop != st["ytop"]:               # rare: the y scale changes, so redraw the static part
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
        clock.set_text(datetime.now().strftime("%a %Y-%m-%d   %H:%M:%S"))
        foot1.set_text(f"Recordings this session: {s['rec']:03d}     last: {s['last_event']}")

        canvas.restore_region(bg[0])         # paint the cached bitmap, then only the moving parts
        for art in anim:
            fig.draw_artist(art)
        canvas.blit(fig.bbox)

    def safe_update():
        try:
            update()
        except Exception as e:               # never let one bad frame kill the timer
            msg = f"{type(e).__name__}: {e}"
            if msg != st["err"]:
                st["err"] = msg
                traceback.print_exc()

    if args.snapshot:                                   # headless: render to a PNG and exit
        canvas.draw()
        t_start = time.monotonic()
        tt, nfr, worst = 0.0, 0, 0.0
        while not stop.is_set() and time.monotonic() - t_start < args.snapshot_after:
            time.sleep(0.02)
            a0 = time.perf_counter()
            update()
            dt = time.perf_counter() - a0
            tt, nfr, worst = tt + dt, nfr + 1, max(worst, dt)
        if os.environ.get("BENCH") and nfr:
            print(f"BENCH frames={nfr}  per-frame={tt / nfr * 1000:.1f} ms  worst={worst * 1000:.1f} ms")
        for art in anim:
            art.set_animated(False)
        fig.savefig(args.snapshot, dpi=110, facecolor="white")
        stop.set()
        return

    timer = canvas.new_timer(interval=33)               # ~30 frames per second
    timer.add_callback(safe_update)
    timer.start()
    fig._timer_keepalive = timer
    plt.show()


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (COM5, /dev/ttyACM0 ...); auto-detected if omitted")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--label", default="unlabeled", help="label written into every row of this run")
    ap.add_argument("--outdir", default="recordings")
    ap.add_argument("--last", type=float, default=5.0,
                    help="keep only the last N seconds before each press (default 5; 0 = keep "
                         "everything, but long rows exceed Excel's 16384-column limit)")
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history shown in the graph (default 10)")
    ap.add_argument("--no-plot", action="store_true", help="no graph window, console + CSV only")
    ap.add_argument("--replay", help="parse a saved raw log instead of opening a serial port")
    ap.add_argument("--demo", action="store_true", help="simulated board, no hardware needed")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--snapshot", help=argparse.SUPPRESS)           # test hook: save a PNG and exit
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
    sess = Session(args.label, outdir, stamp, args.last)
    shared = Shared(args.window, args.baud)
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
        source = serial_lines(args, shared, stop)
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
        sess.write_csv()
        if raw_log:
            raw_log.close()
        if shared.bad:
            print(f"({shared.bad} unparseable lines skipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())