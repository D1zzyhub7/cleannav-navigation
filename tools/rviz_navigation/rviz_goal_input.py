#!/usr/bin/env python3
"""Safely probe the live RViz X11 window and optionally drag its Goal tool.

This sends ordinary XTest mouse events only. It never publishes ROS messages.
The operator must select 2D Goal Pose and confirm the map coordinates first.
"""

from __future__ import annotations

import argparse
import ctypes as C
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


class InputError(RuntimeError):
    pass


@dataclass(frozen=True)
class Window:
    window_id: int
    title: str
    viewable: bool


_WINDOW_LINE = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"')
_WINDOW_HEADER = re.compile(r'Window id:\s*(0x[0-9a-fA-F]+)(?:\s+"([^"]*)")?')


def _run_xwininfo(*args: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            ["xwininfo", *args], capture_output=True, text=True, timeout=2, check=False
        )
    except FileNotFoundError as exc:
        raise InputError("xwininfo is required to verify the RViz window") from exc
    except subprocess.TimeoutExpired as exc:
        raise InputError("xwininfo timed out while checking the RViz window") from exc


def _run_xprop(*args: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            ["xprop", *args], capture_output=True, text=True, timeout=2, check=False
        )
    except FileNotFoundError as exc:
        raise InputError("xprop is required to identify the managed RViz window") from exc
    except subprocess.TimeoutExpired as exc:
        raise InputError("xprop timed out while checking the RViz window") from exc


def rviz_windows_from_tree(output: str) -> list[tuple[int, str]]:
    """Return titled RViz windows from `xwininfo -root -tree` output."""
    windows = []
    for line in output.splitlines():
        match = _WINDOW_LINE.match(line)
        if match and "rviz" in match.group(2).lower():
            windows.append((int(match.group(1), 16), match.group(2)))
    return windows


def has_window_manager_state(window_id: int) -> bool:
    """Exclude RViz child and Qt helper windows; accept managed top-level windows."""
    result = _run_xprop("-id", hex(window_id), "WM_STATE")
    return result.returncode == 0 and bool(
        re.search(r"^WM_STATE\(WM_STATE\):", result.stdout, re.MULTILINE)
    )


def focus_belongs_to_window(focus: int, window_id: int) -> bool:
    """Accept the RViz top-level XID or one of its live child windows."""
    current = focus
    visited = set()
    for _ in range(16):
        if current == window_id:
            return True
        if current == 0 or current in visited:
            return False
        visited.add(current)
        result = _run_xwininfo("-all", "-id", hex(current))
        if result.returncode != 0:
            return False
        parent = re.search(r"Parent window id:\s*(0x[0-9a-fA-F]+)", result.stdout)
        if not parent:
            return False
        current = int(parent.group(1), 16)
    return False


def discover_rviz_window() -> Window:
    tree = _run_xwininfo("-root", "-tree")
    if tree.returncode != 0:
        raise InputError(f"cannot enumerate X11 windows: {tree.stderr.strip()}")
    candidates = rviz_windows_from_tree(tree.stdout)
    if not candidates:
        raise InputError("no RViz-titled X11 window is currently present")
    visible = []
    for window_id, _title in candidates:
        if not has_window_manager_state(window_id):
            continue
        try:
            visible.append(inspect_rviz_window(window_id))
        except InputError as exc:
            # Qt may leave an unmapped popup-menu XID in the root tree. It is
            # not an input target and must not make the live RViz window
            # ambiguous; real viewable RViz windows remain subject to the
            # ambiguity check below.
            if "is not viewable" not in str(exc):
                raise
    if not visible:
        raise InputError("no managed RViz top-level X11 window is currently present")
    if len(visible) != 1:
        found = ", ".join(f"{window.title!r} ({hex(window.window_id)})" for window in visible)
        raise InputError(f"multiple RViz windows found; close ambiguity and retry: {found}")
    return visible[0]


def inspect_rviz_window(window_id: int) -> Window:
    result = _run_xwininfo("-id", hex(window_id))
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise InputError(
            f"RViz window {hex(window_id)} is no longer valid or queryable; "
            f"re-enumerate the live window ({detail})"
        )
    header = _WINDOW_HEADER.search(result.stdout)
    if not header:
        raise InputError(f"xwininfo returned no window identity for {hex(window_id)}")
    actual_id = int(header.group(1), 16)
    title = header.group(2) or ""
    if actual_id != window_id or "rviz" not in title.lower():
        raise InputError(
            f"window identity mismatch: requested {hex(window_id)}, "
            f"found {hex(actual_id)} {title!r}"
        )
    viewable = bool(re.search(r"Map State:\s+IsViewable\b", result.stdout))
    if not viewable:
        raise InputError(f"RViz window {hex(window_id)} {title!r} is not viewable")
    return Window(actual_id, title, viewable)


def input_focus() -> int:
    x11 = C.CDLL("libX11.so.6")
    x11.XOpenDisplay.argtypes = [C.c_char_p]
    x11.XOpenDisplay.restype = C.c_void_p
    x11.XGetInputFocus.argtypes = [C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)]
    x11.XGetInputFocus.restype = C.c_int
    display = x11.XOpenDisplay(None)
    if not display:
        raise InputError("XOpenDisplay failed; no usable X11 display")
    focus, revert = C.c_ulong(), C.c_int()
    try:
        x11.XGetInputFocus(display, C.byref(focus), C.byref(revert))
        x11.XSync.argtypes = [C.c_void_p, C.c_int]
        x11.XSync(display, 0)
    finally:
        x11.XCloseDisplay.argtypes = [C.c_void_p]
        x11.XCloseDisplay(display)
    return int(focus.value)


def checked_live_focus() -> tuple[Window, int]:
    # Resolve the XID afresh for every check; XIDs from a previous RViz process
    # are not treated as durable window identities.
    window = discover_rviz_window()
    focus = input_focus()
    if not focus_belongs_to_window(focus, window.window_id):
        raise InputError(
            f"RViz window {hex(window.window_id)} exists but is not active "
            f"(X11 focus={hex(focus)}); focus RViz and probe again"
        )
    return window, focus


def drag_goal(start: tuple[int, int], end: tuple[int, int], steps: int) -> dict:
    if not 1 <= steps <= 30:
        raise InputError("--steps must be in 1..30")
    window, focus = checked_live_focus()

    x11 = C.CDLL("libX11.so.6")
    xtst = C.CDLL("libXtst.so.6")
    x11.XOpenDisplay.argtypes = [C.c_char_p]
    x11.XOpenDisplay.restype = C.c_void_p
    x11.XSync.argtypes = [C.c_void_p, C.c_int]
    x11.XDefaultScreen.argtypes = [C.c_void_p]
    x11.XDefaultScreen.restype = C.c_int
    x11.XCloseDisplay.argtypes = [C.c_void_p]
    xtst.XTestFakeMotionEvent.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int, C.c_ulong]
    xtst.XTestFakeMotionEvent.restype = C.c_int
    xtst.XTestFakeButtonEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
    xtst.XTestFakeButtonEvent.restype = C.c_int
    display = x11.XOpenDisplay(None)
    if not display:
        raise InputError("XOpenDisplay failed; no usable X11 display")
    screen = x11.XDefaultScreen(display)

    def ensure_same_live_target() -> None:
        current, _ = checked_live_focus()
        if current.window_id != window.window_id:
            raise InputError("RViz window changed during goal drag")

    def move(x: int, y: int) -> bool:
        return bool(xtst.XTestFakeMotionEvent(display, screen, x, y, 0))

    def button(down: bool) -> bool:
        return bool(xtst.XTestFakeButtonEvent(display, 1, int(down), 0))

    pressed = False
    try:
        ensure_same_live_target()
        if not move(*start):
            raise InputError("XTest pointer move to Goal start failed")
        x11.XSync(display, 0)
        time.sleep(0.12)
        ensure_same_live_target()
        if not button(True):
            raise InputError("XTest Button1 press failed")
        pressed = True
        for step in range(1, steps + 1):
            ensure_same_live_target()
            x = round(start[0] + (end[0] - start[0]) * step / steps)
            y = round(start[1] + (end[1] - start[1]) * step / steps)
            if not move(x, y):
                raise InputError("XTest drag motion failed")
            x11.XSync(display, 0)
            time.sleep(0.025)
        ensure_same_live_target()
        if not button(False):
            raise InputError("XTest Button1 release failed")
        pressed = False
        x11.XSync(display, 0)
    finally:
        if pressed:
            button(False)
            x11.XSync(display, 0)
        x11.XCloseDisplay(display)

    return {
        "sent_at_utc": datetime.now(timezone.utc).isoformat(),
        "window_id": hex(window.window_id),
        "window_title": window.title,
        "focus_readback": hex(focus),
        "start_xy": list(start),
        "end_xy": list(end),
        "steps": steps,
        "event_source": "XTestFakeMotionEvent/ButtonEvent",
        "ros_client_or_topic_publish": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-only", action="store_true", help="check live RViz identity/focus; send no input")
    parser.add_argument("--start", nargs=2, type=int)
    parser.add_argument("--end", nargs=2, type=int)
    parser.add_argument("--steps", type=int, default=8)
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.probe_only:
            if args.start is not None or args.end is not None:
                raise InputError("--probe-only cannot be combined with drag coordinates")
            window, focus = checked_live_focus()
            print(json.dumps({"probe": "PASS", "window_id": hex(window.window_id), "title": window.title,
                              "viewable": window.viewable, "focus": hex(focus), "input_sent": False}))
            return 0
        if args.start is None or args.end is None or args.receipt is None:
            raise InputError("drag requires --start, --end and --receipt")
        receipt = drag_goal(tuple(args.start), tuple(args.end), args.steps)
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        args.receipt.write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(receipt))
        return 0
    except InputError as exc:
        print(f"RViz GUI input check failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
