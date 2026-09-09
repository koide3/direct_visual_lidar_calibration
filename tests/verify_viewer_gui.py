#!/usr/bin/env python3
"""Exercise the actual GTK viewer in a disposable Xvfb display and child process."""
import argparse
import ctypes
import ctypes.util
from hashlib import sha256
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from unittest.mock import patch

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dual_mei.viewer import show_result


def exercise(data):
    data = Path(data)
    config_before = (data / "extrinsics.yaml").read_bytes()
    overlay_before = (data / "cam1/overlay_after.png").read_bytes()
    original_imshow, original_wait, original_mouse = cv2.imshow, cv2.waitKey, cv2.setMouseCallback
    state = dict(frame=0, hashes=[], mouse=None, window=None, native_events=[])

    def mouse(window, callback, *args):
        state["mouse"] = callback
        state["window"] = window
        def observed(event, x, y, flags, userdata):
            state["native_events"].append(event)
            return callback(event, x, y, flags, userdata)
        return original_mouse(window, observed, *args)

    def imshow(window, canvas):
        state["hashes"].append(sha256(canvas.tobytes()).hexdigest())
        assert canvas.shape == (900, 1280, 3)
        original_imshow(window, canvas)

    def wait(milliseconds):
        original_wait(20)
        frame = state["frame"]
        state["frame"] += 1
        callback = state["mouse"]
        if frame == 0:
            x11 = ctypes.CDLL(ctypes.util.find_library("X11"))
            xtst = ctypes.CDLL(ctypes.util.find_library("Xtst"))
            x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
            x11.XOpenDisplay.restype = ctypes.c_void_p
            x11.XFlush.argtypes = [ctypes.c_void_p]
            x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
            x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
            x11.XStringToKeysym.restype = ctypes.c_ulong
            x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            x11.XKeysymToKeycode.restype = ctypes.c_uint
            xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
            xtst.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
            xtst.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
            display = x11.XOpenDisplay(None)
            assert display
            try:
                xtst.XTestFakeMotionEvent(display, -1, 700, 400, 0)
                xtst.XTestFakeButtonEvent(display, 1, 1, 0)
                xtst.XTestFakeMotionEvent(display, -1, 740, 430, 0)
                xtst.XTestFakeButtonEvent(display, 1, 0, 0)
                code = x11.XKeysymToKeycode(display, x11.XStringToKeysym(b"equal"))
                xtst.XTestFakeKeyEvent(display, code, 1, 0)
                xtst.XTestFakeKeyEvent(display, code, 0, 0)
                x11.XFlush(display)
                received_key = original_wait(100) & 0xff
            finally:
                x11.XCloseDisplay(display)
            assert cv2.EVENT_LBUTTONDOWN in state["native_events"], state["native_events"]
            assert cv2.EVENT_LBUTTONUP in state["native_events"], state["native_events"]
            assert received_key in (ord("="), ord("+")), received_key
            return received_key
        if frame == 1:
            return ord("b")
        if frame == 2:
            cv2.setTrackbarPos("radius px", state["window"], 6)
            cv2.setTrackbarPos("opacity %", state["window"], 55)
            cv2.setTrackbarPos("zoom %", state["window"], 200)
            return ord("1")
        return {3: ord("a"), 4: ord("r"), 5: ord("s")}.get(frame, ord("q"))

    with patch.object(cv2, "imshow", imshow), patch.object(cv2, "waitKey", wait), \
            patch.object(cv2, "setMouseCallback", mouse):
        show_result(data)
    assert state["frame"] == 7, state["frame"]
    assert len(set(state["hashes"])) >= 4
    assert (data / "extrinsics.yaml").read_bytes() == config_before
    assert (data / "cam1/overlay_after.png").read_bytes() != overlay_before
    assert cv2.imread(str(data / "cam1/overlay_after.png")).shape == (3840, 3840, 3)
    print(json.dumps(dict(status="passed", frames=state["frame"],
                          actions=["zoom", "pan", "camera_switch", "before_after",
                                   "radius", "opacity", "reset", "save", "quit"],
                          extrinsics_unchanged=True, original_result_unchanged=True,
                          native_mouse_drag_and_zoom_key_received=True)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", required=True)
    parser.add_argument("--xvfb")
    parser.add_argument("--exercise", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.exercise:
        exercise(args.result)
        return
    if not args.xvfb:
        parser.error("--xvfb is required")
    with tempfile.TemporaryDirectory(prefix="dual_mei_gui_") as temp:
        temp = Path(temp)
        data = temp / "result"
        shutil.copytree(args.result, data)
        read_fd, write_fd = os.pipe()
        with (temp / "xvfb.txt").open("w") as log:
            server = subprocess.Popen([args.xvfb, "-displayfd", str(write_fd), "-screen", "0",
                                       "1400x1100x24", "-nolisten", "tcp"], pass_fds=(write_fd,),
                                      stdout=log, stderr=subprocess.STDOUT)
        os.close(write_fd)
        try:
            with os.fdopen(read_fd) as stream:
                display = stream.readline().strip()
            if not display:
                raise RuntimeError((temp / "xvfb.txt").read_text())
            # Exit GTK completely before stopping its X server; otherwise XIO can
            # turn an otherwise successful test into exit=1 during interpreter teardown.
            env = dict(os.environ, DISPLAY=":" + display)
            subprocess.run([sys.executable, "-B", __file__, "--result", str(data), "--exercise"],
                           check=True, env=env, timeout=60)
        finally:
            server.terminate()
            server.wait(timeout=10)


if __name__ == "__main__":
    main()
