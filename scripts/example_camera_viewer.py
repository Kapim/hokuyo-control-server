#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import base64
import os
import sys
import time

import numpy as np

try:
    import cv2
    _CV2_IMPORT_ERROR = None
except Exception as exc:
    cv2 = None
    _CV2_IMPORT_ERROR = exc

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from http_api_client import RobotHttpClient


def _ensure_bgr(img):
    if img is None:
        return None
    if len(img.shape) == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _depth_to_display(depth):
    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    if depth.size == 0:
        return None
    non_zero = depth[depth > 0]
    if non_zero.size == 0:
        return np.zeros(depth.shape, dtype=np.uint8)
    lo = float(np.percentile(non_zero, 5))
    hi = float(np.percentile(non_zero, 95))
    if hi <= lo:
        hi = lo + 1.0
    scaled = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    return (scaled * 255.0).astype(np.uint8)


def decode_frame(camera_payload):
    if not camera_payload:
        return None, "no camera payload"

    encoding = camera_payload.get("encoding")
    data_b64 = camera_payload.get("data_base64")
    if not data_b64:
        return None, "missing data_base64"

    try:
        raw_bytes = base64.b64decode(data_b64)
    except Exception as exc:
        return None, "base64 decode failed: %s" % exc

    if encoding == "compressed":
        arr = np.frombuffer(raw_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if img is None:
            return None, "compressed decode returned None"
        return _ensure_bgr(img), None

    if encoding == "raw":
        width = int(camera_payload.get("width", 0))
        height = int(camera_payload.get("height", 0))
        step = int(camera_payload.get("step", 0))
        fmt = camera_payload.get("format", "")
        is_bigendian = int(camera_payload.get("is_bigendian", 0))
        if width <= 0 or height <= 0 or step <= 0:
            return None, "invalid raw geometry"

        arr = np.frombuffer(raw_bytes, dtype=np.uint8)
        if arr.size < height * step:
            return None, "raw buffer too short"
        arr = arr[: height * step].reshape((height, step))

        if fmt == "mono8":
            mono = arr[:, :width]
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR), None

        if fmt == "rgb8":
            rgb = arr[:, : width * 3].reshape((height, width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), None

        if fmt == "bgr8":
            return arr[:, : width * 3].reshape((height, width, 3)), None

        if fmt == "rgba8":
            rgba = arr[:, : width * 4].reshape((height, width, 4))
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR), None

        if fmt == "bgra8":
            bgra = arr[:, : width * 4].reshape((height, width, 4))
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR), None

        if fmt in ("16UC1", "mono16"):
            dtype = np.dtype(">u2" if is_bigendian else "<u2")
            depth = np.frombuffer(raw_bytes, dtype=dtype)
            depth = depth[: width * height].reshape((height, width)).astype(np.float32)
            disp = _depth_to_display(depth)
            return cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR), None

        if fmt == "32FC1":
            dtype = np.dtype(">f4" if is_bigendian else "<f4")
            depth = np.frombuffer(raw_bytes, dtype=dtype)
            depth = depth[: width * height].reshape((height, width))
            disp = _depth_to_display(depth)
            return cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR), None

        if fmt == "8UC1":
            mono = arr[:, :width]
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR), None

        if fmt == "8UC3":
            return arr[:, : width * 3].reshape((height, width, 3)), None

        if fmt == "8UC4":
            bgra = arr[:, : width * 4].reshape((height, width, 4))
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR), None

        # Fallback by guessed channel count from step.
        channels = int(step / max(1, width))
        if channels == 1:
            mono = arr[:, :width]
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR), None
        if channels == 3:
            return arr[:, : width * 3].reshape((height, width, 3)), None
        if channels == 4:
            bgra = arr[:, : width * 4].reshape((height, width, 4))
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR), None

        return None, "unsupported raw format '%s'" % fmt

    return None, "unsupported encoding '%s'" % encoding


def _status_frame(width, height, lines):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    y = 40
    for line in lines:
        cv2.putText(frame, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA)
        y += 28
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True, help="Base server URL, e.g. http://127.0.0.1:8080")
    parser.add_argument("--fps", type=float, default=10.0, help="Polling rate for camera endpoint")
    parser.add_argument("--window", default="Robot Camera")
    args = parser.parse_args()

    if cv2 is None:
        print("OpenCV import failed for Python %s" % sys.version.split()[0])
        print("Error: %s" % _CV2_IMPORT_ERROR)
        print("Fix options:")
        print("1) Install OpenCV for this Python interpreter.")
        print("2) Run the script with an interpreter version that has cv2.")
        print("Example install command:")
        print("  python3 -m pip install opencv-python")
        sys.exit(2)

    poll_hz = max(1.0, float(args.fps))
    delay = 1.0 / poll_hz

    client = RobotHttpClient(args.url, timeout=3.0)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    last_error = "waiting for camera data..."
    last_meta = {}
    last_log_ts = 0.0

    while True:
        try:
            camera_resp = client.get_camera()
            camera = camera_resp.get("data")
            frame, err = decode_frame(camera)
            if camera:
                last_meta = {
                    "encoding": camera.get("encoding"),
                    "format": camera.get("format"),
                    "width": camera.get("width"),
                    "height": camera.get("height"),
                }
            if err:
                last_error = err
        except Exception as exc:
            print("camera read failed: %s" % exc)
            frame = None
            last_error = "http read failed: %s" % exc

        if frame is not None:
            cv2.putText(frame, "q/ESC = quit", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 240, 20), 2, cv2.LINE_AA)
            cv2.imshow(args.window, frame)
        else:
            info_lines = [
                "No image frame",
                "Reason: %s" % last_error,
                "Meta: enc=%s fmt=%s w=%s h=%s" % (
                    last_meta.get("encoding"),
                    last_meta.get("format"),
                    last_meta.get("width"),
                    last_meta.get("height"),
                ),
                "Check camera topic / format in server params.",
            ]
            cv2.imshow(args.window, _status_frame(960, 320, info_lines))

            now = time.time()
            if now - last_log_ts > 2.0:
                print("camera decode info:", info_lines[1], info_lines[2])
                last_log_ts = now

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        time.sleep(delay)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
