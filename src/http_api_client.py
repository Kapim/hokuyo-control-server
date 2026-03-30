#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import threading

try:
    from urllib import urlencode  # Python 2
    from urllib2 import Request, urlopen
except ImportError:
    from urllib.parse import urlencode  # Python 3
    from urllib.request import Request, urlopen


class RobotHttpClient(object):
    def __init__(self, base_url, timeout=2.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._cmd_vel_stream_lock = threading.Lock()
        self._cmd_vel_stream_linear = 0.0
        self._cmd_vel_stream_angular = 0.0
        self._cmd_vel_stream_period = 0.1
        self._cmd_vel_stream_thread = None
        self._cmd_vel_stream_stop = threading.Event()

    def _request(self, method, path, payload=None, params=None):
        url = self.base_url + path
        if params:
            url += "?" + urlencode(params)

        body = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            body = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"

        request = Request(url, data=body, headers=headers)
        request.get_method = lambda: method

        response = urlopen(request, timeout=self.timeout)
        data = response.read().decode("utf-8")
        if not data:
            return {}
        return json.loads(data)

    def health(self):
        return self._request("GET", "/health")

    def _post_cmd_vel(self, linear_x, angular_z):
        return self._request(
            "POST",
            "/cmd_vel",
            payload={"linear_x": linear_x, "angular_z": angular_z},
        )

    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        response = self._post_cmd_vel(linear_x, angular_z)

        applied_linear = linear_x
        applied_angular = angular_z
        if isinstance(response, dict):
            if "linear_x" in response:
                applied_linear = float(response["linear_x"])
            if "angular_z" in response:
                applied_angular = float(response["angular_z"])

        with self._cmd_vel_stream_lock:
            self._cmd_vel_stream_linear = applied_linear
            self._cmd_vel_stream_angular = applied_angular
            if self._cmd_vel_stream_thread and self._cmd_vel_stream_thread.is_alive():
                return response

            self._cmd_vel_stream_stop.clear()
            self._cmd_vel_stream_thread = threading.Thread(target=self._cmd_vel_stream_loop)
            self._cmd_vel_stream_thread.daemon = True
            self._cmd_vel_stream_thread.start()

        return response

    def _cmd_vel_stream_loop(self):
        while not self._cmd_vel_stream_stop.is_set():
            with self._cmd_vel_stream_lock:
                linear_x = self._cmd_vel_stream_linear
                angular_z = self._cmd_vel_stream_angular
                period = self._cmd_vel_stream_period
            try:
                self._post_cmd_vel(linear_x, angular_z)
            except Exception:
                # Keep trying in the background until the stream is stopped.
                pass
            if self._cmd_vel_stream_stop.wait(max(0.01, period)):
                break

    def stop_cmd_vel_stream(self, send_stop=True, wait_timeout=1.0):
        self._cmd_vel_stream_stop.set()
        thread = self._cmd_vel_stream_thread
        if thread and thread.is_alive():
            thread.join(max(0.0, float(wait_timeout)))
        self._cmd_vel_stream_thread = None
        if send_stop:
            return self._post_cmd_vel(0.0, 0.0)
        return {"ok": True}

    def close(self):
        try:
            self.stop_cmd_vel_stream(send_stop=True, wait_timeout=0.2)
        except Exception:
            pass

    def __del__(self):
        try:
            self.stop_cmd_vel_stream(send_stop=False, wait_timeout=0.05)
        except Exception:
            pass

    def get_lidar(self):
        return self._request("GET", "/sensors/lidar")

    @staticmethod
    def _is_finite(value):
        return not (math.isnan(value) or math.isinf(value))

    @staticmethod
    def _angle_diff_deg(a, b):
        diff = (a - b + 180.0) % 360.0 - 180.0
        return abs(diff)

    def _sector_min_distance(self, scan_data, center_deg, width_deg):
        ranges = scan_data.get("ranges") or []
        if not ranges:
            return None

        angle_min = float(scan_data.get("angle_min", 0.0))
        angle_increment = float(scan_data.get("angle_increment", 0.0))
        range_min = float(scan_data.get("range_min", 0.0))
        range_max = float(scan_data.get("range_max", 1e9))
        if angle_increment == 0.0:
            return None

        half_width = width_deg * 0.5
        best = None
        for i, dist in enumerate(ranges):
            if dist is None:
                continue
            dist = float(dist)
            if not self._is_finite(dist):
                continue
            if dist < range_min or dist > range_max:
                continue
            angle_deg = math.degrees(angle_min + i * angle_increment)
            if self._angle_diff_deg(angle_deg, center_deg) > half_width:
                continue
            best = dist if best is None else min(best, dist)
        return best

    def analyze_lidar(self, scan_data):
        if not scan_data:
            return {"error": "no_lidar_data"}

        # 6 contiguous 60-degree sectors covering full 360 degrees.
        front = self._sector_min_distance(scan_data, 0.0, 60.0)
        front_left = self._sector_min_distance(scan_data, 60.0, 60.0)
        front_right = self._sector_min_distance(scan_data, -60.0, 60.0)
        left = self._sector_min_distance(scan_data, 120.0, 60.0)
        right = self._sector_min_distance(scan_data, -120.0, 60.0)
        back = self._sector_min_distance(scan_data, 180.0, 60.0)

        return {
            "front": front,
            "front_left": front_left,
            "front_right": front_right,
            "left": left,
            "right": right,
            "back": back,
        }

    def get_obstacle_report(self):
        lidar = self.get_lidar()
        return self.analyze_lidar(lidar.get("data"))

    def get_camera(self):
        return self._request("GET", "/sensors/camera")

    def get_objects(self):
        return self._request("GET", "/objects")

    def get_battery(self):
        return self._request("GET", "/battery")

    def get_markers(self):
        return self._request("GET", "/markers")

    def get_velocity(self):
        return self._request("GET", "/velocity")

    def set_led(self, led_index, value):
        return self._request(
            "POST",
            "/led",
            payload={"led_index": int(led_index), "value": value},
        )

    def get_led(self):
        return self._request("GET", "/led")

    def play_kobuki_sound(self, value):
        return self._request(
            "POST",
            "/sound",
            payload={"value": value},
        )

    def get_sound(self):
        return self._request("GET", "/sound")

    def say(self, text, command=1):
        return self._request(
            "POST",
            "/tts",
            payload={"text": str(text), "command": int(command)},
        )

    def get_tts(self):
        return self._request("GET", "/tts")
