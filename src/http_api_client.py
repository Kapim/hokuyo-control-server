#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math

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

    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        return self._request(
            "POST",
            "/cmd_vel",
            payload={"linear_x": float(linear_x), "angular_z": float(angular_z)},
        )

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

        front = self._sector_min_distance(scan_data, 0.0, 24.0)
        front_left = self._sector_min_distance(scan_data, 35.0, 24.0)
        front_right = self._sector_min_distance(scan_data, -35.0, 24.0)
        left = self._sector_min_distance(scan_data, 90.0, 32.0)
        right = self._sector_min_distance(scan_data, -90.0, 32.0)
        back = self._sector_min_distance(scan_data, 180.0, 40.0)

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

    def get_faces(self):
        return self._request("GET", "/faces")

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
