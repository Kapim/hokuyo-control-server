#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import base64
import glob
import json
import os
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, String

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _points_to_polygon(points):
    if points is None:
        return []
    try:
        if np is not None:
            shaped = np.array(points).reshape((-1, 2))
            return [[int(pt[0]), int(pt[1])] for pt in shaped]
        polygon = []
        for item in points:
            if hasattr(item, "__iter__") and not isinstance(item, (str, bytes)):
                values = list(item)
                if len(values) >= 2 and not hasattr(values[0], "__iter__"):
                    polygon.append([int(values[0]), int(values[1])])
                else:
                    for sub_item in values:
                        if hasattr(sub_item, "__iter__") and not isinstance(sub_item, (str, bytes)):
                            sub_values = list(sub_item)
                            if len(sub_values) >= 2:
                                polygon.append([int(sub_values[0]), int(sub_values[1])])
        return polygon
    except Exception:
        return []


OBJECTS_IMAGE_WIDTH = 640.0
OBJECTS_IMAGE_HEIGHT = 480.0


class RobotState(object):
    def __init__(
        self,
        cmd_vel_topic,
        lidar_topic,
        camera_topic,
        camera_encoding,
        battery_topic,
        battery_mode,
        objects_topic=None,
        dummy_objects=False,
        detect_markers=False,
        marker_min_interval=0.2,
        aruco_dict_name="DICT_4X4_50",
        led1_topic=None,
        led2_topic=None,
        sound_topic=None,
        kobuki_sound_topic=None,
        tts_mode="disabled",
        hold_cmd_vel=False,
        cmd_vel_hold_rate=10.0,
        cmd_vel_timeout=1.0,
        max_linear_speed=None,
        max_angular_speed=None,
        objects_max_age=1.0,
    ):
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        self._lock = threading.Lock()
        self._latest_lidar = None
        self._latest_camera = None
        self._latest_battery = None
        self._battery_mode = str(battery_mode or "")
        self._laptop_battery_dir = None
        self._latest_objects = None
        self._latest_objects_stamp = 0.0
        self._objects_max_age = max(0.0, float(objects_max_age))
        self._latest_markers = {
            "stamp": 0.0,
            "qr_codes": [],
            "aruco_markers": [],
            "enabled": False,
        }
        self._latest_led = {}

        self._led_cls = None
        self._led_publishers = {}
        self._sound_request_cls = None
        self._sound_pub = None
        self._tts_mode = str(tts_mode or "disabled")
        self._tts_constants = {
            "SAY": -3,
            "PLAY_ONCE": 1,
            "PLAY_START": 2,
            "PLAY_STOP": 0,
        }
        self._latest_tts = None
        self._kobuki_sound_cls = None
        self._kobuki_sound_pub = None
        self._kobuki_sound_constants = {
            "ON": 0,
            "OFF": 1,
            "RECHARGE": 2,
            "BUTTON": 3,
            "ERROR": 4,
            "CLEANINGSTART": 5,
            "CLEANINGEND": 6,
        }
        self._latest_kobuki_sound = None
        self._hold_cmd_vel = bool(hold_cmd_vel)
        self._cmd_vel_hold_rate = max(1.0, float(cmd_vel_hold_rate))
        self._cmd_vel_timeout = max(0.0, float(cmd_vel_timeout))
        self._max_linear_speed = float(max_linear_speed) if max_linear_speed is not None else None
        self._max_angular_speed = float(max_angular_speed) if max_angular_speed is not None else None
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._last_cmd_stamp = 0.0
        self._cmd_vel_timed_out = False
        self._cmd_vel_timer = None

        self._dummy_objects = dummy_objects
        self._dummy_object_counter = 0
        self._marker_min_interval = max(0.0, float(marker_min_interval))
        self._last_marker_update = 0.0
        self._marker_detection_enabled = bool(detect_markers and cv2 is not None)
        self._qr_detector = None
        self._aruco_dictionary = None

        if detect_markers and cv2 is None:
            rospy.logwarn("Marker detection requested, but OpenCV (cv2) is not available.")
        if detect_markers and np is None:
            rospy.loginfo("Marker detection fallback enabled without numpy; QR/legacy contour detection will still run.")

        if self._marker_detection_enabled:
            self._qr_detector = self._build_qr_detector()
            self._aruco_dictionary = self._build_aruco_dictionary(aruco_dict_name)
            self._latest_markers["enabled"] = True

        rospy.Subscriber(lidar_topic, LaserScan, self._lidar_cb, queue_size=1)

        if camera_encoding == "compressed":
            rospy.Subscriber(camera_topic, CompressedImage, self._camera_compressed_cb, queue_size=1)
        else:
            rospy.Subscriber(camera_topic, Image, self._camera_raw_cb, queue_size=1)

        if battery_mode == "battery_state":
            rospy.Subscriber(battery_topic, BatteryState, self._battery_state_cb, queue_size=1)
        elif battery_mode == "float32":
            rospy.Subscriber(battery_topic, Float32, self._battery_float_cb, queue_size=1)
        elif battery_mode == "laptop_sysfs":
            rospy.loginfo("Battery mode laptop_sysfs enabled for /battery endpoint")
        else:
            rospy.logwarn("Unknown battery mode '%s', battery endpoint will be empty", battery_mode)

        if objects_topic and not dummy_objects:
            rospy.Subscriber(objects_topic, String, self._objects_cb, queue_size=1)

        if led1_topic or led2_topic:
            try:
                from kobuki_msgs.msg import Led

                self._led_cls = Led
                if led1_topic:
                    self._led_publishers[1] = rospy.Publisher(led1_topic, Led, queue_size=10)
                if led2_topic:
                    self._led_publishers[2] = rospy.Publisher(led2_topic, Led, queue_size=10)
            except Exception as exc:
                rospy.logwarn("Cannot initialize Kobuki LED publishers (%s).", exc)

        if sound_topic and self._tts_mode == "sound_request":
            try:
                from sound_play.msg import SoundRequest

                self._sound_request_cls = SoundRequest
                self._sound_pub = rospy.Publisher(sound_topic, SoundRequest, queue_size=10)
                for key in self._tts_constants:
                    if hasattr(SoundRequest, key):
                        self._tts_constants[key] = int(getattr(SoundRequest, key))
            except Exception as exc:
                rospy.logwarn("Cannot initialize SoundRequest publisher (%s).", exc)

        if kobuki_sound_topic:
            try:
                from kobuki_msgs.msg import Sound

                self._kobuki_sound_cls = Sound
                self._kobuki_sound_pub = rospy.Publisher(kobuki_sound_topic, Sound, queue_size=10)
                for key in self._kobuki_sound_constants:
                    if hasattr(Sound, key):
                        self._kobuki_sound_constants[key] = int(getattr(Sound, key))
            except Exception as exc:
                rospy.logwarn("Cannot initialize kobuki_msgs/Sound publisher (%s).", exc)

        if self._hold_cmd_vel or self._cmd_vel_timeout > 0.0:
            period = 1.0 / self._cmd_vel_hold_rate
            self._cmd_vel_timer = rospy.Timer(rospy.Duration(period), self._cmd_vel_timer_cb)

    def _lidar_cb(self, msg):
        with self._lock:
            self._latest_lidar = {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id,
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities),
            }

    def _camera_compressed_cb(self, msg):
        payload_bytes = msg.data
        payload_format = msg.format
        marker_payload = None
        if self._marker_detection_enabled or cv2 is not None:
            bgr = self._decode_compressed_to_bgr(msg.data)
            if bgr is not None and self._marker_detection_enabled:
                marker_payload = self._detect_markers_in_image(bgr, msg.header.stamp.to_sec())
            if bgr is not None:
                objects = self._current_objects()
                detections = objects.get("detections", []) if objects else []
                if detections:
                    try:
                        bgr_overlay = self._draw_objects_rectangles(bgr, detections)
                        ok, encoded_image = cv2.imencode(".jpg", bgr_overlay)
                        if ok:
                            payload_bytes = encoded_image.tobytes()
                            payload_format = "jpeg"
                    except Exception:
                        pass

        encoded = base64.b64encode(payload_bytes)
        if not isinstance(encoded, str):
            encoded = encoded.decode("ascii")

        with self._lock:
            self._latest_camera = {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id,
                "format": payload_format,
                "encoding": "compressed",
                "data_base64": encoded,
            }
            if marker_payload is not None:
                self._latest_markers = marker_payload

    def _camera_raw_cb(self, msg):
        raw_bytes = msg.data
        marker_payload = None
        if self._marker_detection_enabled or cv2 is not None:
            bgr = self._decode_raw_to_bgr(msg)
            if bgr is not None and self._marker_detection_enabled:
                marker_payload = self._detect_markers_in_image(bgr, msg.header.stamp.to_sec())
            if bgr is not None:
                objects = self._current_objects()
                detections = objects.get("detections", []) if objects else []
                if detections and msg.encoding in ("bgr8", "rgb8"):
                    try:
                        overlay = self._draw_objects_rectangles(bgr, detections)
                        if msg.encoding == "rgb8":
                            overlay = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
                        raw_bytes = overlay.tobytes()
                    except Exception:
                        pass

        encoded = base64.b64encode(raw_bytes)
        if not isinstance(encoded, str):
            encoded = encoded.decode("ascii")

        with self._lock:
            self._latest_camera = {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id,
                "height": msg.height,
                "width": msg.width,
                "step": msg.step,
                "format": msg.encoding,
                "is_bigendian": msg.is_bigendian,
                "encoding": "raw",
                "data_base64": encoded,
            }
            if marker_payload is not None:
                self._latest_markers = marker_payload

    def _battery_state_cb(self, msg):
        with self._lock:
            self._latest_battery = {
                "stamp": time.time(),
                "voltage": msg.voltage,
                "current": msg.current,
                "charge": msg.charge,
                "capacity": msg.capacity,
                "percentage": msg.percentage,
                "power_supply_status": msg.power_supply_status,
            }

    def _battery_float_cb(self, msg):
        with self._lock:
            self._latest_battery = {
                "stamp": time.time(),
                "percentage": _clamp(float(msg.data), 0.0, 1.0),
            }

    def _read_text_file(self, path):
        try:
            with open(path, "r") as f:
                return f.read().strip()
        except Exception:
            return None

    def _read_float_file(self, path):
        raw = self._read_text_file(path)
        if raw is None:
            return None
        try:
            return float(raw)
        except Exception:
            return None

    def _find_laptop_battery_dir(self):
        if self._laptop_battery_dir and os.path.isdir(self._laptop_battery_dir):
            return self._laptop_battery_dir
        for candidate in sorted(glob.glob("/sys/class/power_supply/BAT*")):
            if not os.path.isdir(candidate):
                continue
            supply_type = self._read_text_file(os.path.join(candidate, "type"))
            if supply_type is None or supply_type.lower() == "battery":
                self._laptop_battery_dir = candidate
                return candidate
        return None

    def _read_laptop_battery(self):
        battery_dir = self._find_laptop_battery_dir()
        if battery_dir is None:
            return None

        capacity_pct = self._read_float_file(os.path.join(battery_dir, "capacity"))
        percentage = None
        if capacity_pct is not None:
            percentage = _clamp(capacity_pct / 100.0, 0.0, 1.0)

        voltage_uv = self._read_float_file(os.path.join(battery_dir, "voltage_now"))
        current_ua = self._read_float_file(os.path.join(battery_dir, "current_now"))
        charge_mah = self._read_float_file(os.path.join(battery_dir, "charge_now"))
        capacity_mah = self._read_float_file(os.path.join(battery_dir, "charge_full"))
        if charge_mah is None:
            charge_mah = self._read_float_file(os.path.join(battery_dir, "energy_now"))
        if capacity_mah is None:
            capacity_mah = self._read_float_file(os.path.join(battery_dir, "energy_full"))

        status = self._read_text_file(os.path.join(battery_dir, "status"))

        return {
            "stamp": time.time(),
            "source": "laptop_sysfs",
            "battery_name": os.path.basename(battery_dir),
            "voltage": (voltage_uv / 1000000.0) if voltage_uv is not None else None,
            "current": (current_ua / 1000000.0) if current_ua is not None else None,
            "charge": (charge_mah / 1000000.0) if charge_mah is not None else None,
            "capacity": (capacity_mah / 1000000.0) if capacity_mah is not None else None,
            "percentage": percentage,
            "power_supply_status": status,
        }

    def _objects_cb(self, msg):
        objects = self._parse_objects_payload(msg.data)
        if objects is None:
            return
        stamp = rospy.get_time()
        if stamp <= 0.0:
            stamp = time.time()
        with self._lock:
            self._latest_objects = objects
            self._latest_objects_stamp = float(stamp)

    def _empty_objects_payload(self):
        return {"timestamp": None, "recv_timestamp": None, "detections": []}

    def _dummy_objects_payload(self):
        # Oscillating one synthetic object for testing API consumers.
        self._dummy_object_counter += 1
        size = 64 + (self._dummy_object_counter % 4) * 8
        center_x = 120 + (self._dummy_object_counter % 5) * 10
        center_y = 90 + (self._dummy_object_counter % 3) * 12
        return {
            "timestamp": int(time.time() * 1e9),
            "recv_timestamp": int(time.time() * 1e9),
            "detections": [
                {
                    "center_x": _clamp(float(center_x) / OBJECTS_IMAGE_WIDTH, 0.0, 1.0),
                    "center_y": _clamp(float(center_y) / OBJECTS_IMAGE_HEIGHT, 0.0, 1.0),
                    "width": _clamp(float(size) / OBJECTS_IMAGE_WIDTH, 0.0, 1.0),
                    "height": _clamp(float(size) / OBJECTS_IMAGE_HEIGHT, 0.0, 1.0),
                    "score": 1.0,
                    "class": 0,
                    "class_name": "person",
                }
            ],
        }

    def _parse_objects_payload(self, raw_data):
        try:
            parsed = json.loads(raw_data)
        except Exception:
            rospy.logwarn_throttle(10.0, "Received invalid JSON on objects topic.")
            return None
        if not isinstance(parsed, dict):
            rospy.logwarn_throttle(10.0, "Objects topic JSON must be an object.")
            return None

        detections = parsed.get("detections", [])
        if not isinstance(detections, list):
            detections = []
        normalized_detections = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            normalized = dict(detection)
            bbox = normalized.get("bbox", [])
            if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
                x1 = float(bbox[0])
                y1 = float(bbox[1])
                x2 = float(bbox[2])
                y2 = float(bbox[3])
                normalized["center_x"] = _clamp(((x1 + x2) * 0.5) / OBJECTS_IMAGE_WIDTH, 0.0, 1.0)
                normalized["center_y"] = _clamp(((y1 + y2) * 0.5) / OBJECTS_IMAGE_HEIGHT, 0.0, 1.0)
                normalized["width"] = _clamp(max(0.0, x2 - x1) / OBJECTS_IMAGE_WIDTH, 0.0, 1.0)
                normalized["height"] = _clamp(max(0.0, y2 - y1) / OBJECTS_IMAGE_HEIGHT, 0.0, 1.0)
            else:
                center_x = normalized.get("center_x")
                center_y = normalized.get("center_y")
                width = normalized.get("width")
                height = normalized.get("height")
                if center_x is not None:
                    normalized["center_x"] = _clamp(float(center_x), 0.0, 1.0)
                else:
                    normalized["center_x"] = None
                if center_y is not None:
                    normalized["center_y"] = _clamp(float(center_y), 0.0, 1.0)
                else:
                    normalized["center_y"] = None
                if width is not None:
                    normalized["width"] = _clamp(max(0.0, float(width)), 0.0, 1.0)
                else:
                    normalized["width"] = None
                if height is not None:
                    normalized["height"] = _clamp(max(0.0, float(height)), 0.0, 1.0)
                else:
                    normalized["height"] = None
            normalized.pop("bbox", None)
            normalized_detections.append(normalized)

        parsed["detections"] = normalized_detections
        return parsed

    def _copy_objects_payload(self, objects):
        if objects is None:
            return None
        copied = dict(objects)
        detections = copied.get("detections", [])
        copied_detections = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            copied_detections.append(dict(detection))
        copied["detections"] = copied_detections
        return copied

    def _current_objects(self):
        with self._lock:
            objects = self._copy_objects_payload(self._latest_objects)
            latest_objects_stamp = float(self._latest_objects_stamp)

        if self._dummy_objects:
            return self._dummy_objects_payload()
        if objects is None:
            return self._empty_objects_payload()
        if self._objects_max_age <= 0.0:
            return objects

        now = rospy.get_time()
        if now <= 0.0:
            now = time.time()
        if latest_objects_stamp <= 0.0 or (float(now) - latest_objects_stamp) > self._objects_max_age:
            return self._empty_objects_payload()
        return objects

    def _draw_objects_rectangles(self, bgr_image, detections):
        if cv2 is None or bgr_image is None:
            return bgr_image
        overlay = bgr_image.copy()
        image_height, image_width = overlay.shape[:2]
        for detection in detections:
            center_x = detection.get("center_x")
            center_y = detection.get("center_y")
            width = detection.get("width")
            height = detection.get("height")
            if None in (center_x, center_y, width, height):
                continue
            half_width = float(width) * float(image_width) * 0.5
            half_height = float(height) * float(image_height) * 0.5
            x_center_px = float(center_x) * float(image_width)
            y_center_px = float(center_y) * float(image_height)
            x1 = int(round(_clamp(x_center_px - half_width, 0.0, float(image_width - 1))))
            y1 = int(round(_clamp(y_center_px - half_height, 0.0, float(image_height - 1))))
            x2 = int(round(_clamp(x_center_px + half_width, 0.0, float(image_width - 1))))
            y2 = int(round(_clamp(y_center_px + half_height, 0.0, float(image_height - 1))))
            if x2 <= x1 or y2 <= y1:
                continue
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = detection.get("class_name")
            score = detection.get("score")
            if label:
                if score is not None:
                    label = "%s %.2f" % (label, float(score))
                cv2.putText(
                    overlay,
                    str(label),
                    (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
        return overlay

    def _build_qr_detector(self):
        if cv2 is None:
            return None
        if not hasattr(cv2, "QRCodeDetector"):
            rospy.logwarn("OpenCV QRCodeDetector is not available.")
            return None
        try:
            return cv2.QRCodeDetector()
        except Exception as exc:
            rospy.logwarn("QRCodeDetector init failed: %s", exc)
            return None

    def _build_aruco_dictionary(self, dict_name):
        if cv2 is None or not hasattr(cv2, "aruco"):
            rospy.logwarn("OpenCV ArUco module is not available.")
            return None
        aruco = cv2.aruco
        aruco_id = getattr(aruco, dict_name, None)
        if aruco_id is None:
            rospy.logwarn("Unknown ArUco dictionary '%s', using DICT_4X4_50", dict_name)
            aruco_id = getattr(aruco, "DICT_4X4_50")
        try:
            if hasattr(aruco, "getPredefinedDictionary"):
                return aruco.getPredefinedDictionary(aruco_id)
            return aruco.Dictionary_get(aruco_id)
        except Exception as exc:
            rospy.logwarn("ArUco dictionary init failed: %s", exc)
            return None

    def _decode_compressed_to_bgr(self, data):
        if np is None or cv2 is None:
            return None
        try:
            np_arr = np.frombuffer(data, dtype=np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _decode_raw_to_bgr(self, msg):
        if np is None or cv2 is None:
            return None
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "bgr8":
                return arr.reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                rgb = arr.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if msg.encoding == "mono8":
                gray = arr.reshape((msg.height, msg.width))
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except Exception:
            return None
        rospy.logwarn_throttle(10.0, "Unsupported raw camera encoding for marker detection: %s", msg.encoding)
        return None

    def _detect_qr_codes(self, bgr_image):
        if self._qr_detector is None:
            return []
        qr_codes = []
        try:
            if hasattr(self._qr_detector, "detectAndDecodeMulti"):
                ok, decoded_info, points, _ = self._qr_detector.detectAndDecodeMulti(bgr_image)
                if ok and decoded_info is not None:
                    for i, text in enumerate(decoded_info):
                        if not text:
                            continue
                        polygon = _points_to_polygon(points[i]) if points is not None and len(points) > i else []
                        qr_codes.append({"data": text, "polygon": polygon})
                return qr_codes
        except Exception:
            pass

        try:
            text, points, _ = self._qr_detector.detectAndDecode(bgr_image)
            if text:
                polygon = _points_to_polygon(points)
                qr_codes.append({"data": text, "polygon": polygon})
        except Exception:
            pass
        return qr_codes

    def _detect_aruco_markers(self, bgr_image):
        if cv2 is None:
            return []
        if self._aruco_dictionary is not None and hasattr(cv2, "aruco"):
            aruco = cv2.aruco
            try:
                gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
                if hasattr(aruco, "DetectorParameters_create"):
                    params = aruco.DetectorParameters_create()
                    corners, ids, _ = aruco.detectMarkers(gray, self._aruco_dictionary, parameters=params)
                else:
                    params = aruco.DetectorParameters()
                    corners, ids, _ = aruco.detectMarkers(gray, self._aruco_dictionary, parameters=params)
            except Exception:
                corners = None
                ids = None
            else:
                markers = []
                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten().tolist()):
                        marker_corners = _points_to_polygon(corners[i][0]) if corners is not None and len(corners) > i and len(corners[i]) > 0 else []
                        markers.append({"id": int(marker_id), "corners": marker_corners})
                if markers:
                    return markers

        return self._detect_aruco_markers_legacy(bgr_image)

    @staticmethod
    def _find_contours_compat(binary_image):
        result = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(result) == 2:
            contours, hierarchy = result
        else:
            _, contours, hierarchy = result
        return contours, hierarchy

    @staticmethod
    def _order_quad_points(points):
        if not points or len(points) != 4:
            return []
        pts = [(float(pt[0]), float(pt[1])) for pt in points]
        pts_by_y = sorted(pts, key=lambda item: (item[1], item[0]))
        top = sorted(pts_by_y[:2], key=lambda item: item[0])
        bottom = sorted(pts_by_y[2:], key=lambda item: item[0])
        if len(top) != 2 or len(bottom) != 2:
            return []
        return [top[0], top[1], bottom[1], bottom[0]]

    def _detect_aruco_markers_legacy(self, bgr_image):
        if cv2 is None:
            return []
        try:
            gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        except Exception:
            return []

        try:
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        except Exception:
            blurred = gray

        image_area = float(gray.shape[0] * gray.shape[1]) if hasattr(gray, "shape") and len(gray.shape) >= 2 else 0.0
        if image_area <= 0.0:
            return []

        accepted = []
        seen_centers = []
        threshold_modes = [
            cv2.THRESH_BINARY + cv2.THRESH_OTSU,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        ]

        for threshold_mode in threshold_modes:
            try:
                _, binary = cv2.threshold(blurred, 0, 255, threshold_mode)
                contours, _ = self._find_contours_compat(binary)
            except Exception:
                continue

            for contour in contours:
                try:
                    area = abs(cv2.contourArea(contour))
                    if area < 200.0 or area > image_area * 0.5:
                        continue
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter <= 0.0:
                        continue
                    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                    if len(approx) != 4 or not cv2.isContourConvex(approx):
                        continue

                    rect = cv2.boundingRect(approx)
                    if rect[2] <= 0 or rect[3] <= 0:
                        continue
                    aspect = float(rect[2]) / float(rect[3])
                    if aspect < 0.65 or aspect > 1.35:
                        continue

                    points = approx.reshape((-1, 2)).tolist() if hasattr(approx, "reshape") else []
                    ordered = self._order_quad_points(points)
                    if len(ordered) != 4:
                        continue

                    center_x = sum(pt[0] for pt in ordered) / 4.0
                    center_y = sum(pt[1] for pt in ordered) / 4.0
                    duplicate = False
                    for seen_x, seen_y in seen_centers:
                        dx = center_x - seen_x
                        dy = center_y - seen_y
                        if (dx * dx + dy * dy) < 100.0:
                            duplicate = True
                            break
                    if duplicate:
                        continue

                    seen_centers.append((center_x, center_y))
                    accepted.append(
                        {
                            "id": -1,
                            "corners": [[int(round(pt[0])), int(round(pt[1]))] for pt in ordered],
                            "method": "legacy_contour",
                        }
                    )
                except Exception:
                    continue

        accepted.sort(key=lambda item: item["corners"][0][1] if item.get("corners") else 0)
        return accepted[:10]

    def _detect_markers_in_image(self, bgr_image, stamp):
        now = time.time()
        if now - self._last_marker_update < self._marker_min_interval:
            return None
        self._last_marker_update = now

        return {
            "stamp": float(stamp),
            "enabled": True,
            "qr_codes": self._detect_qr_codes(bgr_image),
            "aruco_markers": self._detect_aruco_markers(bgr_image),
        }

    def publish_cmd_vel(self, linear_x, angular_z):
        linear_x = self._clamp_with_limit(float(linear_x), self._max_linear_speed)
        angular_z = self._clamp_with_limit(float(angular_z), self._max_angular_speed)
        stamp = rospy.get_time()
        if stamp <= 0.0:
            stamp = time.time()
        with self._lock:
            self._last_cmd_linear = linear_x
            self._last_cmd_angular = angular_z
            self._last_cmd_stamp = float(stamp)
            self._cmd_vel_timed_out = False
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        return linear_x, angular_z

    @staticmethod
    def _clamp_with_limit(value, limit):
        if limit is None:
            return value
        abs_limit = abs(float(limit))
        return _clamp(value, -abs_limit, abs_limit)

    def _cmd_vel_timer_cb(self, _event):
        with self._lock:
            linear_x = self._last_cmd_linear
            angular_z = self._last_cmd_angular
            stamp = self._last_cmd_stamp
            timed_out = self._cmd_vel_timed_out

        if self._cmd_vel_timeout > 0.0:
            now = rospy.get_time()
            if now <= 0.0:
                now = time.time()
            if stamp <= 0.0 or (float(now) - float(stamp)) > self._cmd_vel_timeout:
                linear_x = 0.0
                angular_z = 0.0
                with self._lock:
                    self._last_cmd_linear = 0.0
                    self._last_cmd_angular = 0.0
                    self._cmd_vel_timed_out = True
                if not timed_out:
                    rospy.logwarn(
                        "cmd_vel timeout reached (%.2fs). Publishing stop command.",
                        self._cmd_vel_timeout,
                    )
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def set_led(self, led_index, value):
        led_index = int(led_index)
        value = int(value)
        if value < 0 or value > 3:
            return False, "invalid_led_value"
        if led_index not in self._led_publishers or self._led_cls is None:
            return False, "led_not_available"

        msg = self._led_cls()
        msg.value = value
        self._led_publishers[led_index].publish(msg)
        with self._lock:
            self._latest_led[str(led_index)] = value
        return True, {"led_index": led_index, "value": value}

    def play_kobuki_sound(self, value):
        if self._kobuki_sound_pub is None or self._kobuki_sound_cls is None:
            return False, "kobuki_sound_not_available"

        value = int(value)
        if value < 0 or value > 6:
            return False, "invalid_kobuki_sound_value"

        msg = self._kobuki_sound_cls()
        msg.value = value
        self._kobuki_sound_pub.publish(msg)
        with self._lock:
            self._latest_kobuki_sound = {
                "value": int(value),
                "stamp": time.time(),
            }
        return True, dict(self._latest_kobuki_sound)

    def say_text(self, text, command=None):
        if command is None:
            command = self._tts_constants["PLAY_ONCE"]

        if self._tts_mode == "say_script":
            try:
                subprocess.Popen(["rosrun", "sound_play", "say.py", str(text)])
            except Exception as exc:
                return False, "tts_exec_failed: %s" % exc
            with self._lock:
                self._latest_tts = {
                    "text": str(text),
                    "command": int(command),
                    "mode": "say_script",
                    "stamp": time.time(),
                }
            return True, dict(self._latest_tts)

        if self._tts_mode == "sound_request":
            if self._sound_pub is None or self._sound_request_cls is None:
                return False, "tts_not_available"
            msg = self._sound_request_cls()
            msg.sound = int(self._tts_constants["SAY"])
            msg.command = int(command)
            msg.arg = str(text)
            msg.arg2 = ""
            self._sound_pub.publish(msg)
            with self._lock:
                self._latest_tts = {
                    "text": msg.arg,
                    "command": int(msg.command),
                    "mode": "sound_request",
                    "stamp": time.time(),
                }
            return True, dict(self._latest_tts)

        return False, "tts_not_available"

    def snapshot(self):
        with self._lock:
            lidar = self._latest_lidar
            camera = self._latest_camera
            battery = self._latest_battery
            markers = dict(self._latest_markers)
            led_state = dict(self._latest_led)
            last_kobuki_sound = dict(self._latest_kobuki_sound) if self._latest_kobuki_sound else None
            last_tts = dict(self._latest_tts) if self._latest_tts else None
            cmd_linear = self._last_cmd_linear
            cmd_angular = self._last_cmd_angular

        if self._battery_mode == "laptop_sysfs":
            battery = self._read_laptop_battery()

        velocity = {
            "source": "cmd_vel",
            "stamp": time.time(),
            "linear": {"x": cmd_linear, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": cmd_angular},
        }

        objects = self._current_objects()
        detections = objects.get("detections", []) if objects else []

        return {
            "lidar": lidar,
            "camera": camera,
            "battery": battery,
            "objects": detections,
            "markers": markers,
            "led": {
                "state": led_state,
                "available": len(self._led_publishers) > 0,
            },
            "sound": {
                "last_request": last_kobuki_sound,
                "available": self._kobuki_sound_pub is not None,
                "constants": dict(self._kobuki_sound_constants),
            },
            "tts": {
                "last_request": last_tts,
                "available": self._tts_mode in ("say_script", "sound_request"),
                "mode": self._tts_mode,
            },
            "velocity": velocity,
        }


def build_handler(state, robot_name):
    try:
        from BaseHTTPServer import BaseHTTPRequestHandler  # Python 2
    except ImportError:
        from http.server import BaseHTTPRequestHandler  # Python 3

    class ApiHandler(BaseHTTPRequestHandler):
        @staticmethod
        def _parse_led_value(raw_value):
            if isinstance(raw_value, int):
                return raw_value
            text = str(raw_value).strip().upper()
            mapping = {
                "BLACK": 0,
                "GREEN": 1,
                "ORANGE": 2,
                "RED": 3,
            }
            if text in mapping:
                return mapping[text]
            return int(raw_value)

        @staticmethod
        def _parse_kobuki_sound_value(raw_value):
            if isinstance(raw_value, int):
                return raw_value
            text = str(raw_value).strip().upper()
            mapping = {
                "ON": 0,
                "OFF": 1,
                "RECHARGE": 2,
                "BUTTON": 3,
                "ERROR": 4,
                "CLEANINGSTART": 5,
                "CLEANINGEND": 6,
            }
            if text in mapping:
                return mapping[text]
            return int(raw_value)

        def _read_json(self):
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                return json.loads(body.decode("utf-8")), None
            except Exception:
                return None, {"error": "invalid_json"}

        def _send_json(self, status, payload):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            rospy.loginfo("HTTP %s - %s", self.address_string(), fmt % args)

        def do_GET(self):
            snap = state.snapshot()
            if self.path == "/health":
                return self._send_json(
                    200,
                    {
                        "ok": True,
                        "robot": robot_name,
                        "time": time.time(),
                    },
                )
            if self.path == "/sensors/lidar":
                return self._send_json(200, {"data": snap["lidar"]})
            if self.path == "/sensors/camera":
                return self._send_json(200, {"data": snap["camera"]})
            if self.path == "/objects":
                return self._send_json(200, {"data": snap["objects"]})
            if self.path == "/battery":
                return self._send_json(200, {"data": snap["battery"]})
            if self.path == "/markers":
                return self._send_json(200, {"data": snap["markers"]})
            if self.path == "/velocity":
                return self._send_json(200, {"data": snap["velocity"]})
            if self.path == "/led":
                return self._send_json(200, {"data": snap["led"]})
            if self.path == "/sound":
                return self._send_json(200, {"data": snap["sound"]})
            if self.path == "/tts":
                return self._send_json(200, {"data": snap["tts"]})
            return self._send_json(404, {"error": "not_found"})

        def do_POST(self):
            payload, err = self._read_json()
            if err:
                return self._send_json(400, err)

            if self.path == "/cmd_vel":
                try:
                    linear_x = float(payload.get("linear_x", 0.0))
                    angular_z = float(payload.get("angular_z", 0.0))
                except Exception:
                    return self._send_json(400, {"error": "invalid_cmd_vel_payload"})
                applied_linear_x, applied_angular_z = state.publish_cmd_vel(linear_x, angular_z)
                return self._send_json(
                    200,
                    {
                        "ok": True,
                        "linear_x": applied_linear_x,
                        "angular_z": applied_angular_z,
                    },
                )

            if self.path == "/led":
                try:
                    led_index = int(payload.get("led_index", payload.get("led")))
                    value = self._parse_led_value(payload.get("value"))
                except Exception:
                    return self._send_json(400, {"error": "invalid_led_payload"})
                ok, data = state.set_led(led_index, value)
                if not ok:
                    status = 503 if data == "led_not_available" else 400
                    return self._send_json(status, {"error": data})
                return self._send_json(200, {"ok": True, "data": data})

            if self.path == "/sound":
                try:
                    value = self._parse_kobuki_sound_value(payload.get("kobuki_value", payload.get("value")))
                except Exception:
                    return self._send_json(400, {"error": "invalid_kobuki_sound_payload"})
                ok, data = state.play_kobuki_sound(value=value)
                if not ok:
                    status = 503 if data == "kobuki_sound_not_available" else 400
                    return self._send_json(status, {"error": data})
                return self._send_json(200, {"ok": True, "data": data})

            if self.path == "/tts":
                text = payload.get("text")
                if text is None:
                    return self._send_json(400, {"error": "invalid_tts_payload"})
                try:
                    command = int(payload.get("command", 1))
                except Exception:
                    return self._send_json(400, {"error": "invalid_tts_payload"})
                ok, data = state.say_text(text=text, command=command)
                if not ok:
                    return self._send_json(503, {"error": data})
                return self._send_json(200, {"ok": True, "data": data})

            return self._send_json(404, {"error": "not_found"})

    return ApiHandler


def start_http_server(host, port, handler_class):
    try:
        from BaseHTTPServer import HTTPServer  # Python 2
    except ImportError:
        from http.server import HTTPServer  # Python 3

    server = HTTPServer((host, int(port)), handler_class)
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    return server
