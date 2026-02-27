#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import sys

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from common_http_api import RobotState, build_handler, start_http_server


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def main():
    rospy.init_node("robot_http_server_indigo")

    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8080))

    cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobile_base/commands/velocity")
    lidar_topic = rospy.get_param("~lidar_topic", "/scan")
    camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw/compressed")
    camera_encoding = rospy.get_param("~camera_encoding", "compressed")
    faces_topic = rospy.get_param("~faces_topic", "/detected_faces")
    detect_markers = _as_bool(rospy.get_param("~detect_markers", True))
    marker_min_interval = float(rospy.get_param("~marker_min_interval", 0.2))
    aruco_dict_name = rospy.get_param("~aruco_dict_name", "DICT_4X4_50")

    battery_topic = rospy.get_param("~battery_topic", "/battery_state")
    battery_mode = rospy.get_param("~battery_mode", "battery_state")
    max_speed = float(rospy.get_param("~max_speed", 0.2))
    max_linear_speed = float(rospy.get_param("~max_linear_speed", max_speed))
    max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.5))
    hold_cmd_vel = _as_bool(rospy.get_param("~hold_cmd_vel", True))
    cmd_vel_hold_rate = float(rospy.get_param("~cmd_vel_hold_rate", 10.0))
    led1_topic = rospy.get_param("~led1_topic", "/mobile_base/commands/led1")
    led2_topic = rospy.get_param("~led2_topic", "/mobile_base/commands/led2")
    sound_topic = rospy.get_param("~sound_topic", "/robotsound")
    kobuki_sound_topic = rospy.get_param("~kobuki_sound_topic", "/mobile_base/commands/sound")
    tts_mode = rospy.get_param("~tts_mode", "say_script")

    state = RobotState(
        cmd_vel_topic=cmd_vel_topic,
        lidar_topic=lidar_topic,
        camera_topic=camera_topic,
        camera_encoding=camera_encoding,
        battery_topic=battery_topic,
        battery_mode=battery_mode,
        faces_topic=faces_topic,
        dummy_faces=False,
        detect_markers=detect_markers,
        marker_min_interval=marker_min_interval,
        aruco_dict_name=aruco_dict_name,
        led1_topic=led1_topic,
        led2_topic=led2_topic,
        sound_topic=sound_topic,
        kobuki_sound_topic=kobuki_sound_topic,
        tts_mode=tts_mode,
        hold_cmd_vel=hold_cmd_vel,
        cmd_vel_hold_rate=cmd_vel_hold_rate,
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
    )

    handler = build_handler(state, robot_name="real_robot_indigo")
    start_http_server(host, port, handler)

    rospy.loginfo("Indigo HTTP server listening on %s:%s", host, port)
    rospy.spin()


if __name__ == "__main__":
    main()
