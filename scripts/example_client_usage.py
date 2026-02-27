#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import json
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from http_api_client import RobotHttpClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True, help="Base server URL, e.g. http://127.0.0.1:8080")
    parser.add_argument("--linear", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=0.0)
    args = parser.parse_args()

    client = RobotHttpClient(args.url)

    print("health:", json.dumps(client.health(), indent=2, sort_keys=True))
    print("cmd_vel:", json.dumps(client.send_cmd_vel(args.linear, args.angular), indent=2, sort_keys=True))
    lidar = client.get_lidar()
    print("lidar:", json.dumps(lidar, indent=2, sort_keys=True)[:400], "...")
    print("obstacle_report:", json.dumps(client.get_obstacle_report(), indent=2, sort_keys=True))
    print("camera:", json.dumps(client.get_camera(), indent=2, sort_keys=True)[:400], "...")
    print("faces:", json.dumps(client.get_faces(), indent=2, sort_keys=True))
    print("battery:", json.dumps(client.get_battery(), indent=2, sort_keys=True))
    print("markers:", json.dumps(client.get_markers(), indent=2, sort_keys=True))
    print("velocity:", json.dumps(client.get_velocity(), indent=2, sort_keys=True))
    print("led:", json.dumps(client.get_led(), indent=2, sort_keys=True))
    print("sound:", json.dumps(client.get_sound(), indent=2, sort_keys=True))
    print("tts:", json.dumps(client.get_tts(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
