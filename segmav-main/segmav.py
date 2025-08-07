#!/usr/bin/env python3
"""
SEGMAV: Jetson-based semantic segmentation navigation and recording system.
This script includes SegThread (for navigation) and VideoThread (for recording).
"""

import sys
import os
import argparse
import threading
import signal
import time
import numpy as np
import cv2
from datetime import datetime

from jetson_inference import segNet
from jetson_utils import (
    videoSource, videoOutput,
    cudaDeviceSynchronize, cudaAllocMapped,
    cudaToNumpy, cudaDrawCircle, cudaDrawLine, cudaDrawRect, cudaFont
)

from pymavlink import mavutil

# ========== Shared Flags ==========
exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()


# ========== MAVLink Utilities ==========
def send_msg_to_gcs(conn, text):
    msg = "SEGMAV: " + text
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

def play_tune(conn, tune):
    conn.mav.play_tune_send(conn.target_system, conn.target_component, bytes(tune, "ascii"))

def set_target(conn, speed, yaw):
    POSITION_TARGET_TYPEMASK = 0b101111000111
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, POSITION_TARGET_TYPEMASK,
        0, 0, 0, speed, 0, 0, 0, 0, 0, yaw, 0
    )


# ========== SegThread ==========
class SegThread(threading.Thread):
    def __init__(self, args, aargv, is_headless):
        super().__init__()
        self.should_exit = False
        self.args = args
        self.input = videoSource(args.input, argv=aargv)
        self.output = videoOutput(args.output, argv=aargv) if not is_headless else None
        self.net = segNet(args.network)
        self.net.SetOverlayAlpha(args.alpha)
        self.overlay = None
        self.class_mask = None
        self.class_mask_np = None
        self.grid_width = None
        self.grid_height = None
        self.font = cudaFont()
        self.bearings = []
        self.threadLock = threading.Lock()
        self.timeOfLastUpdate = 0
        self.timeOfCapture = []
        self.numAveraging = 3

    def exit(self):
        self.should_exit = True

    def run(self):
        while not self.should_exit:
            img_input = self.input.Capture()
            if img_input is None:
                continue

            with self.threadLock:
                self.timeOfCapture.append(time.time())
                if len(self.timeOfCapture) > self.numAveraging:
                    self.timeOfCapture = self.timeOfCapture[-self.numAveraging:]

            if self.overlay is None:
                self.overlay = cudaAllocMapped(width=img_input.shape[1],
                                               height=img_input.shape[0],
                                               format=img_input.format)

            if self.class_mask is None:
                self.grid_width, self.grid_height = self.net.GetGridSize()
                self.class_mask = cudaAllocMapped(width=self.grid_width,
                                                  height=self.grid_height,
                                                  format="gray8")
                self.class_mask_np = cudaToNumpy(self.class_mask)

            self.net.Process(img_input, ignore_class=self.args.ignore_class)
            self.net.Overlay(self.overlay, filter_mode=self.args.filter_mode)
            cudaDeviceSynchronize()
            self.net.Mask(self.class_mask, self.grid_width, self.grid_height)

            # Segmentation post-processing
            mask = cv2.inRange(self.class_mask_np, self.args.targetclass, self.args.targetclass)
            zoom = 400
            width = int(mask.shape[1] * zoom / 100)
            height = int(mask.shape[0] * zoom / 100)
            maskzoom = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

            contours, _ = cv2.findContours(maskzoom, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                if self.output:
                    self.output.Render(self.overlay)
                continue

            c = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)

            centroids = []
            x1, y1, w, h = cv2.boundingRect(c)
            for i in range(2):
                roi = maskzoom[y1 + int(i * h / 2): y1 + int((i + 1) * h / 2), :]
                M = cv2.moments(roi)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"]) + y1 + int(i * h / 2)
                    centroids.append((cX, cY))
                    cudaDrawCircle(self.overlay,
                                   (int(cX * img_input.shape[1] / width),
                                    int(cY * img_input.shape[0] / height)),
                                   8, (255, 255, 255))

            if len(centroids) == 2:
                vec = np.array(centroids[1]) - np.array(centroids[0])
                angle = np.arctan2(vec[0], vec[1])
                with self.threadLock:
                    self.bearings.append(angle)
                    if len(self.bearings) > self.numAveraging:
                        self.bearings = self.bearings[-self.numAveraging:]
                        self.timeOfLastUpdate = time.time()

                cudaDrawLine(self.overlay,
                             [int(centroids[0][0] * img_input.shape[1] / width),
                              int(centroids[0][1] * img_input.shape[0] / height)],
                             [int(centroids[1][0] * img_input.shape[1] / width),
                              int(centroids[1][1] * img_input.shape[0] / height)],
                             (255, 255, 255), 4)

            if self.output:
                self.output.Render(self.overlay)

    def getBearing(self):
        with self.threadLock:
            if time.time() - self.timeOfLastUpdate > 2:
                return None
            return -0.7 * np.mean(self.bearings)


# ========== VideoThread ==========
class VideoThread(threading.Thread):
    def __init__(self, args, aargv, filename="video.mp4"):
        super().__init__()
        self.should_exit = False
        self.args = args
        self.input = videoSource(args.input, argv=aargv)
        self.output = videoOutput(filename, argv=aargv)

    def exit(self):
        self.should_exit = True

    def run(self):
        while not self.should_exit:
            frame = self.input.Capture()
            if frame is None:
                continue
            self.output.Render(frame)


# ========== Main Launch ==========
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, default="csi://0")
    parser.add_argument("--output", type=str, default="rtp://192.168.1.124:5400")
    parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes-1024x512")
    parser.add_argument("--targetclass", type=int, default=3)
    parser.add_argument("--alpha", type=float, default=80.0)
    parser.add_argument("--filter-mode", type=str, default="point")
    parser.add_argument("--ignore-class", type=str, default="void")
    parser.add_argument("--device", type=str, default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=1)
    parser.add_argument("--rc", type=int, default=10)
    parser.add_argument("--pwmlow", type=int, default=1000)
    parser.add_argument("--pwmmid", type=int, default=1500)
    parser.add_argument("--pwmhigh", type=int, default=2000)
    parser.add_argument("--vel", type=float, default=0.6)

    args = parser.parse_args()
    is_headless = ["--headless"]

    signal.signal(signal.SIGINT, signal_handler)

    conn = mavutil.mavlink_connection(args.device,
                                      autoreconnect=True,
                                      source_system=args.source_system,
                                      baud=args.baud,
                                      force_connected=False,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for heartbeat...")
    while not exit_event.is_set():
        msg = conn.wait_heartbeat(timeout=0.5)
        if msg and msg.type not in [mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavutil.mavlink.MAV_TYPE_GENERIC]:
            break

    print(f"Connected to system {conn.target_system}")
    send_msg_to_gcs(conn, "SegMAV script started")
    play_tune(conn, "L12DD")

    curHeading = -1
    rc_level = None
    fieldname = f"chan{args.rc}_raw"
    segThread = None
    last_bearing_check = 0

    while not exit_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.1)
        if not msg:
            continue

        if msg.get_type() == 'VFR_HUD':
            curHeading = msg.heading

        if msg.get_type() == 'RC_CHANNELS':
            value = getattr(msg, fieldname)
            if rc_level is None or abs(rc_level - value) > 100:
                if abs(value - args.pwmlow) < 50:
                    if segThread:
                        segThread.exit()
                        segThread = None
                        play_tune(conn, "L12DD")
                elif abs(value - args.pwmhigh) < 50 and not segThread:
                    segThread = SegThread(args, sys.argv, is_headless)
                    segThread.start()
                    conn.set_mode("GUIDED")
                    play_tune(conn, "L12DD")
                rc_level = value

        if segThread and time.time() - last_bearing_check > 0.3:
            bearing = segThread.getBearing()
            if bearing is not None:
                set_target(conn, args.vel, np.deg2rad(bearing))
                send_msg_to_gcs(conn, f"Bearing {np.rad2deg(bearing):.1f} deg")
            last_bearing_check = time.time()

    if segThread:
        segThread.exit()

    send_msg_to_gcs(conn, "SegMAV exiting")
    play_tune(conn, "L12DD")
