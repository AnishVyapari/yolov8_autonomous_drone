#!/usr/bin/env python3
"""
Camera utility: open camera by index or URL, with retries and configuration.
"""
import cv2
import time


def open_camera(source=0, width=640, height=480, fps=30, retries=5):
    cap = None
    for i in range(retries):
        cap = cv2.VideoCapture(source)
        if cap.isOpened():
            break
        time.sleep(0.5)
    if not cap or not cap.isOpened():
        raise RuntimeError(f"Failed to open camera source: {source}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap
