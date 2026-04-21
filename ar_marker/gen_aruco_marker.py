#!/usr/bin/env python3
"""
ArUco マーカー画像を生成するスクリプト。
DICT_4X4_50 の ID 23 を PNG で出力する。

使い方:
  python3 gen_aruco_marker.py
  # → aruco_mark_23.png を生成
  # → tello_pilot/models/marker_23/materials/textures/ に配置
"""

import cv2

MARKER_ID   = 23
MARKER_ID   = 9
MARKER_ID   = 20
MARKER_ID   = 26
MARKER_ID   = 21
MARKER_SIZE = 400  # 出力画像サイズ [px]
OUTPUT_FILE = "aruco_mark_23.png"

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.drawMarker(dictionary, MARKER_ID, MARKER_SIZE)
cv2.imwrite(OUTPUT_FILE, img)
print(f"Generated: {OUTPUT_FILE}  (id={MARKER_ID}, size={MARKER_SIZE}px)")
