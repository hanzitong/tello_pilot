#!/usr/bin/env python3

"""
Simple script to generate an ArUco marker image (DICT_4X4_50, ID 23).
Usage:
  python3 generate_marker.py
This will produce `marker23.png` in the current directory.
"""
import cv2


def main():
    # Choose the predefined dictionary and marker ID
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_id = 23
    side_pixels = 700  # Size of the marker image in pixels

    # Draw and save the marker
    marker = cv2.aruco.drawMarker(aruco_dict, marker_id, side_pixels)
    # marker = cv2.aruco.drawMarker(aruco_dict, marker_id, side_pixels, None, borderBits=5)
    output_filename = f"marker{marker_id}.png"
    cv2.imwrite(output_filename, marker)
    print(f"Generated {output_filename} ({side_pixels}×{side_pixels}px) in current directory.")


if __name__ == '__main__':
    main()

