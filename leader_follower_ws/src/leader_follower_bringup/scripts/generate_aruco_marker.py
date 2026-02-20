#!/usr/bin/env python3
"""Generate ArUco marker ID 42 (DICT_4X4_50) as a 300x300 PNG with white border."""
import cv2, numpy as np, os

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_img = cv2.aruco.generateImageMarker(aruco_dict, 42, 200)
final_img = np.ones((300, 300), dtype=np.uint8) * 255
final_img[50:250, 50:250] = marker_img

out = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'models', 'aruco_marker_42', 'materials', 'textures', 'aruco_marker_42.png'
))
os.makedirs(os.path.dirname(out), exist_ok=True)
cv2.imwrite(out, final_img)
print(f"ArUco marker ID 42 saved to: {out}")
