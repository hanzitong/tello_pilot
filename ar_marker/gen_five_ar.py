

import cv2
import os

# 保存先ディレクトリ
output_dir = "./"
os.makedirs(output_dir, exist_ok=True)

# 使用する辞書
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 生成したいID
marker_ids = [23, 9, 20, 21, 26]

# 画像サイズ [pixel]
marker_size = 400

for marker_id in marker_ids:
    img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    filename = os.path.join(output_dir, f"aruco_{marker_id}.png")
    cv2.imwrite(filename, img)
    print(f"saved: {filename}")


