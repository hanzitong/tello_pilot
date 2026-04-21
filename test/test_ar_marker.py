

#!/usr/bin/env python3
import cv2
import numpy as np


def main():
    cap = cv2.VideoCapture("/dev/video4", cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Failed to open /dev/video4")

    # 任意設定（必要なら）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # ArUco設定
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    # マーカーの1辺の長さ [m]
    marker_length = 0.10

    # 仮のカメラ内部パラメータ（本来はキャリブレーション値を使う）
    camera_matrix = np.array([
        [600.0,   0.0, 320.0],
        [  0.0, 600.0, 240.0],
        [  0.0,   0.0,   1.0]
    ], dtype=np.float64)

    # 歪み係数（仮）
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        # マーカー検出
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            dictionary,
            parameters=parameters
        )

        if ids is not None and len(ids) > 0:
            # マーカー描画
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 姿勢推定
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                marker_length,
                camera_matrix,
                dist_coeffs
            )

            for i in range(len(ids)):
                # 座標軸描画
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    marker_length * 0.5
                )

                marker_id = int(ids[i][0])
                t = tvecs[i][0]

                text = f"id={marker_id} x={t[0]:.3f} y={t[1]:.3f} z={t[2]:.3f} [m]"
                print(text)

                # テキスト描画
                c = corners[i][0]
                x = int(c[0][0])
                y = int(c[0][1]) - 10

                cv2.putText(
                    frame,
                    text,
                    (x, max(y, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

        cv2.imshow("aruco_pose", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

