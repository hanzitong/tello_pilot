# 設計: 座標系整理と ar_detector_node_2

## 目的

1. **3D→2D 丸め問題の解消**
   `ar_detector_node` は `estimatePoseSingleMarkers` で3Dポーズを取得しながら、
   TF に `(pixel_x/100, pixel_y/100, 0)` という2D値を登録していた。
   深度情報を捨てており、将来の拡張（高度制御など）を阻む。

2. **drone_frame の導入**
   ARマーカーのX軸回りに180°回転するとドローン座標系になる物理的事実を TF に反映する。
   cmd_vel を drone_frame 基準で生成することで、軸対応が自然になる。

3. **Bug F 修正（同時適用）**
   `tf2::TimePointZero` による古い TF 問題（bug_report.md Bug F）を
   pid_controller_node / auto_lander_node の修正時に同時に解消する。

---

## TF ツリー（変更後）

```
camera_frame
  └─(ar_detector_node_2 が毎フレーム broadcast)─► marker_23_frame
      │  translation: tvec [m]（estimatePoseSingleMarkers の生の値）
      │  rotation   : rvec → 四元数（R_x180 補正なし）
      └─(static, Rx(180°))─► drone_frame
          translation: (0, 0, 0)
          rotation   : roll=π, pitch=0, yaw=0
```

**変更前との差分:**
| | 変更前 | 変更後 |
|---|---|---|
| marker_23_frame の translation | pixel/100 の2D | tvec [m] の3D |
| marker_23_frame の rotation | R_x180 補正済み | 生の ArUco 姿勢 |
| drone_frame | なし | 静的 TF で定義 |
| camera_center_frame | 静的 TF で定義（不要） | **削除** |

---

## camera_center_frame 廃止の理由

`estimatePoseSingleMarkers` が返す tvec は camera_frame 原点（光学中心）基準。
したがって「カメラ光学中心からのドローン変位」は
`lookupTransform("camera_frame", "drone_frame").translation` で直接得られる。
画素ベースの中心オフセット (`width/2/100`, `height/2/100`) は不要になる。

---

## pid_controller_node の誤差計算（変更後）

```
t = lookupTransform("camera_frame", "drone_frame")
  → t.translation = カメラフレームでのドローン位置 [m]

v_cam = (t.translation.x, t.translation.y, 0)   ← camera_frame での誤差ベクトル

q = t.rotation  (drone_frame → camera_frame の回転四元数)
v_drone = q^(-1) * v_cam                          ← drone_frame で表現した誤差

cmd_vel.linear.x = pid_x.compute(v_drone.x, 0, dt)
cmd_vel.linear.y = pid_y.compute(v_drone.y, 0, dt)
```

v_drone.x/y が drone_frame 座標系での誤差なので、
cmd_vel がそのままドローンの前後・左右速度指令になる。

---

## 変更ファイル一覧

| ファイル | 変更内容 |
|---------|---------|
| `src/ar_detector_node_2.cpp` | **新規作成** — 3D pose broadcast, camera_info 購読 |
| `CMakeLists.txt` | ar_detector_node_2 の追加 |
| `launch/tello_system_dev1.launch.py` | camera_center_frame 削除, drone_frame 追加, ノード差し替え |
| `src/pid_controller_node.cpp` | drone_frame 基準に変更, Bug F 修正, ゲイン更新 |
| `src/auto_lander_node.cpp` | drone_frame 基準に変更, Bug F 修正, 閾値更新 |
| `CLAUDE.md` | TF ツリー記述更新 |

---

## PID ゲインについて

変更前の誤差単位: [pixel/100]（例: 画像端 → 誤差 ≒ 3〜6）
変更後の誤差単位: [m]（例: 横に 0.1m ずれ → 誤差 0.1）

`kp = 0.55, pidxy_norm = 4` では有効ゲイン ≒ 0.14 m/s per unit。
単位変更後は誤差が 1/10〜1/30 倍になるため、大幅に調整が必要。
初期値として `kp = 1.0, pidxy_norm = 1` を設定する（実機テストで調整）。

---

## auto_lander 収束閾値

変更前: `kConvergeThreshold = 1.0` [pixel/100] = 100 pixel
変更後: `kConvergeThreshold = 0.10` [m] = 10 cm（実機で調整）

---

## staleness チェック（Bug F 修正）

`lookupTransform` 成功後、TF のタイムスタンプが現在時刻より
`kMaxStaleSec = 0.5` 秒以上古い場合はマーカーロストとして扱う。

```cpp
rclcpp::Time tf_stamp(t.header.stamp.sec, t.header.stamp.nanosec, RCL_ROS_TIME);
if ((this->now() - tf_stamp).seconds() > kMaxStaleSec) {
    // pid_controller: ゼロ出力 publish して return
    // auto_lander   : converge_count_ リセットして return
}
```

---

## 軸対応の確認（実機テスト事項）

`v_drone.x/y → cmd_vel.linear.x/y` の符号は実機テストで確認が必要。
`cmd_vel_visualizer_node` の黄色矢印（/pid_vel）を見ながら検証する。
不一致の場合は `pid_controller_node.cpp` の符号を調整する。
