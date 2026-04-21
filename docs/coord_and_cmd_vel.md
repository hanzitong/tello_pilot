# 座標系と cmd_vel の注意点

## cmd_vel の座標系

`/pid_vel` および `/cmd_vel` はいずれも **drone_frame（ROS 標準右手座標系）** で表現される。

```
linear.x  > 0  →  前進
linear.y  > 0  →  左移動（ROS 慣習: Y 正 = 左）
linear.z  > 0  →  上昇
angular.z > 0  →  反時計回り yaw（上から見て CCW）
```

## tello_driver による RC コマンドへの変換

`tello_driver_node.cpp` の `cmd_vel_callback` が Twist → Tello RC コマンドに変換する。

```cpp
// tello_driver_node.cpp L98-102
rc << "rc "
   << static_cast<int>(round(msg->linear.y  * -100)) << " "  // lr  = -linear.y * 100
   << static_cast<int>(round(msg->linear.x  *  100)) << " "  // fb  =  linear.x * 100
   << static_cast<int>(round(msg->linear.z  *  100)) << " "  // ud  =  linear.z * 100
   << static_cast<int>(round(msg->angular.z * -100));         // yaw = -angular.z * 100
```

| Twist フィールド | ROS 慣習 | Tello RC | 変換 |
|---|---|---|---|
| `linear.x` | 正 = 前進 | `fb` 正 = 前進 | そのまま |
| `linear.y` | 正 = **左** | `lr` 正 = **右** | 符号反転 |
| `linear.z` | 正 = 上昇 | `ud` 正 = 上昇 | そのまま |
| `angular.z` | 正 = **反時計回り** | `yaw` 正 = **時計回り** | 符号反転 |

`linear.y` と `angular.z` の符号反転は `tello_driver` が担うため、
上流ノード（`pid_controller`・`cmd_multiplexer`）は ROS 慣習のまま出力してよい。

## TF フレームと座標系の流れ

```
camera_frame
  └─(ar_detector_node_2, 毎フレーム)─► marker_23_frame  [tvec m, ArUco 生の姿勢]
      └─(static TF, Rx(π))─► drone_frame
```
（矢印は TF 親子関係。子フレームの位置を親フレームの座標系で表現）

`pid_controller` は `lookupTransform("camera_frame", "drone_frame")` で
ドローンの位置を camera_frame で取得し、`q.inverse()` で drone_frame に変換してから
PID 計算を行う。出力 `pid_vel` は drone_frame 表現になる。

## 注意: linear.y の符号とカメラ画像の関係

PID 設計上の対応（`pid_controller_node.cpp`）:

```
drone_frame の x 誤差 → linear.x（前後）
drone_frame の y 誤差 → linear.y（左右）
```

`linear.y > 0`（ROS 慣習では左移動）は Tello では `lr < 0`（左移動）に変換される。
**カメラ画像上での方向と一致しているかは実機テストで要確認**（Bug D / 課題 E 参照）。

## 速度指令の範囲

Tello RC コマンドの値域は `[-100, 100]`。
`pid_controller` の出力クランプ: `|v| > 0.8` のとき `±0.5` に制限している。
Tello RC に変換すると最大 `±50`（最大値 100 の半分）となる。
