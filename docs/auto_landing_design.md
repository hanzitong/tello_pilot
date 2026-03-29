# 自動着陸 設計ドキュメント

## システム概要

地上固定カメラ（上向き）でドローン底面のARマーカーを検出し、
ドローンがカメラの真上に来たら自動的に着陸させる。

```
[ドローン底面: ARマーカー]
         ↑
[地面: カメラ（上向き）]
```

**着陸シーケンス:**
1. 自動モード（RBボタン）で PID がマーカーを画像中心に引き寄せる
2. `auto_lander` が収束を検出したら `tello_action "land"` を送信
3. ドローンが自律降下・着陸

---

## ノード構成

```
ar_detector ──TF──► pid_controller ──/pid_vel──►
                  ↘                              cmd_multiplexer ──/cmd_vel──► tello_driver ──► Drone
                auto_lander
                    │
                    └──► tello_action "land"
```

| ノード | 役割 |
|--------|------|
| `ar_detector` | ARマーカー検出・TFブロードキャスト |
| `pid_controller` | X/Y方向の速度指令生成 |
| `cmd_multiplexer` | ジョイスティック / 自動モードの切り替え |
| `auto_lander` | 収束検出 → 着陸コマンド送信 |

---

## auto_lander の設計

### 入出力

| 種類 | 名前 | 型 |
|------|------|----|
| Subscribe | `/joy` | sensor_msgs/Joy |
| TF lookup | `camera_center_frame` → `marker_23_frame` | — |
| Service call | `/tello_action` | tello_msgs/TelloAction |

### 状態機械

```
IDLE  ←──────────────────────────────────────────┐
  │ joy.buttons[5] == 1（RBボタン = 自動モード）     │
  ↓                                               │
TRACKING（100ms ごとに TF を確認）                  │
  │ |x| < 閾値 かつ |y| < 閾値 の継続フレーム数をカウント │
  │ → 閾値超または TF 消失 → カウンタリセット ──────►┘
  │ → ボタンOFF → カウンタリセット ────────────────►┘
  │ → N フレーム継続
  ↓
LANDING（tello_action "land" 送信・以後 landed_=true で再トリガーなし）
```

### パラメータ

| 定数 | 値 | 意味 |
|------|----|------|
| `kConvergeThreshold` | `1.0` | 収束とみなす誤差 [ピクセル/100] ≈ 100ピクセル |
| `kConvergeFrames` | `10` | 連続収束フレーム数（= 1秒） |

---

## 閾値の調整方法

収束後にドローンがぶれる場合や、着陸が早すぎる場合は以下を変更する:

```cpp
// src/tello_pilot/src/auto_lander.cpp

static constexpr double kConvergeThreshold = 1.0;  // 小さくすると精度が上がる（着陸しにくくなる）
static constexpr int kConvergeFrames = 10;          // 増やすと安定してから着陸する（時間がかかる）
```

目安:
- 画像が 1920×1080 のとき、`kConvergeThreshold = 1.0` ≈ 中心から 100ピクセル以内
- ホバリングのぶれが大きい場合は `kConvergeThreshold` を大きくする

---

## コントローラー操作

Xbox / PS 系コントローラー（ROS joy デフォルト番号）。

| ボタン | 番号 | 動作 |
|--------|------|------|
| Start（メニュー） | buttons[7] | テイクオフ |
| Back（ビュー） | buttons[6] | 手動着陸 |
| RB（右バンパー） | buttons[5] | 自動モード（PID 追従 + 自動着陸） |
| LB（左バンパー） | buttons[4] | テスト前進（linear.x = 0.5） |
| 右スティック 前後 | axes[4] | 前後スロットル（手動） |
| 右スティック 左右 | axes[3] | 左右ストレーフ（手動） |
| 左スティック 前後 | axes[1] | 上昇・下降（手動） |
| 左スティック 左右 | axes[0] | yaw 回転（手動） |

> **注意:** Start / Back はボタンの立ち上がりエッジで1回だけ送信される（押しっぱなしでも連続送信しない）。

---

## ビルドと起動

```bash
colcon build --packages-select tello_pilot
source install/setup.bash

# 実機（ドローンあり）
ros2 launch tello_pilot teleop_sse.launch.py

# 開発用（ドローンなし、収束検出のみ確認）
ros2 launch tello_pilot cam_dev.launch.py
```

収束カウントは RCLCPP_DEBUG で出力される。確認するには:
```bash
ros2 run tello_pilot auto_lander --ros-args --log-level debug
```
