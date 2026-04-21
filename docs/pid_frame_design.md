# PID 制御の座標フレーム設計

作成日: 2026-04-21

---

## 問題の概要

`pid_controller_node` は `camera_frame` からみた `drone_frame` の位置誤差をもとに
速度指令 (`/pid_vel`) を生成する。
このとき、「どのフレームで PID を計算し、どのフレームで速度指令を出力するか」を
正しく設計しないと、drone の yaw が変化した際に制御が破綻する。

---

## 登場するフレームの整理

| フレーム | 性質 | 説明 |
|----------|------|------|
| `camera_frame` | 固定 | 地面に固定されたカメラの光学フレーム |
| `marker_23_frame` | 動的 | ar_detector が毎フレーム broadcast する ArUco マーカーのフレーム |
| `drone_frame` | 動的 | marker_23_frame に Rx(π) を適用したドローン機体フレーム |

`drone_frame` はドローンの yaw・pitch・roll とともに回転する。
`camera_frame` は常に固定である。

---

## 誤った設計（旧実装）

```
lookupTransform("camera_frame", "drone_frame")
  → 平行移動: [tx, ty, tz]  (camera_frame でのドローン位置)
  → 回転:     q_drone_to_cam

v_drone = R_cam_to_drone * [tx, ty, 0]   ← 位置を drone_frame に回転

pid_vel.linear.x = pid_x_.compute(v_drone.x(), 0, dt)   ← drone_frame で PID
pid_vel.linear.y = pid_y_.compute(v_drone.y(), 0, dt)
```

### なぜ誤りか

PID 制御器は内部に積分器（I 項）と微分器（D 項）を持つ。
これらは「同一フレームで連続する誤差信号」を処理することを前提としている。

旧実装では誤差信号を `drone_frame` に回転してから PID に入力していた。
`drone_frame` の回転行列 `R_cam_to_drone(t)` はドローンが yaw するたびに変化するため、
次の問題が生じる。

**積分項への影響**

yaw 変化前の積分値は「以前の drone_frame の X 軸」方向に蓄積されている。
yaw が変わると drone_frame の X 軸方向が変わるため、
以前の積分値が新しい drone_frame では「X 方向の偏り」として誤って残る。

例:
```
yaw = 0° のとき: drone_X ≈ camera_X
  → ∫e_x dt が camera_X 方向の誤差として蓄積

yaw = 90° に変化した後: drone_X ≈ camera_Y
  → 旧積分値 ∫e_x dt が「camera_Y 方向の偏り」として解釈されてしまう
  → ドローンは camera_Y 方向に引っ張られる（誤動作）
```

**微分項への影響**

同様に、d(e_drone)/dt は「フレームの回転速度」と「実際の誤差変化率」が混合されてしまう。
ドローンが yaw しているだけで d(e_drone.x)/dt ≠ 0 となり、
実際には位置が変化していないのに微分項が出力される。

### P 制御のみの場合は等価

現在のゲイン設定 (kp=1.0, ki=0, kd=0) では、
旧実装と新実装は数学的に同じ値を出力する。

```
旧: output_x = kp * (0 - v_drone.x())
            = kp * (0 - (R_cam_to_drone * [tx, ty, 0]).x)
            = -kp * (R_cam_to_drone * [tx, ty, 0]).x

新: vel_x_cam = kp * (0 - tx) = -kp * tx
    vel_drone  = R_cam_to_drone * [-kp*tx, -kp*ty, 0]
    output_x  = vel_drone.x() = -kp * (R_cam_to_drone * [tx, ty, 0]).x
```

両者は完全に一致する。ただし I・D 項を加えると以降に示す通り設計が重要になる。

---

## 正しい設計（新実装）

```
Step 1: camera_frame（固定フレーム）で PID を計算する
  current = [tx, ty]  (camera_frame でのドローン位置)
  goal    = [0,  0 ]  (camera_frame の原点 = 制御目標)

  vel_x_cam = pid_x_.compute(tx, 0, dt)   ← camera_frame で積分・微分
  vel_y_cam = pid_y_.compute(ty, 0, dt)

Step 2: 速度指令を drone_frame に回転変換してから publish する
  q_drone_to_cam = t.transform.rotation   (drone_frame → camera_frame の回転)

  vel_drone = R_cam_to_drone * [vel_x_cam, vel_y_cam, 0]
            = quatRotate(q_drone_to_cam.inverse(), [vel_x_cam, vel_y_cam, 0])

  pid_vel.linear.x = vel_drone.x()
  pid_vel.linear.y = vel_drone.y()
```

### なぜ正しいか

I 項が積分するのは `tx(t)` と `ty(t)` であり、
これらは `camera_frame` での位置誤差（スカラー）である。
`camera_frame` は固定なので、yaw が変化しても積分軸は変わらない。

積分値が飽和（windup）した場合でも、その方向は常に `camera_frame` の X 軸・Y 軸に対応する。
出力する直前に1回だけ `drone_frame` に回転変換するため、
ドローンの現在姿勢に合わせた方向に速度指令が出力される。

---

## 設計のまとめ

| | 旧実装 | 新実装 |
|--|--------|--------|
| PID の入力フレーム | drone_frame（回転あり） | camera_frame（固定） |
| 積分・微分のフレーム | drone_frame（yaw で変化） | camera_frame（不変） |
| 出力フレーム変換 | なし（drone_frame のまま） | camera_frame → drone_frame に変換 |
| P 制御のみ | 正しく動作 | 正しく動作 |
| I・D 項追加時 | yaw 変化で破綻 | 正しく動作 |

---

## 座標変換の詳細

`lookupTransform("camera_frame", "drone_frame")` が返す変換 T:

- `T.translation` = drone_frame の原点位置を camera_frame で表した座標 `[tx, ty, tz]`
- `T.rotation` (= `q_drone_to_cam`) = drone_frame の姿勢を camera_frame で表したクォータニオン

変換の向き:
```
v_camera = quatRotate(q_drone_to_cam, v_drone)   ← drone → camera
v_drone  = quatRotate(q_drone_to_cam.inverse(), v_camera)   ← camera → drone
```

新実装では、PID 出力 `[vel_x_cam, vel_y_cam, 0]` に対して後者の変換を適用する。

---

---

## Visualizer での座標変換（cmd_vel_img_visualizer_node）

`cmd_vel` / `pid_vel` は drone_frame で表現されているため、
画像に矢印として描画する際は **drone_frame → camera_frame に変換**してから
ピクセルオフセットとして使う必要がある。

### 誤った描画（旧実装）

```cpp
// linear.x/y は drone_frame の値
origin_x + linear.x * scale   // ← drone_frame 値を image X として使っている
origin_y + linear.y * scale   // ← drone_frame 値を image Y として使っている
// → yaw ≠ 0 のとき矢印が実際の移動方向とずれる
```

### 正しい描画（新実装）

```cpp
// q_drone_to_cam: drone_frame → camera_frame の回転（lookupTransform から取得）
// quatRotate(q_drone_to_cam, v_drone) = R_drone_to_cam * v_drone = v_cam
const tf2::Vector3 cmd_cam = tf2::quatRotate(q_drone_to_cam, {cvx, cvy, 0});

origin_x + cmd_cam.x() * scale   // camera_frame X = image X 方向
origin_y + cmd_cam.y() * scale   // camera_frame Y = image Y 方向
// → yaw によらず、矢印が画像上の実際の移動方向を正しく示す
```

### なぜ変換後が正しいか

pid_controller_node は camera_frame で PID を計算し、その出力を drone_frame に変換して publish している。
visualizer でその逆変換（drone_frame → camera_frame）を行うと、
元の camera_frame での速度指令（= 画像平面上の移動方向）に戻る。

```
PID 出力: vel_cam = [-kp*tx, -kp*ty, 0]    (常に画像中心方向を向く)
     ↓ drone_frame に変換 (pid_controller)
cmd_vel: vel_drone = R_cam_to_drone * vel_cam
     ↓ camera_frame に戻す (visualizer)
表示用: R_drone_to_cam * vel_drone = vel_cam  ← 元の値に戻る ✓
```

drone が yaw しても `vel_cam` は常に `[-kp*tx, -kp*ty]` なので、
矢印は常にドローンの投影位置から画像中心方向を向く。

---

## 関連ファイル

| ファイル | 内容 |
|----------|------|
| `src/pid_controller_node.cpp` | 本設計を実装（camera_frame で PID、drone_frame に変換） |
| `src/cmd_vel_img_visualizer_node.cpp` | drone_frame → camera_frame 変換してから矢印描画 |
| `include/pid_controller.hpp` | PID 制御器クラス（error = goal - current） |
| `docs/coord_and_cmd_vel.md` | cmd_vel 座標系・tello_driver 変換表 |
