# デバッグ記録: PID制御器の符号バグ

## 症状

自動モード（RBボタン）に入ると、ドローンがカメラ中心に収束するどころか
「明後日の方向」へ飛び去る。

---

## 調査過程

### Step 1: 症状から仮説を立てる

「目標の逆方向に動く」という症状から、PIDの出力の**符号が逆**になっている可能性を疑った。
符号が逆になる原因として考えられるのは以下の3箇所：

1. AR検出での座標の作り方（`ar_detector_tf2.cpp`）
2. TFルックアップの引数順序（`pid_controller.cpp`）
3. PIDの誤差式の符号（`pid_controller.hpp`）

### Step 2: コードを読んで座標系を追う

各ファイルを読んでデータの流れを追った。

**確認コマンド（ビジュアルデバッグ）**
```bash
# TFツリーの構造を確認
ros2 run tf2_tools view_frames

# 実際のTF値をリアルタイムで確認
ros2 run tf2_ros tf2_echo camera_center_frame marker_23_frame
ros2 run tf2_ros tf2_echo marker_23_frame camera_center_frame

# pid_velの値を確認
ros2 topic echo /pid_vel
```

**データフローを図示**

```
カメラ画像 (pixel座標, 原点=左上, x→右, y→下)
    ↓ ar_detector_tf2.cpp
TF: camera_cv_frame → marker_23_frame
    translation.x = ar_pixel_posx / 100
    translation.y = ar_pixel_posy / 100

TF: camera_cv_frame → camera_center_frame  (static, launchファイルで定義)
    translation.x = 画像幅/2/100 = 9.6  (1920px の場合)
    translation.y = 画像高さ/2/100 = 5.4 (1080px の場合)

    ↓ pid_controller.cpp
lookupTransform("???", "???")  ← ここが問題
    ↓
PIDcompute(translation, 0, dt)
    ↓
pid_vel (geometry_msgs/Twist)
    ↓ tello_driver
rc コマンド (UDP)
    ↓
ドローン
```

### Step 3: lookupTransformの引数を調査する

`lookupTransform(target_frame, source_frame)` のAPIドキュメントを確認。

> Returns the transform from `source_frame` to `target_frame`,
> i.e., the position of `source_frame`'s **origin** expressed in `target_frame` coordinates.

つまり：

- `lookupTransform("A", "B")` → 「BフレームをAフレームから見た位置」を返す
- 欲しいのは「マーカーがカメラ中心からどこにあるか」
  → `lookupTransform("camera_center_frame", "marker_23_frame")`

### Step 4: 具体的な数値で符号を追跡する

マーカーが画像の**右**にある場合（ar_pixel_posx=1200, 画像中心cx=960）で比較した：

```
ar_detector がブロードキャスト:
  camera_cv_frame → marker_23_frame: translation.x = 1200/100 = 12.0
  camera_cv_frame → camera_center_frame: translation.x = 960/100 = 9.6
```

| | 旧コード | 修正後 |
|--|--------|--------|
| lookupTransform | `("marker_23_frame", "camera_center_frame")` | `("camera_center_frame", "marker_23_frame")` |
| translation.x の意味 | カメラ中心をマーカーから見た位置 | マーカーをカメラ中心から見た位置 |
| translation.x の値 | 9.6 - 12.0 = **-2.4**（負） | 12.0 - 9.6 = **+2.4**（正） |
| PID error (current - goal) | -2.4 | +2.4 |
| PID output (kp=0.55, norm=4) | **-0.33** | **+0.33** |
| Tello throttle (×100) | **-33（後退）** | **+33（前進）** |

→ 旧コードはマーカーが右にいるとき「後退」を出力しており、
　マーカーがどんどん遠ざかる**正のフィードバック（発散）**になっていた。

### Step 5: なぜ2つのバグが共存していたか

`pid_controller.hpp` のPID誤差式にも非標準な点があった：

```cpp
// コメントアウトされた標準的な式:
// double error = goal_pos - current_pos;

// 現在使われている式:
double error = current_pos - goal_pos;  // ← 符号が逆
```

これは「TFルックアップが逆だったせいで符号がおかしい」と気づいた開発者が
PID誤差式の符号を反転して補正しようとしたと推測される。

しかし両方を同時に変えると符号が相殺されて元に戻る：

```
TFルックアップ逆 → translation に -1 がかかる
PID誤差逆        → error に -1 がかかる
合計             → (-1) × (-1) = +1 → 変化なし
```

そのため「PIDを直した（つもり）なのにまだ動かない」という状態が続いていた。

---

## 修正箇所

**ファイル**: `src/tello_pilot/src/pid_controller.cpp` L64

```cpp
// 修正前（引数が逆）:
t_tello = tf_buffer_->lookupTransform("marker_23_frame", "camera_center_frame", tf2::TimePointZero);

// 修正後:
t_tello = tf_buffer_->lookupTransform("camera_center_frame", "marker_23_frame", tf2::TimePointZero);
```

`pid_controller.hpp` の `error = current - goal` は**変更しない**。
両方同時に変えると符号が相殺される。

---

## 残存確認事項

修正後のテストで確認が必要な点：

- **軸の対応**: 画像x誤差 → `linear.x`（前後スロットル）、画像y誤差 → `linear.y`（左右ストレーフ）の対応が、カメラとドローンの物理的な向きと一致しているか。不一致なら `linear.x` と `linear.y` の代入を入れ替えるだけで対応可能。
