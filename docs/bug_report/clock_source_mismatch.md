# 時刻源の不一致による誤 staleness 検出

## 症状

ARマーカーを認識しているはずなのに、`pid_controller_node` が
「マーカーロスト」として PID 出力をゼロにし続ける。

```
[pid_controller_node] [WARN] Stale TF (165.62s): marker lost. Stopping PID.
[pid_controller_node] [WARN] Stale TF (166.72s): marker lost. Stopping PID.
[pid_controller_node] [WARN] Stale TF (167.72s): marker lost. Stopping PID.
...（1秒ごとに 1s ずつ増加し続ける）
```

---

## 原因の分析

### 「165s」の意味

`(this->now() - tf_stamp).seconds() = 165` は、**2つの時刻が同じエポックを使っており、TF が 165 秒前に最後に更新されたことを示している。**

もし V4L2 の `CLOCK_MONOTONIC`（OS 起動後の相対時刻 ≈ 165 s）と
ROS の `CLOCK_REALTIME`（Unix エポック基準 ≈ 1,776,740,400 s）のずれが原因なら、
差は約 **17 億秒** になるはずで、`165s` にはならない。

```
CLOCK_MONOTONIC ≈ 165 s
CLOCK_REALTIME  ≈ 1,776,740,400 s
差              ≈ 1,776,740,235 s  ← "165.62s" とは程遠い
```

→ 時刻源の種類の違いは今回の直接原因ではない。

### 本当の原因: TF が 165 秒間 broadcast されていない

`ar_single_detector_node` は `/camera_info` を受信するまで
`camera_matrix_ready_ = false` のまま `imageCallback` の先頭で `return` する。

```cpp
if (!camera_matrix_ready_) {
    RCLCPP_WARN_THROTTLE(..., "Waiting for /camera_info ...");
    return;   // ← TF は broadcast されない
}
```

`/camera_info` が正しいトピック名で届いていない場合、
このノードは TF を一切 broadcast しない。

### tf2::TimePointZero が例外を投げない理由

`lookupTransform(target, source, tf2::TimePointZero)` は
「バッファ内の最新の値を返す」という意味で、
**キャッシュの有効期限（デフォルト 10 秒）に関わらず例外を投げない。**

```
ar_single_detector_node:  TF broadcast 停止（camera_info 未受信）
                         ↓
TF2 バッファ:  最後に受け取った TF を保持し続ける（期限に関係なく）
                         ↓
pid_controller:  lookupTransform 成功（例外なし）→ 165 秒前の TF を受け取る
                         ↓
staleness チェック発火  "Stale TF (165.62s)"
```

これは Bug F（`tf2::TimePointZero` によるマーカーロスト後の古い TF 使用）と
同じ根本メカニズムである。

---

## データフローと問題箇所

```
usb_cam
  └─ 画像を publish（/camera/image_raw）
  └─ カメラ情報を publish（/camera_info または別トピック？）

ar_single_detector_node
  └─ /camera_info 未受信 → camera_matrix_ready_ = false
  └─ imageCallback で即 return → TF broadcast なし

pid_controller_node
  └─ lookupTransform("camera_frame", "drone_frame", TimePointZero)
       → 165 秒前の古い TF が返る（例外なし）
       → staleness チェック: 165 s >> kMaxStaleSec (0.5 s) → 発火
```

---

## 修正

### 根本原因: /camera_info の受信確認

`ar_single_detector_node` が `/camera_info` を受信しているか確認する。

```bash
ros2 topic list | grep camera_info
ros2 topic hz /camera_info
```

usb_cam が `/camera_info` を別トピック名（例: `/usb_cam_node/camera_info`）で
publish している場合は launch ファイルに remapping を追加する。

### 副次的修正: TF スタンプを this->now() に統一

`ar_single_detector_node.cpp` の TF スタンプを `msg->header.stamp` から `this->now()` に変更する。
将来カメラドライバが異なる時刻源（V4L2 の `CLOCK_MONOTONIC` など）を
使うようになった場合の保険になる。

```cpp
// 変更前
ts.header.stamp = msg->header.stamp;

// 変更後
ts.header.stamp = this->now();   // ROS クロックで統一
```

---

## 教訓

### 1. tf2::TimePointZero はキャッシュ期限を無視する

`TimePointZero`（最新値要求）は TF2 の cache_time（デフォルト 10 秒）に関係なく
バッファ内の最後の値を返す。TF の broadcast が止まっても例外にならず、
古い値がそのまま使われ続ける。→ **Bug F の根本メカニズム**

### 2. 「差が 1 s/s で増加する」= TF が更新されていない

staleness の差が 1 秒ごとに 1 秒増えるのは、
`this->now()` は進むが `tf_stamp` が固定（TF が更新されていない）ためである。

- 差が一定: 時刻のオフセットが固定（時刻源の違いなど）
- 差が 1 s/s で増加: TF が更新されていない（broadcast 停止）

### 3. 時刻源の種類と差の大きさ

```
CLOCK_MONOTONIC（OS 起動後の相対時刻）≈ 数百秒〜数万秒
CLOCK_REALTIME（Unix エポック基準）   ≈ 17 億秒（2025年現在）
差                                   ≈ 17 億秒
```

ログに出る差が小さい（数百秒程度）なら、両者は同じエポックを使っており、
TF の更新停止が原因である可能性が高い。
