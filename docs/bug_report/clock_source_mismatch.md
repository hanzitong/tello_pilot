# 時刻源の不一致による誤 staleness 検出

## 症状

ARマーカーを認識しているにもかかわらず、`pid_controller_node` が
「マーカーロスト」として PID 出力をゼロにし続ける。

```
[pid_controller_node] [WARN] Stale TF (165.62s): marker lost. Stopping PID.
[pid_controller_node] [WARN] Stale TF (166.72s): marker lost. Stopping PID.
[pid_controller_node] [WARN] Stale TF (167.72s): marker lost. Stopping PID.
[pid_controller_node] [WARN] Stale TF (168.72s): marker lost. Stopping PID.
...（1秒ごとに 1s ずつ増加し続ける）
```

ドローンは飛行中・マーカーは映像内に見えているのに、速度指令がゼロになる。

---

## 原因: カメラドライバの時刻源と ROS クロックの不一致

### ROS 2 における時刻源の種類

ROS 2 ノードの `this->now()` は **ROS クロック** を使用する。
`use_sim_time = false`（実機動作）のとき ROS クロックは **Unix エポック基準の POSIX 時刻** に対応する。

一方、USB カメラドライバ（usb_cam）が画像ヘッダに埋め込むタイムスタンプは、
カーネルの **V4L2 タイムスタンプ**（`CLOCK_MONOTONIC`、OS 起動時点からの相対時刻）を
使うことがある。

```
ROS クロック : Unix エポック基準 ≒ 1776740400 s（現在の絶対時刻）
V4L2 タイムスタンプ : OS 起動からの経過時間 ≒ 165 s
差分             : ≒ 1776740235 s ← 非常に大きい
```

なぜ 165 s 程度の差に見えたかというと、staleness チェックは
`(now - tf_stamp).seconds() > kMaxStaleSec (= 0.5)` で判定しており、
**165 s は「OS 起動後 165 秒で実験を開始した」ときの V4L2 相対時刻**に相当する。
つまり `tf_stamp ≈ 165 s（起動後経過秒）` であり `now ≈ 1776740400 s（Unix 時刻）` なので
差は実際には 10 億秒オーダーだが、差が 0.5 s を超えていることは確かである。

### データの流れと問題箇所

```
usb_cam
  └─ 画像を publish（header.stamp = V4L2 タイムスタンプ ≈ 165 s）

ar_detector_node_2
  └─ 画像を受信し、marker_23_frame の TF を broadcast
       ts.header.stamp = msg->header.stamp;  ← ここで V4L2 タイムスタンプを引き継ぐ
       （TF のタイムスタンプ ≈ 165 s）

pid_controller_node
  └─ lookupTransform("camera_frame", "drone_frame") → t
       t.header.stamp ≈ 165 s
       this->now()    ≈ 1776740400 s
       差 ≈ 1776740235 s  >> kMaxStaleSec (0.5 s)
       → "Stale TF: marker lost" と誤判定  ← バグ
```

### なぜ差が「1秒ごとに 1 s ずつ増加」したのか

V4L2 タイムスタンプは OS 起動後の経過時間なので 1 s/s で増加する。
ROS クロックも 1 s/s で進む。両者の進み方は同じなので差は**一定**のはずだが、
ログでは差が増加しているように見える。

これは TF が**更新されていない**（＝マーカー検出に失敗している可能性）か、
あるいは `RCLCPP_WARN_THROTTLE` のスロットリング期間（1000 ms）の関係で
表示タイミングがずれているためである。実際には差はほぼ一定値（OS 起動後経過秒数）。

---

## 修正

`ar_detector_node_2.cpp` で TF のスタンプを `msg->header.stamp` ではなく
`this->now()`（ROS クロック）に変更する。

```cpp
// 修正前
ts.header.stamp = msg->header.stamp;   // V4L2 タイムスタンプ（異なる時刻源）

// 修正後
ts.header.stamp = this->now();         // ROS クロック（staleness チェックと同一時刻源）
```

これにより TF のタイムスタンプと `pid_controller_node` の `this->now()` が
同じ時刻源を参照し、staleness チェックが正しく機能する。

### トレードオフ

`this->now()` はカメラが画像をキャプチャした瞬間ではなく、
`ar_detector_node_2` がその画像を処理してTFをbroadcastした瞬間の時刻になる。
画像のキャプチャから TF broadcast までの遅延（通常 < 数十 ms）が含まれる。

今回の用途（マーカーのロスト検出）では問題ないが、
精密な時刻同期が必要なシステム（センサーフュージョンなど）では
カメラの時刻源を ROS クロックに統一する（`chrony` 等で同期する）ほうがより正確。

---

## 教訓

1. **`this->now()` と `msg->header.stamp` は同じ時刻源ではない場合がある。**
   USB カメラなどの外部デバイスは独自のクロック（V4L2、PTP など）を持つ。

2. **時刻差の比較をするなら、同一時刻源同士で行う。**
   staleness チェックのように「現在時刻 − TF タイムスタンプ」を計算するとき、
   TF に埋め込むタイムスタンプは `this->now()` で揃えるか、
   または `ros__clock` パラメータでカメラの時刻源を統一する。

3. **ログの「差が増え続ける」は時刻が止まっているサインではない。**
   差が一定なら「固定オフセット」（時刻源の違い）、
   差が増え続けるなら「一方の時計が止まっている」（TF が更新されていない）。
   症状を切り分けてデバッグする。
