# バグレポート

コードレビューで発見・修正したバグの記録。

---

## 修正済みバグ

### Bug A — teleop_sse.launch.py: カメラ解像度の不一致（致命的）

**ファイル:** `src/tello_pilot/launch/teleop_sse.launch.py` L53-54
**症状:** ドローンが画像中心ではなく大きくずれた位置へ収束しようとする

**原因:**
カメラ(`opencv_cam`)は `pc_cam_pixels`(1920×1080) で起動しているのに、
`camera_center_frame` の static_transform には `usb_cam_pixels`(640×480) の半分を使っていた。

```
期待: center = (9.6, 5.4)  ← 1920/2/100, 1080/2/100
実際: center = (3.2, 2.4)  ← 640/2/100,  480/2/100
```

TF の原点ずれが PID の目標座標に直接影響するため、約 6.4（640ピクセル分）のオフセットが発生していた。

**修正:** `usb_cam_pixels` → `pc_cam_pixels` に変更

---

### Bug B — ar_detector_tf2.cpp: land_direction のユニット不整合（中）

**ファイル:** `src/tello_pilot/src/ar_detector_tf2.cpp` L127-128
**症状:** `/land_direction` トピックの値が意味をなさない

**原因:**
`translation.x/y` は `[pixel/100]` 単位（例: pixel 1200 → 12.0）だが、
`cx`, `cy` は raw pixel 単位（例: 960）のまま引き算していた。

```cpp
// 修正前（ユニット不整合）
land_dir.vector.x = -1. * (tf_ar.transform.translation.x - cx);
// = 12.0 - 960 = -948  ← 意味のない値

// 修正後
land_dir.vector.x = -1. * (tf_ar.transform.translation.x - cx / 100.);
// = 12.0 - 9.6 = 2.4  ← 正しい差分
```

> **注:** `/land_direction` は現在の制御系では使用されていない（`auto_lander` と `pid_controller` は TF を直接参照）。制御への影響はないが、将来使用する場合に備えて修正した。

---

### Bug C — cmd_multiplexer.cpp: joy ボタン配列への境界なしアクセス（低）

**ファイル:** `src/tello_pilot/src/cmd_multiplexer.cpp` L88, L92
**症状:** ボタン数の少ないジョイスティックを接続するとクラッシュ（`std::out_of_range`）

**原因:**
`last_joy_.buttons[5]` や `last_joy_.buttons[4]` へのアクセスに境界チェックがなかった。
（`rising_edge()` は内部でチェックしていたが直接アクセス箇所は未対応）

**修正:** アクセス前に `buttons.size()` と `axes.size()` を確認するよう変更

---

### Bug D — cam_dev.launch.py: pid_controller への不要な引数（低）

**ファイル:** `src/tello_pilot/launch/cam_dev.launch.py` L94
**症状:** なし（サイレント）

**原因:**
`pid_controller` に解像度引数 `[1920, 1080]` を渡していたが、
本体（`pid_controller.cpp`）ではコマンドライン引数をパースしていない（コメントアウト済み）。
引数は無視されるが、コードと実態が乖離していた。

**修正:** 引数指定を削除

---

---

### Bug E — CMakeLists.txt: 実行ファイル名と launch ファイルの不一致（致命的）

**ファイル:** `src/tello_pilot/CMakeLists.txt`
**症状:** `ros2 launch` 実行時に `executable not found` エラーでノードが起動しない

**原因:**
`CMakeLists.txt` の `add_executable` で付けた名前（`_node` サフィックスあり）と、
launch ファイルの `executable` パラメータが一致していない。

| CMakeLists.txt（インストールされるバイナリ名） | launch の executable 指定 |
|---------------------------------------------|--------------------------|
| `ar_detector_node` | `ar_detector` |
| `pid_controller_node` | `pid_controller` |
| `cmd_multiplexer_node` | `cmd_multiplexer` |
| `auto_lander_node` | `auto_lander` |

**修正方針:**
CMakeLists.txt の `add_executable` 名から `_node` サフィックスを除いて launch 側に統一する。
（launch ファイルを変更する方法もあるが、ノード名は短い方が `ros2 run` での利便性が高い）

---

---

### Bug F — pid_controller / auto_lander: マーカーロスト後も古い TF で制御が継続する（致命的）✅ 修正済み

**ファイル:**
- `src/tello_pilot/src/pid_controller_node.cpp`
- `src/tello_pilot/src/auto_lander_node.cpp`

**症状:**
ARマーカーを一度検出してからロスト（視野外に出る・遮蔽など）すると、
ロスト前の最後の位置情報を使ったまま PID 速度指令の出力が継続する。
最悪の場合、`auto_lander` が誤って着陸コマンドを発行する。

**原因:**
`tf2::TimePointZero` は TF2 バッファ内の最新値をそのまま返す。
デフォルトキャッシュ保持時間 10 秒以内は `TransformException` が投げられないため、
古い TF 値がそのまま使われ続ける。

**修正内容:**
`lookupTransform` 成功後に TF の timestamp を確認し、
`kMaxStaleSec = 0.5` 秒以上古い場合はマーカーロストとして扱う。

```cpp
const rclcpp::Time tf_stamp(t.header.stamp.sec, t.header.stamp.nanosec, RCL_ROS_TIME);
if ((this->now() - tf_stamp).seconds() > kMaxStaleSec) {
    // pid_controller: ゼロ出力 / auto_lander: converge_count_ リセット
    return;
}
```

座標系整理（drone_frame 導入）と同時に適用した。

---

## 未修正の既知の課題（実機テストが必要）

### 課題 E — 画像軸とドローン軸の対応

**ファイル:** `src/tello_pilot/src/pid_controller.cpp` L77-78
**詳細:**

```
画像 x 方向（左右）→ pid_vel.linear.x → tello rc fb（前後）
画像 y 方向（上下）→ pid_vel.linear.y → tello rc lr（左右）
```

この対応がカメラとドローンの物理的な向きと一致しているかは実機テストで確認が必要。
不一致の場合は `linear.x` と `linear.y` の代入を入れ替えるか、符号を反転する。

---

## 調査したが問題なしと判断した項目

| 項目 | 判断理由 |
|------|---------|
| `last_pid_` 未初期化 | `geometry_msgs::msg::Twist` はゼロ値で default 構築される |
| PID error 符号 | 前回のデバッグで修正済み（TF lookup の引数順序を修正）|
| `tf2::TimePointZero` | ~~最新 TF を取得する意図的な使用~~ → Bug F として再分類（古い TF がキャッシュから返される問題） |
| 収束判定の `async_send_request` | サービス未接続時も non-fatal（cam_dev では意図的にサービスなし）|
