# 修正計画: tello_system_dev1.launch.py を正常に動かす

## 目的

`ros2 launch tello_pilot tello_system_dev1.launch.py` でシステム全体
（usb_cam + ar_detector + pid_controller + cmd_multiplexer + auto_lander + tello_driver）を
正常に起動・動作させる。

---

## 調査済みの問題一覧

### 問題 A: launchファイルのインストールパスが二重ネスト【致命的】

**ファイル:** `src/tello_pilot/CMakeLists.txt` L98-100

**現状:**
```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)
```

`DIRECTORY launch` は「launchディレクトリそのもの」を `share/tello_pilot/launch/` 以下にコピーする。
結果として：

```
share/tello_pilot/launch/launch/tello_system_dev1.launch.py  ← 実際にインストールされるパス
share/tello_pilot/launch/tello_system_dev1.launch.py         ← ros2 launchが探すパス
```

`ros2 launch tello_pilot tello_system_dev1.launch.py` は
`{share_dir}/launch/tello_system_dev1.launch.py` を探すため、ファイルが見つからずエラーになる。

**修正:**
```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}   # /launch を除く
)
```

`DESTINATION` から末尾の `/launch` を削除することで、
`launch/` ディレクトリが `share/tello_pilot/` 直下に正しく配置される。

---

### 問題 B: `ar_detector_node.cpp` のカメラ行列がダミー値【制御への影響なし】

**ファイル:** `src/tello_pilot/src/ar_detector_node.cpp` L72-74

**現状:**
```cpp
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);    // TODO: put real value
cv::Mat distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);  // TODO: put real value
```

`ost.yaml`（640×480 の実測キャリブレーション値）は `usb_cam` ノードに読み込まれているが、
`ar_detector` はそれを使わず単位行列・ゼロ歪みを使用している。

- `estimatePoseSingleMarkers` の3Dポーズ推定が不正確になる
- `drawAxis` で描画される軸が実際の向きとずれる
- **制御系への影響: なし**（PIDは画像ピクセル座標を使用するため）

**対応方針:**
現時点では対応不要。将来、3Dポーズ（高度推定など）を利用する場合に
`/camera_info` トピックを購読してカメラ行列を取得するよう修正する。

---

### 問題 C: rviz2 に config ファイルが未指定【軽微】

**ファイル:** `src/tello_pilot/launch/tello_system_dev1.launch.py` L118-122

**現状:**
```python
rviz = Node(
    package='rviz2',
    executable='rviz2',
    output='screen',
)
```

rviz2 が空の状態で起動する。`/image_ar`（AR検出済み画像）や TF ツリーを可視化できない。

**対応方針:**
`config/` ディレクトリに RViz 設定ファイルを作成し、arguments で指定する。
現時点では動作確認上の障害ではないため、後回しでよい。

---

### 問題 D: 画像軸とドローン軸の対応【実機テストで確認が必要】

**ファイル:** `src/tello_pilot/src/pid_controller_node.cpp` L79-80

`pid_vel.linear.x`（画像X誤差から生成）が Tello の `fb`（前後）に、
`pid_vel.linear.y`（画像Y誤差）が `lr`（左右）に対応している。

```
画像 x 方向（左右）→ pid_vel.linear.x → rc fb（前後）
画像 y 方向（上下）→ pid_vel.linear.y → rc lr（左右）
```

この対応がカメラとドローンの物理的な向きと一致しているかは実機でのみ確認できる。
不一致の場合は `pid_controller_node.cpp` の `linear.x` と `linear.y` の代入を入れ替えるか、符号を反転する。

---

## TF ツリー（参照用）

```
camera_frame
  ├─(static: launch で定義)──► camera_center_frame
  │   translation: (3.2, 2.4, 0)  [= 640/2/100, 480/2/100]
  └─(ar_detector が毎フレーム broadcast)──► marker_23_frame
      translation: (ar_pixel_x/100, ar_pixel_y/100, 0)
```
（矢印はTF親子関係。子フレームの位置を親フレーム座標系で表現）

`pid_controller` は `lookupTransform("camera_center_frame", "marker_23_frame")` を使って
マーカーの画像中心からのオフセット `(ar_pixel_x/100 - 3.2, ar_pixel_y/100 - 2.4, 0)` を得る。

---

## トピック接続図（参照用）

```
usb_cam ─/image_raw(remap→)─/camera/image_raw──► ar_detector ─TF──► pid_controller ─/pid_vel──► cmd_multiplexer ─/cmd_vel──► tello_driver
                                                                TF──► auto_lander ─/tello_action──►
joy ─/joy──► cmd_multiplexer
         └──► auto_lander
```
（矢印はデータの流れの方向）

---

## 修正手順

### Step 1: CMakeLists.txt を修正する

`src/tello_pilot/CMakeLists.txt` の launch インストール行を変更する。

```cmake
# 変更前
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# 変更後
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

### Step 2: 再ビルド

```bash
colcon build --packages-select tello_pilot
source install/setup.bash
```

### Step 3: launchファイルが見つかるか確認

```bash
ros2 launch tello_pilot tello_system_dev1.launch.py --show-args
```

エラーなく引数一覧が表示されれば、インストールパスの問題は解消されている。

### Step 4: 起動・動作確認

```bash
# ドローンの電源を入れ Wi-Fi 接続後、カメラ・コントローラーを接続してから実行
ros2 launch tello_pilot tello_system_dev1.launch.py
```

確認すべきトピック:
```bash
ros2 topic hz /cam_image_raw           # 25Hz 前後であること
ros2 topic echo /pid_vel               # マーカー検出後に値が出ること
ros2 run tf2_tools view_frames         # TF ツリーの接続を確認
ros2 run tf2_ros tf2_echo camera_center_frame marker_23_frame  # 誤差が出ること
```

### Step 5: 軸対応の確認（問題 D）

RBボタンを押しながらARマーカーを画像右に置いたとき:
- `pid_vel.linear.x > 0` → ドローンが前進するなら ✓
- ドローンが後退するなら `pid_controller_node.cpp` の `linear.x` 符号を反転

同様に画像下方向でも確認する。

---

## 残存する技術的負債

| 項目 | 対応優先度 | 内容 |
|------|-----------|------|
| カメラ行列の実値化 | 低 | ar_detector が ost.yaml の値を使うよう修正 |
| RViz config 作成 | 低 | AR検出画像・TFを可視化するためのconfigを作成 |
| 収束閾値のパラメータ化 | 低 | auto_lander の kConvergeThreshold/kConvergeFrames をROSパラメータにする |
