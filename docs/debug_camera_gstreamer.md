# デバッグ記録: カメラ GStreamer 問題

## 症状

`opencv_cam` ノードが起動直後に停止し、画像が配信されない。

```
[opencv_cam-1] [ WARN] GStreamer: OpenCV | GStreamer warning: Internal data stream error.
[opencv_cam-1] [ WARN] GStreamer: OpenCV | GStreamer warning: Error in element appsrc0: ...
[opencv_cam-1] width 0, height 0
[opencv_cam-1] EOF, stop publishing
```

---

## 原因

### カメラのピクセルフォーマットと解像度の制約

USB カメラはピクセルフォーマットごとに対応解像度が異なる。

| フォーマット | 特徴 | 最大解像度（例） |
|------------|------|----------------|
| **YUYV** | 非圧縮。CPU/ドライバ負荷が低い | 640×480（カメラ依存） |
| **MJPG** | JPEG 圧縮。帯域を節約できる | 1920×1080 が可能なものが多い |

`opencv_cam` の GStreamer バックエンドは、V4L2 デバイスから映像を取得するパイプラインを自動構築する。
このとき **YUYV** フォーマットが優先されることがあり、YUYV では 1920×1080 に対応していないカメラの場合、
`Internal data stream error` が発生して映像取得に失敗する。

MJPG に対応した GStreamer パイプライン（`image/jpeg` → `jpegdec`）が使われれば 1920×1080 が通るが、
環境によっては GStreamer の MJPG デコードが機能しない場合がある。

---

> **コラム: USB 帯域とフォーマット選択**
>
> USBカメラがどのフォーマットを使うかは、PC側（V4L2/GStreamer）とカメラ間のネゴシエーションで決まる。
> フォーマットによって USB 上を流れるデータ量が大きく異なる。
>
> | フォーマット | USB上のデータ | 640×480@25fps | 1920×1080@30fps |
> |------------|-------------|---------------|----------------|
> | YUYV | **非圧縮**（幅×高さ×2バイト/フレーム） | 約 15 MB/s = 120 Mbps | 約 120 MB/s = **960 Mbps** |
> | MJPG | **JPEG圧縮**（1/5〜1/10程度） | 約 2〜3 MB/s | 約 12〜24 MB/s |
>
> USB 2.0 の実効帯域は約 **480 Mbps**（60 MB/s）。
> YUYV で 1920×1080@30fps を要求すると帯域を約2倍超えるため、
> GStreamer パイプラインが `Internal data stream error` で落ちる。
> これが「1920×1080 は MJPG でなければ通らない」理由であり、
> YUYV に限定することで 640×480 が安定して動く理由でもある。
>
> なお YUYV 640×480@25fps は 120 Mbps で USB 2.0 の帯域内に収まるため問題ない。
> 25fps が上限なのは V4L2 レベルのデバイス側制約（`v4l2-ctl --list-formats-ext` で確認可能）。

---

## 確認方法

### 1. カメラが対応するフォーマット・解像度を確認する

```bash
v4l2-ctl --list-formats-ext --device=/dev/video4
```

出力例（YUYV のみ 640×480 まで、MJPG は 1920×1080 も対応している場合）:

```
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture

    [0]: 'YUYV' (YUYV 4:2:2)
        Size: Discrete 640x480
            Interval: Discrete 0.033s (30.000 fps)
        Size: Discrete 320x240
            ...
    [1]: 'MJPG' (Motion-JPEG, compressed)
        Size: Discrete 1920x1080
            Interval: Discrete 0.033s (30.000 fps)
        Size: Discrete 1280x720
            ...
```

### 2. YUYV で開ける最大解像度を確認する

`v4l2-ctl` の出力で YUYV の `Size: Discrete` を確認する。
それを超えた解像度を `width`/`height` に指定すると GStreamer エラーになる。

---

## 修正

`teleop_sse.launch.py` で使用解像度を切り替えられる変数を設けた:

```python
pc_cam_pixels = [1920, 1080]
usb_cam_pixels = [640, 480]

# GStreamer が MJPG に対応していない環境では usb_cam_pixels (640x480) を使う
cam_pixels = usb_cam_pixels
```

カメラノードと静的TF配信の両方が `cam_pixels` を参照するため、
`cam_pixels` を変更するだけで解像度を一括変更できる:

```python
parameters=[
    {'width': cam_pixels[0]},
    {'height': cam_pixels[1]},
    ...
],
```

```python
arguments=[
    '--x', str(cam_pixels[0]/2/100),
    '--y', str(cam_pixels[1]/2/100),
    ...
]
```

---

## 関連バグ: opencv_cam パラメータ名の誤り

`opencv_cam` ノードの正しいパラメータ名は以下の通り:

| 正しい名前 | 誤った名前（使用不可） |
|-----------|------------------|
| `fps`     | `framerate`      |
| `width`   | `image_width`    |
| `height`  | `image_height`   |

パラメータ名が誤っていても ROS2 は警告なく無視するため、意図した設定が反映されない。
`opencv_cam` のソースコード（`opencv_cam_main.cpp` 内の `declare_parameter`）を参照して確認した。

---

## 汎用的な確認フロー

カメラが映らない場合の切り分け手順:

```
1. デバイスが認識されているか?   → ls /dev/video*
2. 対応フォーマット・解像度は?   → v4l2-ctl --list-formats-ext
3. パラメータ名は正しいか?       → ノードのソースコードで declare_parameter を確認
4. GStreamer パイプラインは動くか? → gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! autovideosink
5. ROS2 ノードから取得できるか?  → ros2 topic echo /cam_image_raw --no-arr
```
