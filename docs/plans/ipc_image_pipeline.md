# IPC（Intra-Process Communication）導入計画: 画像パイプライン

作成日: 2026-04-21

## 目的

640×480 BGR 画像 ≈ 900 KB/フレームを、DDS 経由（シリアライズ＋コピー）から
同一プロセス内のポインタ渡しに切り替え、遅延と CPU 負荷を削減する。

```
現状: usb_cam プロセス → シリアライズ → DDS → デシリアライズ → ar_detector プロセス
目標: [同一プロセス内] usb_cam ノード → shared_ptr 参照渡し → ar_detector ノード
```

---

## IPC が有効になる条件（QoS 制約）

以下の条件を1つでも外れると、エラーなく DDS 通信にフォールバックする。

| 条件 | 現状 |
|------|------|
| 同一プロセス内（ComposableNodeContainer）に配置 | ✗ 未実装 |
| Durability が **Volatile** | ✓ SensorDataQoS はVOLATILE |
| `use_intra_process_comms(true)` を **PubとSubの両方** に設定 | ✗ 未実装 |
| `rclcpp_components` による ComposableNode 形式で起動 | ✗ 未実装 |

---

## 2サブスクライバー時の zero-copy 制限

`/camera/image_raw` を ar_detector_node_2 と cmd_vel_visualizer_node の2ノードが
購読する。複数サブスクライバーがいる場合の IPC 動作:

- パブリッシャーが `unique_ptr` で publish → IPC マネージャーが `shared_ptr` に昇格させ参照を配布
- コピーは最大1回（DDS の シリアライズ＋2回コピーよりは改善）
- 「完全 zero-copy」ではないが、DDS シリアライズの排除効果は得られる

---

## usb_cam の ComposableNode 対応

`ros2 component types` で `usb_cam::UsbCamNode` の登録を確認済み。
ComposableNode として起動可能。plugin 名: `"usb_cam::UsbCamNode"`

**注意**: `ComposableNode` は `respawn=True` に非対応。
usb_cam の自動再起動が必要な場合は後述の「部分 IPC」構成を検討する。

---

## 変更が必要なファイルと変更内容

### 1. `src/ar_detector_node_2.cpp` と `src/cmd_vel_visualizer_node.cpp`

各ファイルに共通して3箇所変更する。

**変更 1: インクルード追加**
```cpp
#include <rclcpp_components/register_node_macro.hpp>
```

**変更 2: コンストラクタのシグネチャ変更**
```cpp
// 変更前
ClassName() : Node("node_name")

// 変更後
explicit ClassName(const rclcpp::NodeOptions & options)
    : Node("node_name", options)
```

`NodeOptions` を受け取ることで、ComposableNodeContainer が
`use_intra_process_comms(true)` を含む NodeOptions を外から注入できるようになる。

**変更 3: main 関数を削除し、コンポーネント登録マクロを追加**
```cpp
// main() 関数全体を削除する

// ファイル末尾に追加
RCLCPP_COMPONENTS_REGISTER_NODE(ClassName)
```

---

### 2. `CMakeLists.txt`

**変更 1: find_package 追加**
```cmake
find_package(rclcpp_components REQUIRED)
```

**変更 2: add_executable → add_library(SHARED) + rclcpp_components_register_node に変更**

ar_detector_node_2 と cmd_vel_visualizer_node の2ターゲットに適用する。

```cmake
# 変更前
add_executable(ar_detector_node_2
  src/ar_detector_node_2.cpp
)
ament_target_dependencies(ar_detector_node_2 ...)
install(TARGETS ar_detector_node_2
  DESTINATION lib/${PROJECT_NAME}
)

# 変更後
add_library(ar_detector_node_2 SHARED
  src/ar_detector_node_2.cpp
)
ament_target_dependencies(ar_detector_node_2
  rclcpp rclcpp_components OpenCV cv_bridge sensor_msgs geometry_msgs
  image_transport tf2 tf2_ros tf2_geometry_msgs
)
rclcpp_components_register_node(ar_detector_node_2
  PLUGIN "ArDetectorNode2"
  EXECUTABLE ar_detector_node_2_exe   # スタンドアロン実行バイナリも自動生成
)
install(TARGETS ar_detector_node_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

`rclcpp_components_register_node` の `EXECUTABLE` 引数により、
`ros2 run` でのデバッグ単独起動用バイナリ（`ar_detector_node_2_exe`）も自動生成される。

---

### 3. `launch/tello_system_dev1.launch.py`

**変更 1: import 追加**
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
```

**変更 2: 画像パイプラインを ComposableNodeContainer にまとめる**

既存の `usb_cam = Node(...)` 定義と `tello_node_list` 内の
`ar_detector_node_2`・`cmd_vel_visualizer_node` の Node エントリを削除し、
以下のコンテナ定義に置き換える。

```python
image_pipeline_container = ComposableNodeContainer(
    name='image_pipeline_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name='usb_cam_node',
            parameters=[{
                'video_device': '/dev/video_tello',
                'image_width': usb_cam_pixels['width'],
                'image_height': usb_cam_pixels['height'],
                'camera_frame_id': 'camera_frame',
                'camera_info_url': f'file://{calibration_file_path}',
                'pixel_format': 'yuyv',
                'framerate': 25.0,
            }],
            remappings=[('/image_raw', '/camera/image_raw')],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='tello_pilot',
            plugin='ArDetectorNode2',
            name='ar_detector_node_2',
            remappings=[('/cam_image_raw', '/camera/image_raw')],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='tello_pilot',
            plugin='CmdVelVisualizerNode',
            name='cmd_vel_visualizer',
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
    output='screen',
)
```

**変更 3: LaunchDescription の更新**
```python
return LaunchDescription(
    [image_pipeline_container]   # usb_cam Node は削除
    + [joy_node]
    + [camera_frame_node, drone_frame_node]
    + tello_node_list            # ar_detector_node_2, cmd_vel_visualizer_node を除いたリスト
    + [rviz]
)
```

---

## 実装手順

1. `ar_detector_node_2.cpp`・`cmd_vel_visualizer_node.cpp` を修正
2. `CMakeLists.txt` を修正
3. ビルドして `ar_detector_node_2_exe` などのバイナリが生成されることを確認
4. launch ファイルを修正
5. 起動して `ros2 component list` でコンテナ内のコンポーネントを確認
6. IPC 有効化の確認（ログに `"intra-process enabled"` が出るか、遅延のビフォーアフター比較）

---

## 代替案: 部分 IPC（usb_cam を別プロセスに残す）

usb_cam の `respawn=True` を維持したい場合、usb_cam だけ Node 形式のままにして
ar_detector_node_2 と cmd_vel_visualizer_node のみをコンテナ化する構成でもよい。

- usb_cam → コンテナ間: DDS 通信のまま（シリアライズあり）
- コンテナ内の `/image_ar` など: IPC（シリアライズなし）
- usb_cam の respawn が有効

IPC の効果測定が主目的であれば、まずこの部分 IPC 構成から始めることを推奨する。

---

## 既知のリスク

| リスク | 対策 |
|--------|------|
| usb_cam が `unique_ptr` でなく `const ref` で publish している場合、コピーが発生 | 遅延測定で効果を評価し、効果が薄ければ部分 IPC 構成に戻す |
| usb_cam の `respawn` が ComposableNode で使えない | 部分 IPC 構成にフォールバック |
| 2サブスクライバーで完全 zero-copy にならない | 研究記録に明記。DDS シリアライズ排除の効果は得られる |
