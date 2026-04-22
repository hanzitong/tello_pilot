# ar_multi_detector_node.cpp コード解読メモ

作成日: 2026-04-22

---

## 概要

このドキュメントは `src/ar_multi_detector_node.cpp` の処理内容を、C++ 文法の説明を含めて記述したものである。

---

## ファイル全体の構造

```
（グローバル）
  定数定義 (kMarkerLength, kMarkerDist)
  構造体定義 (MarkerDef)
  配列定義 (kMarkerDefs[])

（クラス）
  class ArMultiDetectorNode : public rclcpp::Node
    コンストラクタ
    メンバ変数
    cameraInfoCallback()
    imageCallback()

main()
```

---

## グローバル定数・構造体

### `static constexpr double`

```cpp
static constexpr double kMarkerLength = 0.03;
static constexpr double kMarkerDist   = 0.06;
```

`constexpr` は「コンパイル時定数」を意味する指定子である。
コンパイラはこの値を実行前に確定させるため、通常の `const` 変数よりも最適化が利きやすい。
`static` はこの変数のリンケージをファイル内に限定する。
`k` プレフィックスは定数であることを示す命名規則である。

### `struct MarkerDef`

```cpp
struct MarkerDef {
    int    id;
    double nx, ny;
};
```

`struct` は複数のデータをひとまとめにする型定義である。
C++ の `struct` は `class` とほぼ同等であり、デフォルトのアクセス修飾子が `public` である点だけが異なる。
`MarkerDef` は「マーカーID」と「drone_frame 上の取り付け方向を表す単位符号 (nx, ny)」をひとつの型として表現するために定義している。

### `static constexpr MarkerDef kMarkerDefs[]`

```cpp
static constexpr MarkerDef kMarkerDefs[] = {
    { 9,  1,  1},
    {20, -1,  1},
    {21, -1, -1},
    {26,  1, -1},
};
```

`MarkerDef` の配列を集成体初期化（aggregate initialization）で定義している。
集成体初期化とは、`{}` による初期化子リストをメンバの宣言順に対応付ける初期化方法である。
`{ 9, 1, 1}` は `id=9, nx=1, ny=1` を意味する。
配列のサイズは初期化子リストの要素数からコンパイラが自動的に推論する。

この配列は各マーカーが drone_frame 上のどの方向に取り付けられているかを定義する。
実際の取り付け距離は後述のコンストラクタで `kMarkerDist / √2` を乗算して計算する。

---

## クラス定義

### 継承

```cpp
class ArMultiDetectorNode : public rclcpp::Node
```

`ArMultiDetectorNode` クラスは `rclcpp::Node` を public 継承している。
`rclcpp::Node` は ROS 2 のノードの基底クラスであり、トピックの購読・配信、パラメータ、ロガーなどの機能を提供する。
public 継承により、`rclcpp::Node` の public メンバがそのまま `ArMultiDetectorNode` の public メンバとして引き継がれる。

---

## コンストラクタ

```cpp
ArMultiDetectorNode() : Node("ar_multi_detector")
```

`: Node("ar_multi_detector")` はメンバ初期化子リストである。
コンストラクタ本体が実行される前に基底クラス `rclcpp::Node` のコンストラクタを呼び出し、ノード名を `"ar_multi_detector"` として ROS 2 に登録する。

### `std::make_unique`

```cpp
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
```

`std::make_unique<T>(args...)` は `T` 型のオブジェクトをヒープ上に生成し、それを管理する `std::unique_ptr<T>` を返す関数テンプレートである。
`unique_ptr` はオブジェクトの所有権をひとつのポインタが独占的に持つスマートポインタであり、スコープを外れると自動的にオブジェクトを破棄する（RAII）。
`this` を渡すことで、`TransformBroadcaster` に自ノードへの参照を与え、ノードのクロックやロガーを使って TF を送信できるようにする。

### `create_subscription`

```cpp
image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/cam_image_raw",
    rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
    std::bind(&ArMultiDetectorNode::imageCallback, this, std::placeholders::_1)
);
```

`create_subscription<T>(topic, qos, callback)` はトピックを購読するサブスクリプションを生成するメンバ関数である。

**QoS について:**
`rclcpp::SensorDataQoS(rclcpp::KeepLast(1))` はセンサデータ向けの QoS 設定であり、`KeepLast(1)` はキューに最新の 1 件のみを保持することを意味する。
これにより、処理が間に合わなかった古いフレームがキューに蓄積されず、常に最新の画像が処理される。

**`std::bind` について:**
`std::bind(&ArMultiDetectorNode::imageCallback, this, std::placeholders::_1)` はメンバ関数をコールバックとして渡すための書き方である。
`&ArMultiDetectorNode::imageCallback` はメンバ関数ポインタであり、単体では呼び出せない（どのオブジェクトに対して呼ぶかが不明なため）。
`std::bind` は第 2 引数 `this` をオブジェクトとして束縛し、`std::placeholders::_1` の位置に呼び出し時の第 1 引数（メッセージのポインタ）を渡すように設定する。
これにより、`rclcpp` がメッセージを受信したとき `this->imageCallback(msg)` と等価な呼び出しが行われる。

### マーカーオフセットマップの構築

```cpp
const double d = kMarkerDist / std::sqrt(2.0);
for (const auto& def : kMarkerDefs) {
    marker_offset_[def.id] = tf2::Vector3(def.nx * d, def.ny * d, 0.0);
}
```

各マーカーが drone_frame 上の 45°/135°/225°/315° 方向に置かれているため、X 成分と Y 成分はそれぞれ `kMarkerDist / √2` になる。
範囲 for 文 `for (const auto& def : kMarkerDefs)` は配列の全要素を順に参照する。
`const auto&` は「型をコンパイラに推論させ、コピーせず参照で受け取る」書き方である。
`std::map::operator[]` はキーが存在しない場合に新しいエントリを生成して代入する。
この処理の結果、`marker_offset_[9]` は `{d, d, 0.0}`、`marker_offset_[20]` は `{-d, d, 0.0}` のようにマップが構築される。

---

## メンバ変数

```cpp
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         image_pub_;
std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
```

`rclcpp::Subscription<T>::SharedPtr` と `rclcpp::Publisher<T>::SharedPtr` は、それぞれ `shared_ptr` でラップされた購読・配信オブジェクトである。
`shared_ptr` はオブジェクトの所有権を複数のポインタで共有するスマートポインタであり、参照カウントがゼロになると自動的に破棄される。
ノードが生きている間これらのオブジェクトが有効であり続けるよう、メンバ変数として保持する。

```cpp
std::map<int, tf2::Vector3> marker_offset_;
```

`std::map<Key, Value>` は「キー → 値」の連想配列であり、内部は赤黒木で実装されている。
ここでは「マーカー ID (int)」を「drone_frame 上のオフセット位置 (tf2::Vector3)」に対応付けるために使用している。

---

## `cameraInfoCallback`

```cpp
void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
```

このコールバックは `/camera_info` トピックにメッセージが届くたびに呼ばれる。
`camera_matrix_ready_` が `true` のときは即座に return する。
これにより、カメラ行列の初期化は一度だけ実行される。

### カメラ行列の取得

```cpp
camera_matrix_ = cv::Mat(3, 3, CV_64F);
for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
        camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];
```

`sensor_msgs::msg::CameraInfo` の `k` フィールドは、カメラの内部パラメータ行列 K を行優先の 1 次元配列（長さ 9）で保持している。
行列の要素 (i, j) は `k[i * 3 + j]` で取得できる。
`cv::Mat::at<double>(i, j)` は行列の (i, j) 要素への参照を返す。

```cpp
const int n = std::min(static_cast<int>(msg->d.size()), 5);
dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
for (int i = 0; i < n; ++i)
    dist_coeffs_.at<double>(i) = msg->d[i];
```

`msg->d` は歪み係数のベクトルであり、キャリブレーションモデルによって要素数が異なる。
最大 5 係数（plumb_bob モデル）を使用するため、`std::min` で上限を 5 に制限する。
`static_cast<int>` は `msg->d.size()` の型（`size_t`、符号なし整数）を `int`（符号あり整数）に明示的に変換している。
`std::min` は同じ型の引数を要求するため、この型変換が必要である。

---

## `imageCallback`

このコールバックは `/cam_image_raw` にフレームが届くたびに呼ばれ、以下の処理を順に実行する。

### 1. 画像のコピーと変換

```cpp
auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
cv::Mat& img = cv_ptr->image;
```

`cv_bridge::toCvCopy` は ROS 2 の画像メッセージ (`sensor_msgs/Image`) を OpenCV の `cv::Mat` に変換する。
`"bgr8"` は変換後のピクセルフォーマットを指定している。
`cv_ptr` の型は `cv_bridge::CvImagePtr`（実体は `shared_ptr<cv_bridge::CvImage>`）であり、`auto` によって型推論される。
`cv::Mat& img = cv_ptr->image` は `cv_ptr` が持つ `cv::Mat` への参照を取得している。
参照 (`&`) を使うことで、`img` に対する描画操作が `cv_ptr->image` に直接反映される。

### 2. マーカー検出

```cpp
std::vector<int> ids;
std::vector<std::vector<cv::Point2f>> corners;
cv::aruco::detectMarkers(img, dictionary_, corners, ids, parameters_);
```

`cv::aruco::detectMarkers` は画像中の ArUco マーカーを検出し、結果を `ids`（検出されたマーカーの ID リスト）と `corners`（各マーカーの 4 頂点座標のリスト）に格納する。
`std::vector<std::vector<cv::Point2f>>` は「`cv::Point2f` のベクタ」のベクタであり、マーカーごとに 4 頂点（各頂点は 2D 座標）を格納する構造になっている。

### 3. ポーズ推定

```cpp
std::vector<cv::Vec3d> rvecs, tvecs;
if (!ids.empty()) {
    cv::aruco::estimatePoseSingleMarkers(
        corners, kMarkerLength, camera_matrix_, dist_coeffs_, rvecs, tvecs);
}
```

`ids.empty()` は `ids` ベクタが空かどうかを判定するメンバ関数である。
`cv::aruco::estimatePoseSingleMarkers` は各マーカーの 3D ポーズをカメラ座標系で推定し、`rvecs`（回転ベクトル）と `tvecs`（並進ベクトル）に格納する。
`cv::Vec3d` は 3 次元の `double` 型ベクトルを表す OpenCV の型である。
`rvecs[i]` と `tvecs[i]` が `corners[i]`（`ids[i]` のマーカー）に対応するポーズである。

### 4. Rx(π) 行列の定義

```cpp
static const tf2::Matrix3x3 Rx180(1, 0, 0,  0, -1, 0,  0, 0, -1);
```

`static` ローカル変数は関数が複数回呼ばれても一度しか初期化されない。
`Rx180` は X 軸まわりの 180° 回転行列であり、Y と Z 成分の符号を反転させる。

```
Rx(π) = | 1  0   0 |
        | 0 -1   0 |
        | 0  0  -1 |
```

marker_frame の Z 軸はカメラ方向（下向き）であり、drone_frame の Z 軸はドローン上方向（上向き）である。
この 180° 回転を marker_frame に適用することで drone_frame の向きが得られる。

### 5. 各マーカーの処理ループ

```cpp
for (size_t i = 0; i < ids.size(); ++i) {
    auto it = marker_offset_.find(ids[i]);
    if (it == marker_offset_.end()) continue;
```

`std::map::find(key)` はキーに対応する要素への `iterator` を返す。
キーが存在しない場合は `map::end()`（末尾の番兵イテレータ）を返す。
`it == marker_offset_.end()` が真であれば、検出されたマーカー `ids[i]` は `kMarkerDefs` に登録されていない未知のマーカーであるため、`continue` でそのループを飛ばす。

```cpp
const tf2::Vector3& p = it->second;
```

`iterator` の `->second` は map の「値」へのアクセスである（`->first` がキー）。
`const tf2::Vector3&` で参照を取得することにより、`marker_offset_` 内の値をコピーせずに参照する。

### 6. 回転行列の変換

```cpp
cv::Mat R_cv;
cv::Rodrigues(rvecs[i], R_cv);
const tf2::Matrix3x3 R_cam_marker(
    R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
    ...
);
```

`estimatePoseSingleMarkers` が返す `rvecs[i]` はロドリゲスの回転ベクトルである。
回転ベクトルは「軸 × 角度」をひとつの 3 次元ベクトルで表したものであり、そのまま行列演算には使えない。
`cv::Rodrigues` はロドリゲスベクトルを 3×3 回転行列に変換する関数である。
`tf2::Matrix3x3` は引数を行優先で受け取るコンストラクタを持つ。
`R_cam_marker` は「marker_frame のベクトルを camera_frame のベクトルに変換する回転行列」である。

### 7. ドローン中心の推定

```cpp
const tf2::Matrix3x3 R_cam_drone = R_cam_marker * Rx180;
const tf2::Vector3 tvec(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
pos_sum += tvec - R_cam_drone * p;
```

**数学的な導出:**

TF2 の規約では、変換 `T_A_B` は「B フレームの点を A フレームの座標に変換する」ことを意味する。

`T_drone_marker_i`（drone_frame から見たマーカー i の姿勢）:
- 並進: p_i（drone_frame 上のマーカーオフセット）
- 回転: Rx(π)

この逆変換（マーカーから drone_frame への変換）を `T_cam_marker_i` に連結すると:

```
T_cam_drone = T_cam_marker_i × T_marker_i_drone

T_marker_i_drone の並進 = -Rx(π) × p_i
T_cam_drone の回転      = R_cam_marker × Rx(π)
T_cam_drone の並進      = tvec - (R_cam_marker × Rx(π)) × p_i
                        = tvec - R_cam_drone × p_i
```

`R_cam_drone * p` は、drone_frame で表されたオフセットベクトル `p` を camera_frame のベクトルに変換する行列積である。
`tf2::Matrix3x3 * tf2::Vector3` は Bullet Math ライブラリの演算子オーバーロードにより定義されている。

### 8. 四元数の平均

```cpp
tf2::Quaternion q;
R_cam_drone.getRotation(q);
if (count > 0 && q.dot(q_sum) < 0.0)
    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
q_sum = q_sum + tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
```

`tf2::Matrix3x3::getRotation` は回転行列を四元数に変換してメンバ変数 `q` に格納する。

**同一半球への符号統一:**
四元数 `q` と `-q`（全成分の符号を反転したもの）は数学的に同一の回転を表す。
しかし加算による平均を正しく計算するには、加算する四元数の向きを統一する必要がある。
`q.dot(q_sum)` は 4 次元ベクトルとしての内積であり、内積が負のとき `q` と `q_sum` は「逆向き」（反対の半球）にある。
その場合は `q` の全成分の符号を反転して同じ半球に揃える。

**演算子について:**
`tf2::Quaternion` は `btQuaternion`（Bullet Physics の四元数型）の typedef である。
`btQuaternion` は `operator+` を持つが `operator+=` を持たないため、`q_sum = q_sum + ...` の形式で加算している。

### 9. 個別マーカー TF の broadcast

```cpp
geometry_msgs::msg::TransformStamped ts;
ts.header.stamp    = now;
ts.header.frame_id = "camera_frame";
ts.child_frame_id  = "marker_" + std::to_string(ids[i]) + "_frame";
```

`std::to_string(int)` は整数を文字列に変換する標準関数である。
`std::string::operator+` で文字列を連結している。
例えば `ids[i] = 9` のとき、`child_frame_id` は `"marker_9_frame"` になる。

この TF はデバッグ用であり、Rx(π) を含まない生の marker_frame を broadcast する。

### 10. drone_frame の broadcast

```cpp
const tf2::Vector3 pos = pos_sum / static_cast<double>(count);
tf2::Quaternion q_avg = q_sum;
q_avg.normalize();
```

`pos_sum / static_cast<double>(count)` は `tf2::Vector3`（`btVector3`）の `operator/(btScalar)` を使い、3 成分をそれぞれ `count` で除算して算術平均を求める。
`static_cast<double>(count)` は `int` を `double` に明示変換している。
`q_avg.normalize()` は四元数の全成分の符号統一後に加算した結果（非単位四元数）を単位四元数に正規化する。
これにより加算ベースの平均四元数が得られる。

```cpp
ts.transform.rotation = tf2::toMsg(q_avg);
```

`tf2::toMsg` は `tf2::Quaternion` を `geometry_msgs::msg::Quaternion`（ROS 2 メッセージ型）に変換する関数である。

### 11. 画像の publish

```cpp
image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
```

`cv_bridge::CvImage(header, encoding, mat)` は `cv::Mat` を ROS 2 の画像メッセージに変換するオブジェクトを一時生成する。
`.toImageMsg()` は `sensor_msgs::msg::Image::SharedPtr` を返す。
`*` で `shared_ptr` を間接参照し、`publish` にメッセージ実体を渡している。
`msg->header` をそのまま使うことで、画像のタイムスタンプと `frame_id` が元フレームと一致したまま publish される。

---

## `main` 関数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArMultiDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
```

`rclcpp::init` は ROS 2 の通信基盤を初期化し、コマンドライン引数を解析する。
`std::make_shared<ArMultiDetectorNode>()` はノードオブジェクトをヒープ上に生成し、`shared_ptr` で管理する。
`rclcpp::spin` はノードを渡し、コールバックを処理し続けるイベントループに入る。
Ctrl+C などでプロセスが終了シグナルを受け取ると `spin` から抜けて `rclcpp::shutdown` が実行され、ROS 2 の通信基盤が解放される。

---

## 処理の全体フロー（図）

```
main()
  │
  └─ rclcpp::spin()
        │
        ├─(camera_info 受信)─► cameraInfoCallback()
        │                          camera_matrix_ を初期化する（一度だけ）
        │
        └─(画像フレーム受信)─► imageCallback()
                                   │
                                   ├─ cv_bridge で cv::Mat に変換
                                   ├─ detectMarkers で全マーカーを検出
                                   ├─ estimatePoseSingleMarkers でポーズ推定
                                   │
                                   ├─(各既知マーカーに対して)
                                   │    ├─ Rodrigues でrvec→回転行列変換
                                   │    ├─ R_cam_drone = R_cam_marker * Rx(π) を計算
                                   │    ├─ pos = tvec - R_cam_drone * p_i を累積
                                   │    ├─ 四元数を符号統一して累積
                                   │    └─ 個別マーカーTFをbroadcast
                                   │
                                   ├─(1枚以上検出されたとき)
                                   │    ├─ pos_avg = pos_sum / count
                                   │    ├─ q_avg = normalize(q_sum)
                                   │    └─ drone_frame を camera_frame の子としてbroadcast
                                   │
                                   └─ 可視化画像を /image_ar に publish
```

（矢印はデータの流れ・処理の順序の方向）
