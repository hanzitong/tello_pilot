# SingleThreadedExecutor におけるデッドロック

## SingleThreadedExecutor の動作原理

`SingleThreadedExecutor` は1本のスレッドでコールバックキューを逐次処理する。

```
キュー: [callbackA, callbackB, callbackC, ...]
          ↑ 実行中（完了するまで次には進まない）
```

このスレッドが何らかの理由でブロックされると、キュー内の後続コールバックはすべて待機し続ける。

---

## デッドロックが発生する条件

コールバック内で「**同じ Executor のスレッドが処理しないと完了しないもの**」を同期的に待つと、デッドロックが発生する。

### 具体例: サービス応答の同期待ち

```cpp
void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto request = std::make_shared<MySrv::Request>();
    auto future = service_client_->async_send_request(request);

    // NG: spin_until_future_complete は Executor のスレッドがスピンして
    //     レスポンスを処理する必要がある。しかしそのスレッドは今ここで
    //     ブロックされているため、レスポンスが永遠に処理されない。
    rclcpp::spin_until_future_complete(this->get_shared_ptr(), future);
}
```

デッドロックの連鎖を図示する（矢印はブロックの依存関係を示す）:

```
Executor スレッド
  └─ imageCallback 実行中
       └─ spin_until_future_complete で待機
            └─（レスポンスを処理するには Executor のスレッドが必要）
                 └─ そのスレッドは待機中
                      └─ → 永遠に解決しない（デッドロック）
```

### デッドロックになる処理の一覧

| 処理 | 理由 |
|------|------|
| `rclcpp::spin_until_future_complete(...)` | Executor の再入が必要 |
| サービス応答の future を `.get()` で同期待ち | 同上 |
| アクションのゴール送信・結果受信の同期待ち | 同上 |

---

## デッドロックにならない処理

Executor を必要としない処理はコールバック内で同期的に行っても問題ない。ただし処理時間が長いと後続コールバックの遅延（レイテンシ増加）につながる。

| 処理 | 影響 |
|------|------|
| 画像処理・行列演算などの純粋な計算 | 後続コールバックの遅延のみ |
| TF の broadcast | 問題なし |
| ファイル I/O・スリープ | 後続コールバックの遅延のみ |
| サービス応答の非同期コールバック受け取り | 問題なし（後述） |

---

## 解決策

### 1. async + コールバックパターン（推奨）

サービス応答をコールバックで受け取ることで、待機を発生させない。

```cpp
void imageCallback(...)
{
    auto request = std::make_shared<MySrv::Request>();

    // レスポンスが届いたとき Executor がこのラムダをコールバックとして実行する。
    // imageCallback 自体はここで終了し、Executor のスレッドをブロックしない。
    service_client_->async_send_request(request,
        [this](rclcpp::Client<MySrv>::SharedFuture future) {
            auto response = future.get();
            // レスポンスをここで処理する
        });
}
```

### 2. MultiThreadedExecutor を使う

複数スレッドでコールバックを並列実行するため、1つのコールバックがブロックしても他のコールバックは進める。ただし共有データへのアクセスに mutex が必要になる。

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<MyNode>());
    executor.spin();
    rclcpp::shutdown();
}
```

### 3. ノードを分割し、別 Executor で実行する

ブロッキングが避けられない処理は専用ノードに切り出し、独立した Executor（別スレッド）で動かす。

---

## このプロジェクトにおける現状

| ノード | サービス呼び出し | デッドロックリスク |
|--------|----------------|-----------------|
| `ar_single_detector_node` | なし | なし |
| `ar_multi_detector_node` | なし | なし |
| `pid_controller_node` | なし | なし |
| `cmd_multiplexer_node` | `/tello_action`（呼び出し側） | 実装次第で**あり** |
| `auto_lander_node` | `/tello_action`（呼び出し側） | 実装次第で**あり** |

`cmd_multiplexer_node` と `auto_lander_node` は `/tello_action` サービスを呼び出している。これらがサービス応答を同期的に待っている場合、`SingleThreadedExecutor` 上でデッドロックが発生する可能性があるため、実装を確認すること。

---

## まとめ

`SingleThreadedExecutor` においてコールバック内で禁止される操作は、「**Executor のスレッドを再入しようとする待機**」である。純粋な計算処理や TF broadcast はどれだけ重くても禁止ではない（後続コールバックの遅延という別問題は生じる）。サービス・アクションの応答待ちは必ず非同期コールバックパターンで実装すること。
