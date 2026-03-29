# デバッグ記録: ネットワーク・ファイアウォール問題

## 症状

`tello_action` サービスが常に `rc=2`（ERROR_NOT_CONNECTED）を返す。
公式アプリからは正常に飛行できる。

---

## 原因

Linux のファイアウォール（UFW）が Tello からの UDP 応答パケットをブロックしていた。

ドライバは "command" を送信してドローンの応答を待つが、
応答が届かないため `state_socket_->receiving()` が永遠に false のまま。

---

## ポート番号の調べ方

**ソースコードから読む。**

`tello_driver_node.cpp` のコンストラクタでデフォルト値付きで宣言されている:

```cpp
this->declare_parameter<std::string>("drone_ip",     "192.168.10.1");
this->declare_parameter<int>("drone_port",    8889);   // ドローンへの送信先
this->declare_parameter<int>("command_port",  38065);  // コマンド応答の受信ポート
this->declare_parameter<int>("data_port",     8890);   // State（テレメトリ）受信ポート
this->declare_parameter<int>("video_port",    11111);  // ビデオストリーム受信ポート
```

ネットワーク問題のデバッグ手順:

1. **ソースコードで使用ポートを確認する**
   - `declare_parameter` や `bind` を grep する
   - `grep -r "declare_parameter\|bind\|port" src/` など

2. **ファイアウォールの状態を確認する**
   ```bash
   sudo ufw status
   ```

3. **必要なポートが開いているか照合する**
   - 送信（OUT）は通常許可されているが、受信（IN）がブロックされやすい
   - UDP は TCP と別に管理されている点に注意

4. **tcpdump でパケットが届いているか確認する**
   ```bash
   sudo tcpdump -i wlo1 udp port 8890
   ```
   ドローンから State パケットが届いていれば表示される

---

## 修正

Tello からの受信ポートを UFW で開放:

```bash
sudo ufw allow in on wlo1 from 192.168.10.1 to any port 38065 proto udp  # コマンド応答
sudo ufw allow in on wlo1 from 192.168.10.1 to any port 8890  proto udp  # State
sudo ufw allow in on wlo1 from 192.168.10.1 to any port 11111 proto udp  # ビデオ
```

---

## 汎用的な教訓

ネットワーク通信の問題は以下の順に切り分ける:

```
1. 相手に届いているか？       → ping
2. ポートが開いているか？     → ufw status / tcpdump
3. ソフトウェアが受信しているか？ → ros2 topic echo / ログ
4. データが正しいか？          → 値の検証
```

ソースコードを読んでポート番号を特定するのは最初のステップとして有効。
`declare_parameter` や `socket.bind` を検索すると見つかりやすい。
