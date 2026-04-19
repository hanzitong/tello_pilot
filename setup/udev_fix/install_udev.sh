#!/bin/bash
# install_udev.sh
#
# Tello 地上カメラの udev ルールをセットアップするスクリプト。
# カメラを接続した状態で実行してください。
#
# 使い方:
#   cd ws_tello
#   bash setup/install_udev.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/udev/99-tello-camera.rules"
DEST="/etc/udev/rules.d/99-tello-camera.rules"
SYMLINK_NAME="video_tello"  # udev が作成するシンボリックリンク名


# ---- 接続中のカメラ一覧を表示 ----
echo "========================================"
echo "  接続中の Video デバイス"
echo "========================================"
if command -v v4l2-ctl &>/dev/null; then
    v4l2-ctl --list-devices 2>/dev/null
else
    ls /dev/video* 2>/dev/null || echo "  (カメラが見つかりません)"
fi
echo ""


# ---- ユーザーにデバイス番号を入力させる ----
echo "地上カメラのデバイス番号を入力してください（例: 4  →  /dev/video4 ）"
read -r -p "番号: " VIDNUM
DEVICE="/dev/video${VIDNUM}"

if [[ ! -e "$DEVICE" ]]; then
    echo "エラー: $DEVICE が存在しません。番号を確認してください。"
    exit 1
fi


# ---- USB ID を取得 ----
VENDOR=$(udevadm info --query=all --name="$DEVICE" | grep "ID_VENDOR_ID" | cut -d= -f2)
PRODUCT=$(udevadm info --query=all --name="$DEVICE" | grep "ID_MODEL_ID" | cut -d= -f2)

if [[ -z "$VENDOR" || -z "$PRODUCT" ]]; then
    echo "エラー: $DEVICE の USB ID を取得できませんでした。"
    echo "  内蔵カメラや一部のデバイスは USB ID を持たないため対応できません。"
    exit 1
fi

echo ""
echo "検出された USB ID:  ${VENDOR}:${PRODUCT}"
echo "デバイス:           $DEVICE"
echo "インストール先:     $DEST"
echo ""


# ---- udev ルールを生成してインストール ----
sed "s/VENDOR_ID/${VENDOR}/g; s/PRODUCT_ID/${PRODUCT}/g" "$TEMPLATE" \
    | sudo tee "$DEST" > /dev/null

echo "udev ルールをインストールしました。"


# ---- udev を再ロード ----
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "udev を再ロードしました。"
echo ""


# ---- 結果確認 ----
echo "========================================"
echo "  確認"
echo "========================================"
sleep 1  # trigger が反映されるまで少し待つ

if [[ -L /dev/${SYMLINK_NAME} ]]; then
    RESOLVED=$(realpath /dev/${SYMLINK_NAME})
    echo "  /dev/${SYMLINK_NAME} -> $RESOLVED  ✓"
    echo "  launch ファイルは起動時に /dev/${SYMLINK_NAME} を自動解決します  ✓"
else
    echo "  /dev/${SYMLINK_NAME} がまだ作成されていません。"
    echo "  カメラを一度抜いて再接続してください。"
fi

echo ""
echo "セットアップ完了。"
