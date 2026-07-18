#!/bin/bash
# ============================================================================
# connect_vps.sh — VERSI 2 (WebSocket/rosbridge)
# ----------------------------------------------------------------------------
# Dijalankan di KOMPUTER ROBOT (bukan di VPS). Script ini:
#   1. Menjalankan rosbridge_websocket (port 9090) — membuka SEMUA topic ROS2
#      lewat WebSocket, tanpa perlu skrip penerjemah custom seperti di V1.
#   2. Menjalankan web_video_server (port 8080) — streaming video kamera
#      overhead secara langsung (MJPEG) ke browser.
#   3. Membuka reverse SSH tunnel ke VPS, supaya kedua port di atas bisa
#      diakses lewat alamat IP publik VPS oleh dashboard_v2.html dari
#      mana saja di internet.
#
# CATATAN: file ini adalah TEMPLATE. Salin ke connect_vps.sh (tanpa .example)
# dan sesuaikan variabel di bawah dengan kredensial Anda. File connect_vps.sh
# hasil salinan sudah diabaikan oleh .gitignore supaya kredensial tidak
# ter-upload ke GitHub.
# ============================================================================

set -e

# --- GANTI BAGIAN INI SESUAI VPS ANDA ---
VPS_USER="ubuntu"
VPS_IP="IP_PUBLIK_VPS_ANDA"          # sama seperti IP yang dipakai di V1
VPS_KEY="$HOME/Downloads/selvia.pem"  # path file .pem Anda
# -----------------------------------------

echo "[1/3] Menjalankan rosbridge_websocket (port 9090)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

sleep 2

echo "[2/3] Menjalankan web_video_server (port 8080)..."
ros2 run web_video_server web_video_server &
WEBVIDEO_PID=$!

sleep 2

echo "[3/3] Membuka reverse SSH tunnel ke VPS ${VPS_IP}..."
echo "      (port 9090 & 8080 di VPS akan meneruskan ke komputer robot ini)"
ssh -N \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -R 9090:localhost:9090 \
  -R 8080:localhost:8080 \
  -i "${VPS_KEY}" \
  "${VPS_USER}@${VPS_IP}"

# Jika SSH terputus (misal koneksi internet sempat hilang), matikan juga
# rosbridge & web_video_server yang tadi dijalankan di background.
kill "${ROSBRIDGE_PID}" "${WEBVIDEO_PID}" 2>/dev/null || true
