# SWARM_ROBOT — Versi 2 (WebSocket / rosbridge)

Versi ini adalah **arsitektur komunikasi alternatif** dari Versi 1
(Zenoh + Firebase). **Logika robot (deteksi ArUco, navigasi otomatis ke
target) sama persis dengan Versi 1** — yang berbeda **hanya jalur
komunikasinya**: menggantikan Zenoh + skrip jembatan Python + Firebase
Firestore dengan `rosbridge_websocket` + `web_video_server` + reverse SSH
tunnel ke VPS yang sama.

## Kenapa Versi 2 Lebih Sederhana

`rosbridge_websocket` secara otomatis membuka **semua** topic ROS2 lewat
WebSocket tanpa perlu skrip penerjemah custom. Dashboard V2 langsung
membaca/menulis topic ROS2 asli (`/aruco/leader_pose`, `/mission/start`,
`/cmd_vel`, dst.) — **tidak ada lagi** basis data perantara (Firestore),
**tidak ada lagi** skrip Python jembatan yang perlu dijaga jalan sebagai
systemd service.

## Perbandingan Singkat V1 vs V2

| Aspek | V1 (Zenoh + Firebase) | V2 (WebSocket / rosbridge) |
|---|---|---|
| Middleware jaringan | Zenoh (client + router) | rosbridge_websocket + SSH reverse tunnel |
| Basis data perantara | Firebase Firestore | Tidak ada (koneksi langsung) |
| Skrip jembatan custom | Ya (`zenoh_to_firebase.py`) | Tidak perlu |
| Video kamera | Snapshot JPEG berkala | Live streaming MJPEG (`web_video_server`) |
| Riwayat/histori data | Tersimpan di Firestore | Tidak ada, kecuali dibuat sendiri |
| Keamanan akses | Ada lapisan Firestore security rules | Tidak ada (siapa pun yang tahu IP:port bisa akses) |

## Prasyarat Tambahan (di luar yang sudah ada di V1)

Pasang paket ROS2 berikut di **komputer robot**:
```bash
sudo apt install -y \
  ros-jazzy-rosbridge-server \
  ros-jazzy-web-video-server
```

## Setup VPS (menggunakan VPS yang SAMA dengan V1)

Karena V2 tidak lagi memakai Zenoh router maupun skrip Python jembatan,
**service V1 di VPS (`zenoh-router.service`, `zenoh-firebase-bridge.service`)
boleh tetap dibiarkan jalan** (tidak bentrok, beda port) — atau dimatikan
kalau memang ingin fokus pakai V2 saja.

Yang perlu disiapkan di VPS untuk V2:

1. **Buka port tambahan di Security Group EC2**: 9090 (rosbridge) dan 8080
   (web_video_server), TCP, sama seperti waktu membuka port 7447 untuk Zenoh
   di V1.
2. **Izinkan reverse tunnel diakses dari luar** — edit file
   `/etc/ssh/sshd_config` di VPS, pastikan baris berikut ada dan tidak
   dikomentari:
   ```
   GatewayPorts yes
   ```
   Lalu restart service SSH:
   ```bash
   sudo systemctl restart ssh
   ```
   Tanpa pengaturan ini, port yang di-reverse-tunnel hanya bisa diakses dari
   VPS itu sendiri (localhost), bukan dari alamat IP publik VPS.

## Menjalankan Simulasi dengan V2

Simulasi Gazebo dan node ROS2 (`aruco_detector`, `target_follower`) dijalankan
**persis sama seperti V1** (Terminal 1–3 tidak berubah). Yang berbeda hanya
Terminal ke-4:

**Terminal 1 — Gazebo** (sama seperti V1)
**Terminal 2 — Launch file ROS2** (sama seperti V1, `simulation.launch.py`
dari folder `robot_ws` V1 maupun V2 — isinya identik)
**Terminal 3 — Camera uploader** — TIDAK DIPERLUKAN lagi di V2 (video sudah
di-live-stream langsung oleh `web_video_server`, tidak perlu upload snapshot
berkala ke Firestore)

**Terminal 4 — Reverse Tunnel ke VPS (menggantikan Zenoh client V1):**
```bash
cd v2_websocket_rosbridge/vps
cp connect_vps.example.sh connect_vps.sh
chmod +x connect_vps.sh
# edit connect_vps.sh: isi VPS_IP dan VPS_KEY sesuai VPS Anda
./connect_vps.sh
```

## Membuka Dashboard V2

Buka `web_dashboard_v2/dashboard_v2.html` langsung di browser, isi kolom
IP publik VPS Anda, klik **Sambungkan**. Video kamera overhead akan langsung
live-streaming, telemetri (posisi leader/target, status misi) akan update
real-time, dan tombol START/STOP/manual langsung mengirim perintah ke robot
tanpa perantara basis data apa pun.

## Catatan Keamanan

Karena V2 tidak memiliki lapisan keamanan (siapa pun yang tahu IP VPS dan
port 9090/8080 dapat terhubung dan mengirim perintah ke robot Anda), V2
**disarankan hanya untuk keperluan pengujian/demo**, bukan untuk penggunaan
produksi jangka panjang tanpa menambahkan lapisan keamanan tambahan
(misalnya autentikasi pada rosbridge, atau membatasi akses lewat firewall
VPS ke IP tertentu saja).
