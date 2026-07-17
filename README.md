# SWARM_ROBOT — Ground Control (ArUco + ROS2/Gazebo + Zenoh + Firebase)

Sistem simulasi robot beroda (leader) yang dipandu kamera overhead (deteksi
ArUco marker) untuk bergerak otomatis menuju titik target, dan dapat dipantau
serta dikendalikan (otomatis maupun manual) dari dashboard web yang
terhubung real-time lewat Firebase Firestore.

## 1. Ringkasan Arsitektur

![Blok Diagram Sistem Terintegrasi](docs/blok_diagram_sistem_terintegrasi.png)

### 🎥 Video Demo Simulasi

<!-- 
  CARA MENEMPEL VIDEO DI SINI (supaya video ikut terputar langsung
  di halaman README, bukan cuma jadi link biasa):
  1. Buka repo ini di github.com, klik file README.md, klik ikon
     pensil (Edit this file).
  2. Cari baris tulisan "GANTI BARIS INI..." tepat di bawah komentar ini.
  3. Hapus baris itu, lalu drag & drop file video Anda (docs/videos/
     demo_simulasi.webm) langsung ke kotak editor, PERSIS di posisi
     baris yang tadi dihapus.
  4. Tunggu sampai GitHub selesai meng-upload (progress bar), akan
     muncul otomatis satu baris link (https://github.com/user-
     attachments/assets/....) menggantikan posisi tadi.
  5. Scroll ke bawah, klik "Commit changes".
-->
[demo_simulasi.webm](https://github.com/user-attachments/assets/73114f3f-b063-4bef-b085-e219feb73d70)



```
┌─────────────────┐   ArUco (kamera overhead)   ┌──────────────────┐
│  Gazebo (robot   │ ───────────────────────────▶│  aruco_detector   │
│  + arena + target)│                             │  (posisi & arah)  │
└─────────────────┘                              └────────┬─────────┘
                                                            │ /aruco/leader_pose
                                                            │ /aruco/target_pose
                                                            ▼
                                                   ┌──────────────────┐
                                                   │  target_follower  │
                                                   │  (kontrol gerak)  │
                                                   └────────┬─────────┘
                                                            │ /cmd_vel
                                                            ▼
                                                        Gazebo (fisik)

     Robot PC (Zenoh client)  ⇄  VPS AWS EC2 (Zenoh router)  ⇄  zenoh_to_firebase.py
                                                                        │
                                                                        ▼
                                                            Firebase Firestore (real-time)
                                                                        │
                                                                        ▼
                                                          Dashboard Web (browser)
```

**Komponen inti:**
- **`aruco_detector`** (C++/ROS2): membaca gambar kamera overhead di Gazebo,
  mendeteksi marker ArUco (4 sudut arena untuk kalibrasi homography, marker
  LEADER, marker TARGET), dan mempublish posisi (x, y dalam cm) + arah
  hadap (theta) robot secara real-time.
- **`target_follower`** (C++/ROS2): node kontrol gerak. Menghitung jarak &
  sudut ke target, lalu menjalankan strategi **dua-tahap**: robot berputar
  di tempat sampai benar-benar menghadap target, baru maju (dengan koreksi
  arah kecil selama berjalan). Mendukung mode Otomatis (START dari GUI) dan
  Manual (tombol panah).
- **`zenoh_to_firebase.py`** (Python, jalan di VPS AWS EC2): jembatan dua
  arah antara topic ROS2 (lewat Zenoh) dan Firebase Firestore — data sensor
  naik ke Firestore, perintah dari dashboard turun ke robot.
- **`dashboard_v3.html`**: antarmuka web (HTML/JS + Firebase SDK) untuk
  memantau posisi robot/target secara live dan mengirim perintah
  (Kalibrasi, Start, Stop, kontrol manual).

## 2. Struktur Folder

```
robot_ws/src/swarm_robot/
├── src/
│   ├── aruco_detector.cpp   — deteksi ArUco & homography kamera overhead
│   ├── target_follower.cpp  — node kontrol gerak AKTIF (dipakai sekarang)
│   └── dijkstra_node.cpp    — node lama berbasis grid Dijkstra (arsip,
│                               tidak dipakai karena arena tanpa rintangan)
├── urdf/robot.urdf          — model fisik robot
├── worlds/my_world.sdf      — dunia simulasi Gazebo (arena, dinding,
│                               marker sudut, marker target)
└── launch/simulation.launch.py

zenoh_firebase_bridge/
├── vps/zenoh_to_firebase.py         — jalan di server AWS EC2
└── robot_pc/configs/zenoh-client.json5 — config Zenoh di komputer robot

web_dashboard/dashboard_v3.html      — dashboard, dibuka langsung di browser
```

## 3. Prasyarat

- Ubuntu 24.04 dengan ROS2 **Jazzy** dan **Gazebo (gz sim)** terpasang
- Paket ROS2: `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`,
  `joint_state_publisher`, `tf2_ros`, OpenCV (untuk `aruco_detector`)
- 1 unit VPS (contoh: AWS EC2, Ubuntu 24.04) dengan IP publik & port **7447**
  terbuka (untuk Zenoh router)
- Akun **Firebase** dengan Firestore diaktifkan, plus file kredensial
  *service account* (JSON)
- `zenoh-bridge-ros2dds` terpasang di komputer robot maupun di VPS
- Python 3 + paket: `zenoh`, `firebase-admin`, `rosbags`, `python-dotenv`

## 4. Langkah Instalasi & Menjalankan Simulasi (dari nol)

### 4.1 Setup VPS (AWS EC2)
1. Buat instance EC2 Ubuntu 24.04, buka port **7447** (TCP) di Security
   Group untuk Zenoh, dan catat **IP publik**-nya (perhatikan: IP publik EC2
   bisa **berubah** setiap kali instance di-restart, kecuali memakai
   Elastic IP — cek ulang IP-nya tiap sesi baru).
2. Pasang `zenoh-bridge-ros2dds` dan jalankan sebagai **router**:
   ```bash
   zenoh-bridge-ros2dds --config configs/zenoh-router.json5
   ```
3. Salin `zenoh_firebase_bridge/vps/zenoh_to_firebase.py` ke VPS, taruh di
   `/opt/zenoh_firebase_bridge/vps/zenoh_to_firebase.py`, siapkan file
   kredensial Firebase (`serviceAccountKey.json`), lalu jalankan (idealnya
   sebagai systemd service `zenoh-firebase-bridge.service`) supaya otomatis
   restart kalau VPS reboot.

### 4.2 Setup Firebase
1. Buat project Firebase baru → aktifkan **Firestore Database** (mode
   production/test sesuai kebutuhan).
2. Buat **Service Account** (Project Settings → Service Accounts →
   Generate new private key) → simpan JSON-nya di VPS, atur environment
   variable `FIREBASE_CRED_PATH` mengarah ke file itu.
3. Ambil **Firebase config** (apiKey, projectId, dst.) untuk ditempel di
   `web_dashboard/dashboard_v3.html` bagian inisialisasi Firebase SDK.

### 4.3 Setup Workspace ROS2 (komputer robot)
```bash
mkdir -p ~/robot_ws/src
cp -r robot_ws/src/swarm_robot ~/robot_ws/src/
cd ~/robot_ws
colcon build --packages-select swarm_robot
source install/setup.bash
```

### 4.4 Setup Zenoh Client (komputer robot)
```bash
cp -r zenoh_firebase_bridge/robot_pc ~/Downloads/zenoh_firebase_bridge/robot_pc
# edit configs/zenoh-client.json5, ganti IP_PUBLIK_VPS_ANDA dengan IP EC2 Anda
```

### 4.5 Menjalankan Simulasi
Buka **4 terminal terpisah** di komputer robot:

```bash
# Terminal 1 — Simulator Gazebo
source ~/robot_ws/install/setup.bash
gz sim ~/robot_ws/src/swarm_robot/worlds/my_world.sdf

# Terminal 2 — Node ROS2 (robot + aruco_detector + target_follower)
source ~/robot_ws/install/setup.bash
ros2 launch swarm_robot simulation.launch.py

# Terminal 3 — Uploader gambar kamera overhead
source ~/robot_ws/install/setup.bash
ros2 run swarm_robot camera_uploader.py

# Terminal 4 — Jembatan Zenoh (komputer robot ⇄ VPS)
cd ~/Downloads/zenoh_firebase_bridge/robot_pc
zenoh-bridge-ros2dds --config configs/zenoh-client.json5
```

Lalu buka `web_dashboard/dashboard_v3.html` langsung di browser (`file://...`),
pastikan status **"Terhubung ke Firestore (real-time)"** menyala, dan tekan
tombol **START** (mode Otomatis) atau gunakan tombol panah (mode Manual).

## 5. Dokumentasi Lengkap

Untuk penjelasan teori, arsitektur mendetail, seluruh proses debugging (yang
cukup panjang!), analisis, dan kesimpulan, lihat:
- `Laporan_Praktikum_Swarm_Robot.docx`
- `Panduan_Simulasi_Swarm_Robot.docx`

## 6. Ringkasan Masalah yang Pernah Ditemukan & Diperbaiki

| # | Masalah | Akar Penyebab | Perbaikan |
|---|---|---|---|
| 1 | Robot tidak bergerak sama sekali saat START ditekan | `typestore.serialize_cdr()` mengembalikan `memoryview`, sedangkan API Zenoh Python butuh `bytes` — publish gagal setiap saat | Bungkus dengan `bytes(...)` sebelum `publisher.put()` |
| 2 | Robot berputar-putar tak menentu, tak pernah lurus ke target | Kontrol lama (grid Dijkstra) memakai ambang batas & "kunci arah" yang rapuh, plus arah putar robot **terbalik 180°** akibat kesalahan tanda pada offset orientasi marker (`YAW_OFFSET`) dikombinasikan dengan koreksi mirror sumbu-Y koordinat lokal | Ganti ke strategi dua-tahap sederhana (putar dulu → baru maju) di `target_follower.cpp`, dan set `YAW_OFFSET = -M_PI/2` (dibuktikan lewat perhitungan selisih arah gerak nyata vs. arah yang dilaporkan sensor) |
| 3 | Tampilan arena di dashboard tidak sinkron dengan kamera overhead | Rasio kanvas dipaksa landscape padahal arena aslinya portrait, dan query riwayat jejak Firestore memakai `orderBy asc + limit` (selalu ambil data **paling lama**, bukan terbaru) | Render kanvas dengan rasio asli arena (letterbox), ubah query jadi `orderBy desc + limit` lalu `reverse()` |
| 4 | Tombol kontrol Manual tidak berfungsi saat mode Otomatis nonaktif | `target_follower` terus-menerus mempublish perintah "berhenti" ke `/cmd_vel` setiap 50 ms selagi idle, menimpa perintah manual dari dashboard | Kirim perintah "berhenti" hanya sekali (single-shot) saat pertama kali masuk kondisi idle |

## 7. Referensi

- ROS 2 Documentation — <https://docs.ros.org/>
- Gazebo (gz sim) Documentation — <https://gazebosim.org/docs>
- OpenCV ArUco Module — <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>
- Zenoh / zenoh-bridge-ros2dds — <https://zenoh.io/>
- Firebase Firestore Documentation — <https://firebase.google.com/docs/firestore>
- AWS EC2 Documentation — <https://docs.aws.amazon.com/ec2/>
