# SWARM_ROBOT — Ground Control

Sistem simulasi robot beroda (leader) yang dipandu kamera overhead (deteksi
ArUco marker) untuk bergerak otomatis menuju titik target, dapat dipantau
dan dikendalikan (otomatis maupun manual) dari dashboard web.

Repository ini berisi **dua versi arsitektur komunikasi** yang bisa dipilih.
Logika robot (deteksi ArUco, navigasi otomatis ke target) **sama persis** di
kedua versi — yang berbeda hanya cara robot terhubung ke dashboard web.

## 📦 Versi 1 — Zenoh + Firebase Firestore

➡️ Lihat [`v1_zenoh_firebase/README.md`](v1_zenoh_firebase/README.md)

Robot ⇄ Zenoh (client/router) ⇄ VPS ⇄ skrip jembatan Python ⇄ Firebase
Firestore ⇄ Dashboard Web. Sudah teruji stabil, mendukung riwayat data dan
lapisan keamanan Firestore.

## 📦 Versi 2 — WebSocket / rosbridge (tanpa Firebase)

➡️ Lihat [`v2_websocket_rosbridge/README_V2.md`](v2_websocket_rosbridge/README_V2.md)

Robot ⇄ rosbridge_websocket + web_video_server ⇄ reverse SSH tunnel ⇄ VPS
(sama dengan V1) ⇄ Dashboard Web langsung lewat `roslibjs`. Lebih sederhana
(tidak perlu skrip jembatan maupun basis data perantara), dengan tambahan
live video streaming — namun tanpa riwayat data dan tanpa lapisan keamanan
tambahan.

## 🎥 Video Demo Simulasi

<!--
  CARA MENEMPEL VIDEO DI SINI (supaya video ikut terputar langsung di README):
  Edit file ini di github.com, drag & drop file video (di bawah 10MB) ke
  posisi ini, tunggu upload selesai, lalu Commit changes.
-->

## 📄 Dokumentasi Lengkap

- [Laporan Praktikum](Laporan_Praktikum_Swarm_Robot.docx)
- [Panduan Simulasi V1](Panduan_Simulasi_Swarm_Robot.docx)
