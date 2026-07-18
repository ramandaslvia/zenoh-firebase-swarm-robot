#!/usr/bin/env python3
"""
zenoh_to_firebase.py (v2 -- dua arah + kamera)

Bagian 1 (seperti sebelumnya): subscribe topik sensor ROS2 lewat Zenoh,
decode CDR pakai "rosbags" (tanpa perlu ROS2 di EC2), simpan ke Firestore.

Bagian 2 (BARU): topik gambar kamera (/overhead_camera/compressed) di-upload
ke Firebase Storage, alamatnya dicatat di Firestore supaya web bisa
menampilkannya.

Bagian 3 (BARU): mendengarkan koleksi Firestore "commands_inbox" -- setiap
kali web dashboard menambah dokumen perintah baru (kalibrasi/start/stop),
script ini otomatis publish balik ke Zenoh supaya robot menerimanya, seolah
tombol itu ditekan langsung dari GUI PC.

Environment variables (lihat .env.example):
  FIREBASE_CRED_PATH     path ke service account JSON Firebase (wajib)
  FIREBASE_COLLECTION    nama koleksi Firestore tujuan (default: swarm_robot)
  FIREBASE_STORAGE_BUCKET nama bucket Firebase Storage (WAJIB diisi manual,
                          cek di Firebase Console > Storage)
  ZENOH_ENDPOINT         endpoint router Zenoh lokal (default: tcp/127.0.0.1:7447)
"""

import os
import sys
import time
import base64
import signal
import logging
from datetime import datetime, timezone

import zenoh
import firebase_admin
from firebase_admin import credentials, firestore
from rosbags.typesys import Stores, get_typestore
from dotenv import load_dotenv

load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("zenoh_to_firebase")

FIREBASE_CRED_PATH = os.getenv("FIREBASE_CRED_PATH", "serviceAccountKey.json")
FIREBASE_COLLECTION = os.getenv("FIREBASE_COLLECTION", "swarm_robot")
ZENOH_ENDPOINT = os.getenv("ZENOH_ENDPOINT", "tcp/127.0.0.1:7447")
# Batas aman ukuran base64 gambar per dokumen Firestore (limit asli 1 MiB per
# dokumen). Kalau frame kompresi lebih besar dari ini, akan di-skip (bukan
# dikirim setengah-setengah) supaya tidak error saat simpan ke Firestore.
MAX_IMAGE_BASE64_BYTES = 900_000

# --- HEMAT KUOTA GRATIS FIRESTORE (20.000 tulis/hari) ---
# Tanpa batas ini, topik yang update puluhan kali/detik (posisi robot) bisa
# menghabiskan jatah harian hanya dalam beberapa menit. Jadi setiap topik
# dibatasi frekuensi penulisannya ke Firestore, TIDAK setiap pesan Zenoh
# yang masuk otomatis ditulis.
MIN_WRITE_INTERVAL_SEC = 0.5          # dokumen utama tiap topik: maks 2x/detik
MIN_HISTORY_WRITE_INTERVAL_SEC = 1.0  # sub-koleksi history: maks 1x/detik
MIN_CAMERA_WRITE_INTERVAL_SEC = 2.0   # kamera: maks 1x per 2 detik

# ---------------------------------------------------------------------------
# Peta topik sensor ROS2 -> tipe pesan (dikirim ke Firestore sebagai dokumen)
# ---------------------------------------------------------------------------
TOPIC_TYPE_MAP = {
    "aruco/leader_pose":         "geometry_msgs/msg/Pose2D",
    "aruco/target_pose":         "geometry_msgs/msg/Pose2D",
    "dijkstra/current_waypoint": "geometry_msgs/msg/Pose2D",
    "mission/status":            "std_msgs/msg/String",
    "mission/start":             "std_msgs/msg/Bool",
    "mission/calibrate":         "std_msgs/msg/Bool",
    "cmd_vel":                   "geometry_msgs/msg/Twist",
}

# Topik khusus gambar kamera -- ditangani beda (upload ke Storage, bukan Firestore langsung)
CAMERA_TOPIC = "overhead_camera/compressed"
CAMERA_MSG_TYPE = "sensor_msgs/msg/CompressedImage"

typestore = get_typestore(Stores.ROS2_JAZZY)


def msg_to_dict(msg):
    """Ubah dataclass hasil decode rosbags menjadi dict polos agar bisa
    disimpan langsung ke Firestore. Field internal seperti __msgtype__
    dibuang karena Firestore melarang nama field diawali garis bawah ganda."""
    if hasattr(msg, "__dataclass_fields__"):
        return {
            name: msg_to_dict(getattr(msg, name))
            for name in msg.__dataclass_fields__
            if not name.startswith("__")
        }
    if isinstance(msg, (list, tuple)):
        return [msg_to_dict(item) for item in msg]
    return msg


def init_firebase():
    if not os.path.exists(FIREBASE_CRED_PATH):
        log.error(
            "File kredensial Firebase tidak ditemukan di '%s'.", FIREBASE_CRED_PATH
        )
        sys.exit(1)
    cred = credentials.Certificate(FIREBASE_CRED_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    return db


def make_sensor_listener(db, topic_name, msg_type):
    doc_id = topic_name.replace("/", "_")
    state = {"last_doc_write": 0.0, "last_history_write": 0.0}

    def listener(sample):
        try:
            raw = bytes(sample.payload.to_bytes())
            msg = typestore.deserialize_cdr(raw, msg_type)
            data = msg_to_dict(msg)
            data["_topic"] = topic_name
            data["_msg_type"] = msg_type
            data["_received_at"] = datetime.now(timezone.utc).isoformat()

            now = time.monotonic()
            if now - state["last_doc_write"] >= MIN_WRITE_INTERVAL_SEC:
                db.collection(FIREBASE_COLLECTION).document(doc_id).set(data, merge=True)
                state["last_doc_write"] = now
                log.info("%s -> Firebase: %s", topic_name, data)

            if now - state["last_history_write"] >= MIN_HISTORY_WRITE_INTERVAL_SEC:
                db.collection(FIREBASE_COLLECTION).document(doc_id) \
                  .collection("history").add(data)
                state["last_history_write"] = now
        except Exception:
            log.exception("Gagal memproses sample dari topik %s", topic_name)

    return listener


def make_camera_listener(db):
    state = {"last_write": 0.0}

    def listener(sample):
        now = time.monotonic()
        if now - state["last_write"] < MIN_CAMERA_WRITE_INTERVAL_SEC:
            return
        try:
            raw = bytes(sample.payload.to_bytes())
            msg = typestore.deserialize_cdr(raw, CAMERA_MSG_TYPE)
            jpeg_bytes = bytes(msg.data)

            b64 = base64.b64encode(jpeg_bytes).decode("ascii")
            if len(b64) > MAX_IMAGE_BASE64_BYTES:
                log.warning(
                    "Frame kamera %d bytes (base64) melebihi batas aman "
                    "Firestore, dilewati. Perkecil resolusi/kualitas JPEG "
                    "di camera_uploader.py.", len(b64)
                )
                return

            db.collection(FIREBASE_COLLECTION).document("camera_meta").set({
                "image_base64": b64,
                "updated_at": datetime.now(timezone.utc).isoformat(),
                "size_bytes": len(jpeg_bytes),
            }, merge=True)
            state["last_write"] = now

            log.info("Kamera -> Firestore: %d bytes (JPEG asli)", len(jpeg_bytes))
        except Exception:
            log.exception("Gagal memproses frame kamera")

    return listener


def make_command_listener(session):
    """Dengarkan koleksi Firestore 'commands_inbox'. Setiap dokumen baru
    (ditambah dari web dashboard) akan diteruskan sebagai perintah ke robot
    lewat Zenoh, lalu dokumennya dihapus supaya tidak diproses dua kali.

    Mendukung dua kelompok aksi:
      - Misi otomatis: 'calibrate', 'start', 'stop'
      - Gerak manual (mode manual di web): 'move_forward', 'move_backward',
        'move_left', 'move_right', 'move_stop' -- publish langsung ke
        /cmd_vel (geometry_msgs/msg/Twist), sama seperti robot digerakkan
        manual dari teleop."""

    Bool = typestore.types["std_msgs/msg/Bool"]
    Twist = typestore.types["geometry_msgs/msg/Twist"]
    Vector3 = typestore.types["geometry_msgs/msg/Vector3"]

    pub_start = session.declare_publisher("mission/start")
    pub_calibrate = session.declare_publisher("mission/calibrate")
    pub_cmd_vel = session.declare_publisher("cmd_vel")

    MANUAL_LINEAR_SPEED = 0.3   # m/s
    MANUAL_ANGULAR_SPEED = 0.8  # rad/s

    def send_bool(publisher, value: bool):
        msg = Bool(data=value)
        raw = bytes(typestore.serialize_cdr(msg, "std_msgs/msg/Bool"))
        publisher.put(raw)

    def send_twist(linear_x: float, angular_z: float):
        msg = Twist(
            linear=Vector3(x=linear_x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z),
        )
        raw = bytes(typestore.serialize_cdr(msg, "geometry_msgs/msg/Twist"))
        pub_cmd_vel.put(raw)

    def on_snapshot(col_snapshot, changes, read_time):
        for change in changes:
            if change.type.name != "ADDED":
                continue
            doc = change.document
            data = doc.to_dict() or {}
            action = data.get("action")
            log.info("Perintah baru dari web dashboard: %s", action)
            try:
                if action == "calibrate":
                    send_bool(pub_calibrate, True)
                elif action == "start":
                    send_bool(pub_start, True)
                elif action == "stop":
                    send_bool(pub_start, False)
                elif action == "move_forward":
                    send_twist(MANUAL_LINEAR_SPEED, 0.0)
                elif action == "move_backward":
                    send_twist(-MANUAL_LINEAR_SPEED, 0.0)
                elif action == "move_left":
                    send_twist(0.0, MANUAL_ANGULAR_SPEED)
                elif action == "move_right":
                    send_twist(0.0, -MANUAL_ANGULAR_SPEED)
                elif action == "move_stop":
                    send_twist(0.0, 0.0)
                else:
                    log.warning("Aksi tidak dikenal: %s", action)
            except Exception:
                log.exception("Gagal meneruskan perintah '%s' ke robot", action)
            finally:
                # Selalu hapus, supaya perintah lama tidak dobel-eksekusi
                doc.reference.delete()

    return on_snapshot


def main():
    db = init_firebase()
    log.info("Firebase siap, koleksi tujuan: %s", FIREBASE_COLLECTION)

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ENDPOINT}"]')
    session = zenoh.open(conf)
    log.info("Terhubung ke Zenoh router di %s", ZENOH_ENDPOINT)

    subs = []
    for topic, msg_type in TOPIC_TYPE_MAP.items():
        sub = session.declare_subscriber(topic, make_sensor_listener(db, topic, msg_type))
        subs.append(sub)
        log.info("Subscribe: %s (%s)", topic, msg_type)

    cam_sub = session.declare_subscriber(CAMERA_TOPIC, make_camera_listener(db))
    subs.append(cam_sub)
    log.info("Subscribe kamera: %s", CAMERA_TOPIC)

    # Dengarkan perintah dari web dashboard (arah sebaliknya)
    col_ref = db.collection(FIREBASE_COLLECTION).document("commands").collection("inbox")
    cmd_watch = col_ref.on_snapshot(make_command_listener(session))
    log.info("Mendengarkan perintah web di koleksi commands_inbox...")

    stop_flag = {"stop": False}

    def handle_signal(signum, frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    log.info("Bridge (dua arah) berjalan. Tekan Ctrl+C untuk berhenti.")
    while not stop_flag["stop"]:
        time.sleep(0.2)

    cmd_watch.unsubscribe()
    for sub in subs:
        sub.undeclare()
    session.close()
    log.info("Bridge dihentikan.")


if __name__ == "__main__":
    main()
