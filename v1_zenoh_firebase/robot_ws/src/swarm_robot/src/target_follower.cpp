#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <chrono>
#include <string>

static const double GOAL_TOLERANCE_CM = 15.0;
static const double V_MAX_CM_S        = 15.0;
static const double W_MAX_RAD_S       = 0.9;   // diturunkan lagi -> kurangi motion blur ArUco saat berputar
static const double KV                = 1.5;
static const double KW                = 1.5;   // diturunkan dari 3.0 -> tidak overshoot
static const double CM_TO_M           = 0.01;
static const double MAX_DW_PER_STEP   = 0.10;  // slew-rate limiter (rad/s per siklus 50ms)

static double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}
static double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

enum class FollowState { ALIGNING, MOVING };

class TargetFollower : public rclcpp::Node {
public:
    TargetFollower()
    : Node("target_follower"),
      lx_(0), ly_(0), lth_(0),
      tx_(0), ty_(0),
      have_leader_(false), have_target_(false), mission_started_(false),
      prev_w_(0.0),
      last_leader_update_(this->now()),
      state_(FollowState::ALIGNING)
    {
        sub_leader_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/aruco/leader_pose", 10,
            std::bind(&TargetFollower::cbLeader, this, std::placeholders::_1));

        sub_target_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/aruco/target_pose", 10,
            std::bind(&TargetFollower::cbTarget, this, std::placeholders::_1));

        sub_start_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/start", 10,
            std::bind(&TargetFollower::cbStart, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        pub_status_ = create_publisher<std_msgs::msg::String>(
            "/mission/status", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TargetFollower::loop, this));

        RCLCPP_INFO(get_logger(),
            "Target Follower started | Menunggu ArUco LEADER dan TARGET terdeteksi...");
    }

private:
    void publishStatus(const std::string & s) {
        if (s == last_status_) return;  // cuma kirim kalau berubah, hemat kuota Firestore
        last_status_ = s;
        std_msgs::msg::String m;
        m.data = s;
        pub_status_->publish(m);
    }

    void cbLeader(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        lx_ = msg->x; ly_ = msg->y;
        // Filter low-pass pada theta supaya tidak melompat-lompat akibat noise
        // deteksi ArUco (pelajaran dari debugging dijkstra_node sebelumnya).
        if (have_leader_) {
            double diff = wrapPi(msg->theta - lth_);
            lth_ = wrapPi(lth_ + 0.35 * diff);
        } else {
            lth_ = msg->theta;
        }
        have_leader_ = true;
        last_leader_update_ = this->now();
    }

    void cbStart(const std_msgs::msg::Bool::SharedPtr msg) {
        mission_started_ = msg->data;
        if (mission_started_) {
            RCLCPP_INFO(get_logger(), "MISI DIMULAI oleh GUI!");
            state_ = FollowState::ALIGNING;
            prev_w_ = 0.0;
            publishStatus("SEDANG_PUTAR");
        } else {
            RCLCPP_INFO(get_logger(), "MISI DIHENTIKAN oleh GUI!");
            prev_w_ = 0.0;
            publishStatus("MISI_DIHENTIKAN");
        }
    }

    void cbTarget(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        if (!have_target_) {
            RCLCPP_INFO(get_logger(), "TARGET terdeteksi! Mulai bergerak menuju target.");
        }
        tx_ = msg->x; ty_ = msg->y;
        have_target_ = true;
    }

    void loop() {
        geometry_msgs::msg::Twist cmd;

        // === Pemantauan jarak ke target: BERJALAN TERUS, tidak peduli mode
        // Otomatis atau Manual, supaya status "TARGET_TERCAPAI" tetap
        // dilaporkan ke dashboard biarpun robot dikendalikan manual. Bagian
        // ini HANYA memantau & melapor, TIDAK PERNAH mengirim perintah motor
        // -- supaya tidak mengganggu kendali manual dari dashboard.
        if (have_leader_ && have_target_) {
            double mdx = tx_ - lx_;
            double mdy = ty_ - ly_;
            double mdist = std::hypot(mdx, mdy);
            if (mdist < GOAL_TOLERANCE_CM) {
                publishStatus("TARGET_TERCAPAI");
            } else if (mission_started_) {
                // biarkan status "SEDANG_PUTAR"/"SEDANG_MAJU" (di-set di
                // tempat lain) tetap berlaku saat misi otomatis aktif
            } else {
                publishStatus("MODE_MANUAL");
            }
        }

        if (!have_leader_ || !have_target_ || !mission_started_) {
            // PENTING: kirim "berhenti" cuma SEKALI saat baru masuk kondisi
            // idle (misi belum mulai / belum terdeteksi), lalu diam. Kalau
            // ini terus-menerus mengirim tiap 50ms, perintah gerak MANUAL
            // dari dashboard (yang juga menulis ke /cmd_vel) akan langsung
            // ketimpa lagi 20x per detik oleh node ini — itu sebabnya
            // tombol panah manual tidak berfungsi sebelumnya.
            if (!idle_stop_sent_) {
                pub_cmd_->publish(cmd);
                idle_stop_sent_ = true;
            }
            prev_w_ = 0.0;
            return;
        }
        idle_stop_sent_ = false;  // misi aktif lagi, siap kirim stop-sekali berikutnya kalau idle lagi

        // PENGAMAN: kalau data posisi/arah leader sudah "basi" (kamera sempat
        // gagal mendeteksi marker, misal karena buram saat robot berputar
        // cepat), JANGAN lanjut kirim perintah berdasarkan data lama —
        // berhenti dulu sampai data segar kembali. Tanpa ini, robot bisa
        // terus "berputar buta" berdasarkan posisi yang sudah tidak akurat.
        double stale_sec = (this->now() - last_leader_update_).seconds();
        if (stale_sec > 0.3) {
            pub_cmd_->publish(cmd);  // stop sementara, tunggu data segar
            prev_w_ = 0.0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Data posisi leader basi (%.2fs) - berhenti sementara, tunggu deteksi kamera.",
                stale_sec);
            return;
        }

        double dx = tx_ - lx_;
        double dy = ty_ - ly_;
        double dist = std::hypot(dx, dy);
        double bearing = std::atan2(dy, dx);
        double ang_err_local = wrapPi(bearing - lth_);
        // PERBAIKAN PENTING: sumbu Y di sistem koordinat lokal (cm) dibalik
        // dari konvensi asli Gazebo (lihat rumus y_lokal = 220 - y_gazebo di
        // aruco_detector.cpp). Membalik satu sumbu otomatis membalik "arah
        // rasa putaran" (CW/CCW), sehingga galat sudut yang dihitung di sini
        // adalah KEBALIKAN dari galat sudut asli di dunia Gazebo. Twist yang
        // dikirim ke robot diinterpretasi memakai konvensi Gazebo asli, jadi
        // di sini harus dibalik lagi supaya robot berputar ke arah yang benar.
        double ang_err = -ang_err_local;

        if (dist < GOAL_TOLERANCE_CM) {
            pub_cmd_->publish(cmd);  // sudah sampai, stop
            prev_w_ = 0.0;
            publishStatus("TARGET_TERCAPAI");
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Target tercapai! dist=%.1fcm", dist);
            return;
        }

        // === Strategi dua-tahap: PUTAR dulu sampai pas, baru MAJU ===
        // Sebelumnya robot boleh maju sedikit sambil masih belok besar,
        // sehingga lintasannya melengkung (bukan garis lurus) dan bisa
        // melenceng terlalu dekat ke dinding arena. Sekarang: robot WAJIB
        // diam & berputar dulu sampai benar-benar menghadap target, baru
        // boleh maju (dengan koreksi arah kecil saja selama maju). Kalau di
        // tengah jalan arahnya melenceng jauh, berhenti maju & putar ulang.
        const double ALIGN_DONE_RAD   = 0.08;  // ~4.6 derajat: dianggap "sudah lurus"
        const double REALIGN_TRIGGER  = 0.35;  // ~20 derajat: kalau melenceng segini, putar ulang

        if (state_ == FollowState::ALIGNING) {
            double w_desired = clamp(1.5 * ang_err, -W_MAX_RAD_S, W_MAX_RAD_S);
            double w;
            if (w_desired - prev_w_ > MAX_DW_PER_STEP) w = prev_w_ + MAX_DW_PER_STEP;
            else if (prev_w_ - w_desired > MAX_DW_PER_STEP) w = prev_w_ - MAX_DW_PER_STEP;
            else w = w_desired;
            prev_w_ = w;

            cmd.linear.x = 0.0;
            cmd.angular.z = w;
            pub_cmd_->publish(cmd);

            if (std::fabs(ang_err) < ALIGN_DONE_RAD) {
                state_ = FollowState::MOVING;
                publishStatus("SEDANG_MAJU");
                RCLCPP_INFO(get_logger(), "Sudah lurus menghadap target, mulai maju.");
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[PUTAR] dist=%.1fcm ang=%.1fdeg w=%.2frad/s",
                dist, ang_err * 180.0 / M_PI, w);
            return;
        }

        // state_ == MOVING
        if (std::fabs(ang_err) > REALIGN_TRIGGER) {
            state_ = FollowState::ALIGNING;
            prev_w_ = 0.0;
            publishStatus("SEDANG_PUTAR");
            RCLCPP_INFO(get_logger(), "Melenceng terlalu jauh (%.1fdeg), berhenti & putar ulang.",
                ang_err * 180.0 / M_PI);
            pub_cmd_->publish(cmd);  // stop dulu di siklus ini
            return;
        }

        {
            double v = clamp(KV * dist, 0.0, V_MAX_CM_S);
            // Koreksi arah kecil saja selama maju (gain lebih rendah + batas
            // lebih sempit), supaya lintasan tetap kurang-lebih lurus.
            double w_desired = clamp(0.8 * ang_err, -0.5, 0.5);
            double w;
            if (w_desired - prev_w_ > MAX_DW_PER_STEP) w = prev_w_ + MAX_DW_PER_STEP;
            else if (prev_w_ - w_desired > MAX_DW_PER_STEP) w = prev_w_ - MAX_DW_PER_STEP;
            else w = w_desired;
            prev_w_ = w;

            cmd.linear.x  = v * CM_TO_M;
            cmd.angular.z = w;
            pub_cmd_->publish(cmd);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[MAJU] dist=%.1fcm ang=%.1fdeg v=%.2fm/s w=%.2frad/s",
                dist, ang_err * 180.0 / M_PI, cmd.linear.x, w);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_leader_, sub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    double lx_, ly_, lth_;
    double tx_, ty_;
    bool have_leader_, have_target_, mission_started_;
    bool idle_stop_sent_ = false;
    double prev_w_;
    rclcpp::Time last_leader_update_;
    FollowState state_;
    std::string last_status_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetFollower>());
    rclcpp::shutdown();
    return 0;
}
