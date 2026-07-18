#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <map>

// =============================================
// ID MARKER
// =============================================
static const int LEADER_ID     = 0;
static const int FOLLOWER_ID   = 1;
static const int TARGET_ID     = 2;
static const int CORNER_TL_ID  = 3;   // Top-Left
static const int CORNER_TR_ID  = 4;   // Top-Right
static const int CORNER_BR_ID  = 5;   // Bottom-Right
static const int CORNER_BL_ID  = 6;   // Bottom-Left

static const double LEADER_YAW_OFFSET   = -M_PI / 2.0;
static const double FOLLOWER_YAW_OFFSET = -M_PI / 2.0;
static const double DETECTION_RADIUS_CM = 40.0;

// --- Koreksi paralaks ketinggian kamera overhead ---
// Homography dikalibrasi pakai marker sudut yang RATA DI LANTAI (z=0).
// Tapi marker LEADER/FOLLOWER dipasang di ATAS badan robot (tidak di
// lantai), jadi posisi yang dibaca via homography apa adanya akan sedikit
// "membesar" menjauhi titik tengah gambar kamera (efek paralaks) dibanding
// posisi asli robot. Kamera di dunia SDF ada di tinggi 5.0m menghadap lurus
// ke bawah, tepat di tengah arena; marker LEADER/FOLLOWER ada di tinggi
// ~0.71m dari lantai (dari robot.urdf: aruco_joint z=0.81 dikurangi tinggi
// base_link -0.10 relatif lantai). Konstanta di bawah dipakai untuk
// mengoreksi itu supaya posisi yang dipakai kontrol/dashboard sesuai posisi
// fisik robot yang sebenarnya, bukan yang "digelembungkan" paralaks.
static const double CAMERA_HEIGHT_CM       = 500.0;
static const double MARKER_HEIGHT_CM       = 71.0;
static const double ARENA_CENTER_LOCAL_X   = 420.0; // titik tengah arena dalam sistem (0-840,0-440)
static const double ARENA_CENTER_LOCAL_Y   = 220.0;

// Kembalikan posisi (dalam sistem lokal 0-840,0-440) yang sudah dikoreksi
// dari efek paralaks ketinggian. marker_height_cm=0 berarti sudah di lantai,
// tidak perlu koreksi (dipakai untuk marker TARGET & sudut arena).
static cv::Point2f correctHeightParallax(const cv::Point2f & apparent_local, double marker_height_cm) {
    if (marker_height_cm <= 0.0) return apparent_local;
    double xc = apparent_local.x - ARENA_CENTER_LOCAL_X;
    double yc = ARENA_CENTER_LOCAL_Y - apparent_local.y;
    double scale = (CAMERA_HEIGHT_CM - marker_height_cm) / CAMERA_HEIGHT_CM;
    double xc_true = xc * scale;
    double yc_true = yc * scale;
    float X_true = (float)(xc_true + ARENA_CENTER_LOCAL_X);
    float Y_true = (float)(ARENA_CENTER_LOCAL_Y - yc_true);
    return cv::Point2f(X_true, Y_true);
}

// Koordinat dunia nyata (cm) dari 4 sudut arena, HARUS SAMA PERSIS
// dengan posisi model corner_TL/TR/BR/BL di world SDF (meter -> cm).
static const cv::Point2f WORLD_TL(  0.0f,   0.0f);
static const cv::Point2f WORLD_TR(840.0f,   0.0f);
static const cv::Point2f WORLD_BR(840.0f, 440.0f);
static const cv::Point2f WORLD_BL(  0.0f, 440.0f);

static double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

static double smoothAngle(double prev, double next, double alpha = 0.3) {
    if (std::isnan(prev)) return next;
    return wrapPi(prev + alpha * wrapPi(next - prev));
}

static void drawGrid(cv::Mat & frame, int gs = 45) {
    int h = frame.rows, w = frame.cols;
    int cx = w/2, cy = h/2;
    cv::Scalar c(200,200,200);
    for (int x=cx; x<w; x+=gs) cv::line(frame,{x,0},{x,h},c,1);
    for (int x=cx; x>=0; x-=gs) cv::line(frame,{x,0},{x,h},c,1);
    for (int y=cy; y<h; y+=gs) cv::line(frame,{0,y},{w,y},c,1);
    for (int y=cy; y>=0; y-=gs) cv::line(frame,{0,y},{w,y},c,1);
    cv::line(frame,{cx,0},{cx,h},{255,255,255},2);
    cv::line(frame,{0,cy},{w,cy},{255,255,255},2);
}

class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector()
    : Node("aruco_detector"),
      prev_tl_(std::numeric_limits<double>::quiet_NaN()),
      prev_tf_(std::numeric_limits<double>::quiet_NaN()),
      last_leader_{-1,-1},
      last_follower_{-1,-1},
      follower_mode_("IDLE"),
      have_homography_(false)
    {
        aruco_dict_   = cv::aruco::getPredefinedDictionary(
                            cv::aruco::DICT_4X4_50);
        aruco_params_ = cv::aruco::DetectorParameters::create();
        cv::namedWindow("ArUco Detector - Overhead Camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("ArUco Detector - Overhead Camera", 640, 360);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/overhead_camera/image_raw", 10,
            std::bind(&ArucoDetector::cb, this,
                      std::placeholders::_1));

        pub_l_ = create_publisher<geometry_msgs::msg::Pose2D>(
            "/aruco/leader_pose", 10);
        pub_f_ = create_publisher<geometry_msgs::msg::Pose2D>(
            "/aruco/follower_pose", 10);
        pub_t_ = create_publisher<geometry_msgs::msg::Pose2D>(
            "/aruco/target_pose", 10);

        RCLCPP_INFO(get_logger(),
            "ArUco Detector C++ (Homography) started | "
            "Leader=ID%d Follower=ID%d Target=ID%d Corners=ID%d,%d,%d,%d",
            LEADER_ID, FOLLOWER_ID, TARGET_ID,
            CORNER_TL_ID, CORNER_TR_ID, CORNER_BR_ID, CORNER_BL_ID);
    }

private:
    void cb(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg,"bgr8")->image;
        } catch (...) { return; }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point2f>> corners, rej;
        std::vector<int> ids;
        cv::aruco::detectMarkers(gray, aruco_dict_,
                                 corners, ids,
                                 aruco_params_, rej);

        drawGrid(frame);
        updateHomography(corners, ids, frame);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Frame %dx%d | ids_found=%zu | rejected=%zu | homography=%s",
            frame.cols, frame.rows, ids.size(), rej.size(),
            have_homography_ ? "OK" : "BELUM");

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            process(frame, corners, ids);
        } else {
            cv::putText(frame, "No ArUco Detected",
                {20,50}, cv::FONT_HERSHEY_SIMPLEX,
                1.0, {0,0,255}, 2);
        }

        for (size_t i = 1; i < lpath_.size(); ++i)
            cv::line(frame, lpath_[i-1], lpath_[i], {0,255,0}, 2);

        cv::putText(frame,
            std::string("Follower: ") + follower_mode_ +
            (have_homography_ ? "  |  Arena: CALIBRATED" : "  |  Arena: MENUNGGU 4 SUDUT"),
            {10,30}, cv::FONT_HERSHEY_SIMPLEX,
            0.6, have_homography_ ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 2);

        cv::imshow("ArUco Detector - Overhead Camera", frame);
        cv::waitKey(1);
    }

    // ------------------------------------------------------------
    // Hitung homography dari 4 marker sudut (kalau ke-4nya terlihat)
    // ------------------------------------------------------------
    void updateHomography(
        const std::vector<std::vector<cv::Point2f>> & corners,
        const std::vector<int> & ids,
        cv::Mat & frame)
    {
        std::map<int, cv::Point2f> found;
        for (size_t i = 0; i < ids.size(); ++i) {
            int id = ids[i];
            if (id == CORNER_TL_ID || id == CORNER_TR_ID ||
                id == CORNER_BR_ID || id == CORNER_BL_ID) {
                const auto & pts = corners[i];
                cv::Point2f c(0,0);
                for (auto & p : pts) c += p;
                c *= (1.0f / 4.0f);
                found[id] = c;
            }
        }

        if (found.count(CORNER_TL_ID) && found.count(CORNER_TR_ID) &&
            found.count(CORNER_BR_ID) && found.count(CORNER_BL_ID)) {

            std::vector<cv::Point2f> src = {
                found[CORNER_TL_ID], found[CORNER_TR_ID],
                found[CORNER_BR_ID], found[CORNER_BL_ID]
            };
            std::vector<cv::Point2f> dst = {
                WORLD_TL, WORLD_TR, WORLD_BR, WORLD_BL
            };

            homography_ = cv::getPerspectiveTransform(src, dst);
            have_homography_ = true;

            // Gambar garis batas arena (visual feedback)
            for (int i = 0; i < 4; ++i) {
                cv::line(frame, src[i], src[(i+1)%4], {0,200,255}, 2);
                cv::circle(frame, src[i], 6, {0,200,255}, -1);
            }
        }
    }

    // Transform 1 titik pixel -> koordinat dunia nyata (cm)
    cv::Point2f pixelToWorldCm(const cv::Point2f & p) {
        double x = p.x, y = p.y;
        double Hm00 = homography_.at<double>(0,0), Hm01 = homography_.at<double>(0,1), Hm02 = homography_.at<double>(0,2);
        double Hm10 = homography_.at<double>(1,0), Hm11 = homography_.at<double>(1,1), Hm12 = homography_.at<double>(1,2);
        double Hm20 = homography_.at<double>(2,0), Hm21 = homography_.at<double>(2,1), Hm22 = homography_.at<double>(2,2);

        double w = Hm20 * x + Hm21 * y + Hm22;
        double X = (Hm00 * x + Hm01 * y + Hm02) / w;
        double Y = (Hm10 * x + Hm11 * y + Hm12) / w;
        return cv::Point2f((float)X, (float)Y);
    }

    void process(cv::Mat & frame,
        const std::vector<std::vector<cv::Point2f>> & corners,
        const std::vector<int> & ids)
    {
        if (!have_homography_) return;  // tunggu kalibrasi arena dulu

        for (size_t i = 0; i < ids.size(); ++i) {
            int id = ids[i];
            const auto & pts = corners[i];

            cv::Point2f center_px(0,0);
            for (auto & p : pts) center_px += p;
            center_px *= (1.0f / 4.0f);
            int cxi = static_cast<int>(center_px.x);
            int cyi = static_cast<int>(center_px.y);

            // Posisi akurat dalam cm (dunia nyata) via homography
            cv::Point2f center_world = pixelToWorldCm(center_px);
            // Marker LEADER/FOLLOWER dipasang di atas badan robot (bukan di
            // lantai), jadi perlu koreksi paralaks. Marker TARGET & sudut
            // arena tidak perlu (sudah rata di lantai) - lihat pemakaian di
            // bawah, koreksi cuma dipanggil untuk id LEADER/FOLLOWER.
            double cx_cm = center_world.x;
            double cy_cm = center_world.y;

            // Orientasi: transform 2 titik sudut marker (TL, TR) ke dunia nyata
            cv::Point2f tl_w = pixelToWorldCm(pts[0]);
            cv::Point2f tr_w = pixelToWorldCm(pts[1]);
            double vx = tr_w.x - tl_w.x;
            double vy = tr_w.y - tl_w.y;
            double theta = std::atan2(vy, vx);

            if (id == LEADER_ID) {
                cv::Point2f corrected = correctHeightParallax(cv::Point2f((float)cx_cm,(float)cy_cm), MARKER_HEIGHT_CM);
                cx_cm = corrected.x; cy_cm = corrected.y;

                theta = wrapPi(theta + LEADER_YAW_OFFSET);
                theta = smoothAngle(prev_tl_, theta, 0.3);
                prev_tl_ = theta;
                lpath_.push_back({cxi,cyi});
                last_leader_ = {cxi,cyi};

                geometry_msgs::msg::Pose2D p;
                p.x=cx_cm; p.y=cy_cm; p.theta=theta;
                pub_l_->publish(p);

                int L=40; double td=-theta;
                cv::arrowedLine(frame,{cxi,cyi},
                    {cxi+(int)(L*cos(td)), cyi+(int)(L*sin(td))},
                    {0,255,255},2,8,0,0.35);
                cv::circle(frame,{cxi,cyi}, (int)(DETECTION_RADIUS_CM), {0,255,255},1);
                cv::putText(frame,"LEADER",{cxi-30,cyi-15},
                    cv::FONT_HERSHEY_SIMPLEX,0.6,{0,255,0},2);
                cv::putText(frame,
                    "("+std::to_string((int)cx_cm)+","+std::to_string((int)cy_cm)+")cm",
                    {cxi-30,cyi+20},
                    cv::FONT_HERSHEY_SIMPLEX,0.5,{0,255,0},1);
                cv::circle(frame,{cxi,cyi},5,{0,255,0},-1);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "LEADER (%.1f,%.1f)cm theta=%.1fdeg",
                    cx_cm, cy_cm, theta*180/M_PI);
            }
            else if (id == FOLLOWER_ID) {
                cv::Point2f corrected = correctHeightParallax(cv::Point2f((float)cx_cm,(float)cy_cm), MARKER_HEIGHT_CM);
                cx_cm = corrected.x; cy_cm = corrected.y;

                theta = wrapPi(theta + FOLLOWER_YAW_OFFSET);
                theta = smoothAngle(prev_tf_, theta, 0.3);
                prev_tf_ = theta;
                last_follower_ = {cxi,cyi};

                geometry_msgs::msg::Pose2D p;
                p.x=cx_cm; p.y=cy_cm; p.theta=theta;
                pub_f_->publish(p);

                if (last_leader_.x >= 0) {
                    double dx=cxi-last_leader_.x;
                    double dy=cyi-last_leader_.y;
                    double dist=std::hypot(dx,dy);
                    follower_mode_ = dist<=DETECTION_RADIUS_CM ? "FOLLOW":"CATCH_UP";
                    cv::arrowedLine(frame,{cxi,cyi},last_leader_,
                        {0,100,255},2,8,0,0.3);
                }

                int L=40; double td=-theta;
                cv::arrowedLine(frame,{cxi,cyi},
                    {cxi+(int)(L*cos(td)), cyi+(int)(L*sin(td))},
                    {0,255,255},2,8,0,0.35);
                cv::putText(frame,"FOLLOWER["+follower_mode_+"]",
                    {cxi-50,cyi-15},
                    cv::FONT_HERSHEY_SIMPLEX,0.6,{0,0,255},2);
                cv::circle(frame,{cxi,cyi},5,{0,0,255},-1);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "FOLLOWER (%.1f,%.1f)cm mode=%s",
                    cx_cm, cy_cm, follower_mode_.c_str());
            }
            else if (id == TARGET_ID) {
                geometry_msgs::msg::Pose2D p;
                p.x = cx_cm; p.y = cy_cm; p.theta = 0.0;
                pub_t_->publish(p);

                cv::putText(frame, "TARGET",{cxi-35,cyi-15},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,0,255}, 2);
                cv::putText(frame,
                    "("+std::to_string((int)cx_cm)+","+std::to_string((int)cy_cm)+")cm",
                    {cxi-30,cyi+20},
                    cv::FONT_HERSHEY_SIMPLEX,0.5,{255,0,255},1);
                cv::circle(frame,{cxi,cyi},8,{255,0,255},-1);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TARGET (%.1f,%.1f)cm", cx_cm, cy_cm);
            }
            // id CORNER_* sudah ditangani di updateHomography(), tidak perlu apa-apa lagi
        }
    }

    cv::Ptr<cv::aruco::Dictionary>         aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr
        pub_l_, pub_f_, pub_t_;
    std::vector<cv::Point> lpath_;
    cv::Point last_leader_, last_follower_;
    std::string follower_mode_;
    double prev_tl_, prev_tf_;

    cv::Mat homography_;
    bool have_homography_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
