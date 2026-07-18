#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <chrono>

static const float GT = 15.f; // toleransi jarak (cm) dianggap "sampai target"

static float wp(float a){while(a>M_PI)a-=2*M_PI;while(a<=-M_PI)a+=2*M_PI;return a;}
static float cl(float x,float lo,float hi){return std::max(lo,std::min(hi,x));}

using P2D=geometry_msgs::msg::Pose2D;
using TW=geometry_msgs::msg::Twist;
using MA=visualization_msgs::msg::MarkerArray;
using MK=visualization_msgs::msg::Marker;
using BL=std_msgs::msg::Bool;
using STR=std_msgs::msg::String;
using IMU=sensor_msgs::msg::Imu;

// =====================================================================
// Versi disederhanakan: TIDAK ADA lagi grid/Dijkstra/pathfinding hindari
// rintangan. Robot langsung menuju posisi TARGET (live dari ArUco) dengan
// garis lurus, pakai kontrol halus + fusi sensor IMU+kamera yang sudah
// terbukti stabil dari versi sebelumnya. Nama node & semua topic dibiarkan
// SAMA dengan versi Dijkstra lama (dijkstra_node, /mission/start, dst) -
// supaya launch file, dashboard, dan bridge Zenoh TIDAK PERLU diubah sama
// sekali.
// =====================================================================
class DN:public rclcpp::Node{
public:
    DN():Node("dijkstra_node"),lx(0),ly(0),lt(0),gx(0),gy(0),
        hl(false),hg(false),ac(false),started(false)
    {
        sl=create_subscription<P2D>("/aruco/leader_pose",10,
            std::bind(&DN::cbL,this,std::placeholders::_1));

        si=create_subscription<IMU>("/imu",50,
            std::bind(&DN::cbImu,this,std::placeholders::_1));

        sg=create_subscription<P2D>("/aruco/target_pose",10,
            std::bind(&DN::cbG,this,std::placeholders::_1));

        ss=create_subscription<BL>("/mission/start",10,
            std::bind(&DN::cbStart,this,std::placeholders::_1));

        sc=create_subscription<BL>("/mission/calibrate",10,
            std::bind(&DN::cbCalibrate,this,std::placeholders::_1));

        pc=create_publisher<TW>("/cmd_vel",10);
        pm=create_publisher<MA>("/dijkstra/path_markers",10);
        pw=create_publisher<P2D>("/dijkstra/current_waypoint",10);
        pst=create_publisher<STR>("/mission/status",10);

        tm=create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&DN::loop,this));

        RCLCPP_INFO(get_logger(),
            "Robot->Target (langsung, tanpa pathfinding) started | Goal=otomatis dari /aruco/target_pose | Menunggu /mission/start");
    }
private:
    void cbL(const P2D::SharedPtr m){
        lx=m->x;ly=m->y;
        // Fusi sensor: kamera + IMU (complementary filter) untuk theta.
        float cam_theta = m->theta;
        if (have_fused_) {
            float diff = wp(cam_theta - theta_fused_);
            theta_fused_ = wp(theta_fused_ + CAM_CORRECTION_GAIN * diff);
        } else {
            theta_fused_ = cam_theta;
            have_fused_ = true;
        }
        lt = theta_fused_;
        hl=true;
    }

    void cbImu(const IMU::SharedPtr m){
        double gz = m->angular_velocity.z;
        rclcpp::Time now = get_clock()->now();
        if (have_imu_time_) {
            double dt = (now - last_imu_time_).seconds();
            if (dt > 0.0 && dt < 0.5 && have_fused_) {
                theta_fused_ = wp(theta_fused_ + (float)(gz * dt));
                lt = theta_fused_;
            }
        }
        last_imu_time_ = now;
        have_imu_time_ = true;
    }

    void cbG(const P2D::SharedPtr m){
        gx=m->x;gy=m->y;hg=true;
    }

    void cbStart(const BL::SharedPtr m){
        started = m->data;
        if (started) {
            RCLCPP_INFO(get_logger(), "MISI DIMULAI oleh GUI!");
        } else {
            RCLCPP_INFO(get_logger(), "MISI DIHENTIKAN oleh GUI!");
            pc->publish(TW());
        }
    }

    void cbCalibrate(const BL::SharedPtr m){
        if (!m->data) return;
        RCLCPP_INFO(get_logger(), "KALIBRASI ditekan dari GUI!");
        if (!hl || !hg) {
            RCLCPP_WARN(get_logger(), "Leader atau Target belum terdeteksi, tidak bisa mulai!");
            return;
        }
        started = false;
        ac = true;
        drawTargetMarker();
        STR stmsg; stmsg.data="RUTE_SIAP"; pst->publish(stmsg);
        RCLCPP_INFO(get_logger(), "Siap! Robot akan menuju target secara langsung.");
    }

    void loop(){
        if(!ac||!hl||!hg) return;

        if (!started) {
            pc->publish(TW());
            return;
        }

        float wx=gx, wy=gy; // target LIVE - kalau target bergeser, ikut update
        float dx=wx-lx, dy=wy-ly;
        float dist=std::hypot(dx,dy);
        float ang=wp(std::atan2(dy,dx)-lt);

        P2D w2; w2.x=wx; w2.y=wy; pw->publish(w2);

        if(dist<GT){
            RCLCPP_INFO(get_logger(),"MISI SELESAI! Target tercapai.");
            ac=false; started=false; pc->publish(TW());
            STR s; s.data="TARGET_TERCAPAI"; pst->publish(s);
            return;
        }

        // Kontrol halus: w sebanding galat sudut, v sebanding cos(galat sudut)
        // (otomatis 0 kalau menghadap jauh dari target), + slew-rate limiter
        // supaya perintah motor tidak menyentak walau bacaan sensor jitter.
        float desiredW = cl(1.0f*ang, -1.2f, 1.2f);
        const float MAX_DW_PER_STEP = 0.12f;
        if(desiredW - prevW > MAX_DW_PER_STEP) desiredW = prevW + MAX_DW_PER_STEP;
        else if(prevW - desiredW > MAX_DW_PER_STEP) desiredW = prevW - MAX_DW_PER_STEP;
        prevW = desiredW;

        float speedScale = std::cos(ang);
        float v = (speedScale > 0.f) ? cl(0.8f*dist*speedScale, 0.f, 6.f) : 0.f;

        TW cmd; cmd.linear.x=v*0.01f; cmd.angular.z=desiredW;
        pc->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "->TARGET d=%.1f a=%.1f v=%.2f w=%.2f",
            dist,ang*180/M_PI,cmd.linear.x,desiredW);
    }

    void drawTargetMarker(){
        MA ma;
        MK m; m.header.frame_id="map"; m.header.stamp=now();
        m.ns="target"; m.id=0;
        m.type=MK::SPHERE; m.action=MK::ADD;
        m.pose.position.x=gx*0.01f;
        m.pose.position.y=gy*0.01f;
        m.pose.position.z=0.1f;
        m.pose.orientation.w=1;
        m.scale.x=m.scale.y=m.scale.z=0.2f;
        m.color.r=1;m.color.g=0.2f;m.color.a=1;
        ma.markers.push_back(m);
        pm->publish(ma);
    }

    rclcpp::Subscription<P2D>::SharedPtr sl,sg;
    rclcpp::Subscription<IMU>::SharedPtr si;
    rclcpp::Subscription<BL>::SharedPtr ss,sc;
    rclcpp::Publisher<TW>::SharedPtr pc;
    rclcpp::Publisher<MA>::SharedPtr pm;
    rclcpp::Publisher<P2D>::SharedPtr pw;
    rclcpp::Publisher<STR>::SharedPtr pst;
    rclcpp::TimerBase::SharedPtr tm;
    float lx,ly,lt,gx,gy;
    bool hl,hg;
    bool ac,started;
    float prevW=0.f;

    float theta_fused_=0.f;
    bool have_fused_=false;
    rclcpp::Time last_imu_time_;
    bool have_imu_time_=false;
    static constexpr float CAM_CORRECTION_GAIN = 0.15f;
};

int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DN>());
    rclcpp::shutdown();
    return 0;
}
