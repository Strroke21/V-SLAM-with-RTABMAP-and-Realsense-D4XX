// slam_localization.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/common/mavlink.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace mavsdk;

static int reset_counter = 1;
constexpr float jump_threshold = 0.1f;
constexpr float jump_speed_threshold = 20.0f;
constexpr double home_lat = 19.1345054;
constexpr double home_lon = 72.9120648;
constexpr float home_alt = 53.0;

void send_statustext(MavlinkPassthrough& passthrough, const std::string& text) {
    mavlink_message_t msg;
    mavlink_statustext_t statustext{};
    statustext.severity = MAV_SEVERITY_INFO;
    strncpy(statustext.text, text.c_str(), sizeof(statustext.text));
    mavlink_msg_statustext_encode(passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg, &statustext);
    passthrough.send_message(msg);
}

void send_home_position(MavlinkPassthrough& passthrough) {
    mavlink_message_t msg;
    mavlink_set_home_position_t home{};
    home.latitude = static_cast<int32_t>(home_lat * 1e7);
    home.longitude = static_cast<int32_t>(home_lon * 1e7);
    home.altitude = static_cast<int32_t>(home_alt);
    home.q[0] = 1.0f; home.q[1] = 0.0f; home.q[2] = 0.0f; home.q[3] = 0.0f;
    home.approach_x = 0; home.approach_y = 0; home.approach_z = 1;
    mavlink_msg_set_home_position_encode(passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg, &home);
    passthrough.send_message(msg);
}

void send_vision_position(MavlinkPassthrough& passthrough, float x, float y, float z, float roll, float pitch, float yaw, const std::array<float, 21>& cov) {
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(
        passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        x, y, z, roll, pitch, yaw,
        cov.data(),     // covariance pointer (float*)
        cov.size()      // covariance length (uint8_t)
    );
}

void send_vision_speed(MavlinkPassthrough& passthrough, float vx, float vy, float vz, const std::array<float, 9>& cov) {
    mavlink_message_t msg;
    mavlink_msg_vision_speed_estimate_pack(
        passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        vx, vy, vz,
        cov.data(),  // covariance pointer
        cov.size()   // covariance length
    );
    passthrough.send_message(msg);
}

void increment_reset_counter() {
    if (++reset_counter > 255)
        reset_counter = 1;
}

class SlamLocalizationNode : public rclcpp::Node {
public:
    SlamLocalizationNode(std::shared_ptr<System> system)
    : Node("slam_localization"),
      passthrough_(system)
    {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/rtabmap/odom",
        rclcpp::SensorDataQoS(),  // QoS compatible with rtabmap
        std::bind(&SlamLocalizationNode::odom_callback, this, _1));

    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto& pos = msg->pose.pose.position;
        auto& ori = msg->pose.pose.orientation;
        auto& lin = msg->twist.twist.linear;

        tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(), "[Pos] x: %.2f, y: %.2f, z: %.2f", pos.x, pos.y, pos.z);
        RCLCPP_INFO(get_logger(), "[Ori] roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);
        RCLCPP_INFO(get_logger(), "[Vel] x: %.2f, y: %.2f, z: %.2f", lin.x, lin.y, lin.z);

        std::array<float, 21> pos_cov{};
        std::copy_n(msg->pose.covariance.begin(), 21, pos_cov.begin());

        std::array<float, 9> vel_cov{};
        std::copy_n(msg->twist.covariance.begin(), 9, vel_cov.begin());

        send_vision_position(passthrough_, pos.x, pos.y, pos.z, roll, pitch, yaw, pos_cov);
        send_vision_speed(passthrough_, lin.x, lin.y, lin.z, vel_cov);

        if (has_prev_pos_) {
            float dx = pos.x - prev_x_;
            float dy = pos.y - prev_y_;
            float dz = pos.z - prev_z_;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist > jump_threshold) {
                RCLCPP_WARN(get_logger(), "Position jump detected: %.2f m", dist);
                increment_reset_counter();
            }
        }

        if (has_prev_vel_) {
            float dvx = lin.x - prev_vx_;
            float dvy = lin.y - prev_vy_;
            float dvz = lin.z - prev_vz_;
            float delta_speed = std::sqrt(dvx*dvx + dvy*dvy + dvz*dvz);
            if (delta_speed > jump_speed_threshold) {
                RCLCPP_WARN(get_logger(), "Speed jump detected: %.2f m/s", delta_speed);
                increment_reset_counter();
            }
        }

        prev_x_ = pos.x; prev_y_ = pos.y; prev_z_ = pos.z;
        prev_vx_ = lin.x; prev_vy_ = lin.y; prev_vz_ = lin.z;
        has_prev_pos_ = true;
        has_prev_vel_ = true;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    MavlinkPassthrough passthrough_;
    bool has_prev_pos_ = false;
    bool has_prev_vel_ = false;
    float prev_x_, prev_y_, prev_z_;
    float prev_vx_, prev_vy_, prev_vz_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    std::string fcu_address = "tcp://127.0.0.1:5763"; // ✅ note: use `tcp://`, not `tcp:`
    
    Mavsdk::Configuration config{ComponentType::GroundStation};
    Mavsdk mavsdk(config);  // ✅ only constructor that works

    ConnectionResult connection_result = mavsdk.add_any_connection(fcu_address);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    std::shared_ptr<System> system;
    while (true) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) {
            system = systems.at(0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    MavlinkPassthrough passthrough(system);
    send_statustext(passthrough, "SLAM localization node started");
    send_home_position(passthrough);

    auto node = std::make_shared<SlamLocalizationNode>(system);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



