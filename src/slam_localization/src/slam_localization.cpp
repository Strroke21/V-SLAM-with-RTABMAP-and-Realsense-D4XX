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

void send_vision_position(MavlinkPassthrough& passthrough, float x, float y, float z, float roll, float pitch, float yaw) {
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(
        passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        x, y, z, roll, pitch, yaw,
        0, // Assuming no covariance for simplicity
        0 // Reset counter (not used)
    );
}

void send_vision_speed(MavlinkPassthrough& passthrough, float vx, float vy, float vz) {
    mavlink_message_t msg;
    mavlink_msg_vision_speed_estimate_pack(
        passthrough.get_our_sysid(), passthrough.get_our_compid(), &msg,
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000),
        vx, vy, vz,
        0, // Assuming no covariance for simplicity
        0 // Reset counter (not used)
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

        float cam_x = pos.z;
        float cam_y = -pos.y;
        float cam_z = pos.x;
        float cam_roll = yaw;
        float cam_pitch = pitch;
        float cam_yaw = roll;
        float cam_vx = lin.z;
        float cam_vy = -lin.y;
        float cam_vz = lin.x;

        RCLCPP_INFO(get_logger(), "[SLAM] x: %.2f, y: %.2f, z: %.2f", cam_x, cam_y, cam_z);
        RCLCPP_INFO(get_logger(), "[Ori] roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

        std::array<float, 21> pos_cov{};
        std::copy_n(msg->pose.covariance.begin(), 21, pos_cov.begin());

        std::array<float, 9> vel_cov{};
        std::copy_n(msg->twist.covariance.begin(), 9, vel_cov.begin());

        send_vision_position(passthrough_, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);
        send_vision_speed(passthrough_, cam_vx, cam_vy, cam_vz);
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
    
    std::string fcu_address = "/dev/ttyACM0"; //fcu address
    
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


// build env: colcon build --packages-select slam_localization --symlink-install
// source env: source install/setup.bash
// run node: ros2 run slam_localization slam_node
