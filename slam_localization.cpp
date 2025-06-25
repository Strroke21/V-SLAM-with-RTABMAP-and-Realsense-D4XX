#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <array>
#include <memory>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

// Utilities for quaternion to euler conversion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono;
using namespace mavsdk;


uint8_t reset_counter = 1;
constexpr double jump_threshold = 0.1; // meters

void increment_reset_counter()
{
    if (reset_counter >= 255) {
        reset_counter = 1;
    }
    reset_counter += 1;
}

void vision_position_send(MavlinkPassthrough& mavlink_passthrough, float x, float y, float z, float roll, float pitch, float yaw)
{
    uint64_t usec = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        usec, x, y, z, roll, pitch, yaw
    );
    mavlink_passthrough.send_message(message);
}

void vision_speed_send(MavlinkPassthrough& mavlink_passthrough, float vx, float vy, float vz)
{
    uint64_t usec = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

    mavlink_message_t message;
    mavlink_msg_vision_speed_estimate_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &message,
        usec, vx, vy, vz
    );
    mavlink_passthrough.send_message(message);
}

class SlamLocalization : public rclcpp::Node
{
public:
    SlamLocalization(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough)
        : Node("localization"), mavlink_passthrough_(mavlink_passthrough)
    {
        using std::placeholders::_1;
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/rtabmap/localization_pose", 10,
            std::bind(&SlamLocalization::localization_pose_callback, this, _1)
        );

        // QoS for BEST_EFFORT (matches rtabmap odom)
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", qos_profile,
            std::bind(&SlamLocalization::odom_callback, this, _1)
        );

        prev_position_ = std::nullopt;
    }

private:
    void localization_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const auto& position = msg->pose.pose.position;
        const auto& orientation = msg->pose.pose.orientation;

        // Eigen/TF expects w,x,y,z
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        vision_position_send(*mavlink_passthrough_, position.x, position.y, position.z, roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), 
            "Sending vision position estimate to vehicle\nPos_x: %.3f, Pos_y: %.3f, Pos_z: %.3f, Roll= %.3f, Pitch: %.3f, Yaw: %.3f, reset_counter: %d",
            position.x, position.y, position.z, roll, pitch, yaw, reset_counter);

        if (prev_position_) {
            double dx = position.x - prev_position_->x;
            double dy = position.y - prev_position_->y;
            double dz = position.z - prev_position_->z;
            double position_displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (position_displacement > jump_threshold) {
                RCLCPP_WARN(this->get_logger(),
                    "Position jump detected: %.3f m, resetting reset_counter.", position_displacement);
                increment_reset_counter();
            }
        }

        prev_position_ = position;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& linear = msg->twist.twist.linear;
        RCLCPP_INFO(this->get_logger(),
            "Linear Velocity - x: %.3f, y: %.3f, z: %.3f",
            linear.x, linear.y, linear.z
        );
        vision_speed_send(*mavlink_passthrough_, linear.x, linear.y, linear.z);
    }

    std::shared_ptr<MavlinkPassthrough> mavlink_passthrough_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    std::optional<geometry_msgs::msg::Point> prev_position_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Setup MAVSDK system and connection string here (UDP, TCP, or Serial)
    Mavsdk mavsdk;
    std::string connection_url = "udp://10.61.225.90:14550"; // Replace with your connection string
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Failed to connect: " << connection_result << std::endl;
        return 1;
    }

    // Wait for system to connect
    std::shared_ptr<System> system;
    for (int i = 0; i < 20; ++i) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) {
            system = systems.front();
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!system) {
        std::cerr << "No system found, exiting." << std::endl;
        return 1;
    }

    // Create MavlinkPassthrough plugin
    auto mavlink_passthrough = std::make_shared<MavlinkPassthrough>(*system);

    auto node = std::make_shared<SlamLocalization>(mavlink_passthrough);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

//g++ slam_localization.cpp -lmavsdk -lpthread -I/opt/ros/humble/include/rclcpp -o slam_localization
