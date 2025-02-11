#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <openvino/openvino.hpp>
#include <librealsense2/rs.hpp>
#include <math.h>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb", 10);
        publisher_depth_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);
        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("camera/imu", 10);
        publisher_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

        // Load OpenVINO model
        ov::Core ie;
        auto model = ie.read_model("/home/deathstroke/Desktop/vision_safe_landing/openvino_midas_v21_small_256.xml", 
                                   "/home/deathstroke/Desktop/vision_safe_landing/openvino_midas_v21_small_256.bin");
        compiled_model_ = ie.compile_model(model, "CPU");
        input_layer_ = compiled_model_.input();
        output_layer_ = compiled_model_.output();

        // Open video capture
        cap_.open(4);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening video file");
            rclcpp::shutdown();
        }

        // Configure RealSense IMU pipeline
        rs2::config config;
        config.enable_stream(RS2_STREAM_ACCEL);
        config.enable_stream(RS2_STREAM_GYRO);
        pipeline_.start(config);

        // Timers
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&ImagePublisher::publish_frames, this));
        imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                             std::bind(&ImagePublisher::publish_imu, this));
        camera_info_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                     std::bind(&ImagePublisher::publish_camera_info, this));
    }

    ~ImagePublisher() {
        cap_.release();
        pipeline_.stop();
    }

private:
    void publish_frames() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Failed to read frame.");
            rclcpp::shutdown();
            return;
        }

        cv::resize(frame, frame, cv::Size(640, 480));
        cv::Mat img;
        cv::cvtColor(frame, img, cv::COLOR_BGR2RGB);
        cv::resize(img, img, cv::Size(256, 256));
        img.convertTo(img, CV_32F, 1.0 / 255.0);

        ov::Tensor input_tensor = ov::Tensor(input_layer_.get_element_type(), input_layer_.get_shape(), img.data);
        auto result = compiled_model_.infer({{input_layer_, input_tensor}});
        ov::Tensor output_tensor = result.at(output_layer_);

        cv::Mat depth_map(256, 256, CV_32F, output_tensor.data<float>());
        cv::resize(depth_map, depth_map, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
        depth_map /= cv::norm(depth_map, cv::NORM_MAX);

        auto msg_rgb = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        auto msg_depth = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_map).toImageMsg();
        
        publisher_rgb_->publish(*msg_rgb);
        publisher_depth_->publish(*msg_depth);
    }

    void publish_imu() {
        rs2::frameset frames = pipeline_.wait_for_frames();
        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
        
        if (accel_frame && gyro_frame) {
            auto accel_data = accel_frame.get_motion_data();
            auto gyro_data = gyro_frame.get_motion_data();

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.linear_acceleration.x = accel_data.x;
            imu_msg.linear_acceleration.y = accel_data.y;
            imu_msg.linear_acceleration.z = accel_data.z;
            imu_msg.angular_velocity.x = gyro_data.x;
            imu_msg.angular_velocity.y = gyro_data.y;
            imu_msg.angular_velocity.z = gyro_data.z;
            
            publisher_imu_->publish(imu_msg);
        }
    }

    void publish_camera_info() {
        double hfov = 87.0, vfov = 58.0;
        int width = 640, height = 480;
        double hfov_rad = hfov * M_PI / 180.0;
        double vfov_rad = vfov * M_PI / 180.0;
        double fx = (width / 2.0) / tan(hfov_rad / 2.0);
        double fy = (height / 2.0) / tan(vfov_rad / 2.0);
        double cx = width / 2.0, cy = height / 2.0;
        
        sensor_msgs::msg::CameraInfo camera_info_msg;
        camera_info_msg.height = height;
        camera_info_msg.width = width;
        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info_msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        
        publisher_camera_info_->publish(camera_info_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_camera_info_;
    rclcpp::TimerBase::SharedPtr timer_, imu_timer_, camera_info_timer_;
    cv::VideoCapture cap_;
    rs2::pipeline pipeline_;
    ov::CompiledModel compiled_model_;
    ov::Output<const ov::Node> input_layer_, output_layer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
