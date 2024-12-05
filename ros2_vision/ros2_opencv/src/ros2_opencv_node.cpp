#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 messages and OpenCV representations.
#include <image_transport/image_transport.hpp> // For publishing and subscribing to compressed image streams in ROS2.
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
    MinimalImagePublisher() : Node("opencv_image_publisher"), marker_visible_(false) {
        
        // Publisher to republish processed images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

        // Subscriber to receive images from the camera
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/videocamera", 10,
            std::bind(&MinimalImagePublisher::image_callback, this, std::placeholders::_1));

        // Subscription to the topic aruco_single/pose
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&MinimalImagePublisher::pose_callback, this, std::placeholders::_1)
        );

        // Publisher for the detected ArUco marker pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);

        // Define the camera matrix and distortion coefficients
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320,
                                                    0, 600, 240,
                                                    0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.1, -0.05, 0, 0, 0);

        // Timer to continuously publish the pose
        timer_ = this->create_wall_timer(10ms, std::bind(&MinimalImagePublisher::publish_pose_continuously, this));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert the ROS 2 message to an OpenCV image
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Detect ArUco markers in the image
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;

            // Create the dictionary of ArUco markers
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

            // Configure the ArUco detector parameters for better accuracy
            cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
            detector_params->adaptiveThreshWinSizeMin = 5;
            detector_params->adaptiveThreshWinSizeMax = 50;
            detector_params->adaptiveThreshConstant = 5;
            detector_params->minMarkerPerimeterRate = 0.02;
            detector_params->polygonalApproxAccuracyRate = 0.03;

            // Detect the markers in the image
            cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, detector_params);

            // Draw the detected markers
            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

            // Update marker visibility status
            marker_visible_ = !marker_ids.empty();

            // Display the image with drawn markers
            cv::imshow("Debug Image", image);
            cv::waitKey(1);

            // Publish the processed image
            auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
            publisher_->publish(*output_msg);

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Save the last received pose message
        last_pose_msg_ = *msg;

        // Update timestamp and frame_id 
        last_pose_msg_.header.stamp = this->now();
        last_pose_msg_.header.frame_id = "camera_link";

        marker_visible_ = true;
    }

    void publish_pose_continuously() {
        if (marker_visible_) {
            pose_publisher_->publish(last_pose_msg_);
            RCLCPP_INFO(this->get_logger(), "Current pose published on /aruco_pose topic.");
        } else {
            RCLCPP_INFO(this->get_logger(), "No marker visible, no pose is published.");
        }
    }

    // Definitions of publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Last detected pose
    geometry_msgs::msg::PoseStamped last_pose_msg_;
    bool marker_visible_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<MinimalImagePublisher>();

    // Execute ROS 2 callbacks until SIGINT (ctrl-c) is received
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

