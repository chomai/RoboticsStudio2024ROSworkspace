#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class SimpleLocalizerNode : public rclcpp::Node {
public:
    SimpleLocalizerNode() : Node("simple_localizer_node") {
        
        // Subscribers to LaserScan and Odometry
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizerNode::scanCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleLocalizerNode::odomCallback, this, std::placeholders::_1));

        // Initialize robot's known pose
        robot_pose_ = cv::Point2f(100, 100);  // Example initial known location
        RCLCPP_INFO(this->get_logger(), "Simple Localizer Node started.");
    }

private:
    // Robot's current known position and orientation
    cv::Point2f robot_pose_;
    double robot_orientation_ = 0.0;  // In radians

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // Map section (Image A)
    cv::Mat map_image_, map_edge_image_;

    // Function to handle incoming laser scans
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert the laser scan into an image (Image C)
        cv::Mat laser_image = laserScanToImage(msg);

        // Compare map edges (Image B) with laser scan image (Image C)
        double rotation_estimate = estimateRotation(map_edge_image_, laser_image);

        RCLCPP_INFO(this->get_logger(), "Estimated Rotation: %.2f degrees", rotation_estimate);
        
        // Update the robot's pose based on odometry and the estimated rotation
        updatePose(rotation_estimate);
    }

    // Function to handle odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Use odometry data to propagate the robot's position and orientation
        
        // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::Quaternion tf_quat;
        tf2::fromMsg(msg->pose.pose.orientation, tf_quat);

        // Get the yaw (rotation around the Z axis) from the quaternion
        robot_orientation_ = tf2::getYaw(tf_quat);
    }

    // Function to convert laser scan into an image (Image C)
    cv::Mat laserScanToImage(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        cv::Mat laser_image = cv::Mat::zeros(img_size, img_size, CV_8UC1);
        int center_x = img_size / 2;
        int center_y = img_size / 2;

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range < scan->range_max && range > scan->range_min) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>(center_x + range * cos(angle) * 100);
                int y = static_cast<int>(center_y + range * sin(angle) * 100);
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    laser_image.at<uchar>(y, x) = 255;
                }
            }
        }

        return laser_image;
    }

    // Function to extract a section of the map around the robot (Image A) and create an edge image (Image B)
    cv::Mat extractMapSection() {
        int section_size = 500;  // Extract a 500x500 section around the robot
        cv::Mat map_section = map_image_(cv::Rect(robot_pose_.x - section_size / 2, robot_pose_.y - section_size / 2, section_size, section_size)).clone();
        
        // Apply edge detection to create an edge image (Image B)
        cv::Mat map_edge_image;
        cv::Canny(map_section, map_edge_image, 50, 150);
        return map_edge_image;
    }

    // Function to estimate rotation by comparing map edges and laser scan images
    double estimateRotation(const cv::Mat& map_edge_image, const cv::Mat& laser_image) {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(map_edge_image, laser_image, srcPoints, dstPoints);

        if (srcPoints.size() >= 3 && dstPoints.size() >= 3) {
            cv::Mat transformation_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (!transformation_matrix.empty()) {
                double angle_radians = std::atan2(transformation_matrix.at<double>(1, 0),
                                                  transformation_matrix.at<double>(0, 0));
                return angle_radians * (180.0 / M_PI);  // Convert to degrees
            }
        }
        return 0.0;
    }

    // Function to detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Keep the best matches
        size_t numGoodMatches = std::min(matches.size(), static_cast<size_t>(30));
        matches.resize(numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Function to update the robot's pose using odometry and the estimated rotation
    void updatePose(double rotation_estimate) {
        robot_orientation_ += rotation_estimate * (M_PI / 180.0);  // Update orientation in radians
        // Optionally, you could also update the position using odometry data
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleLocalizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
