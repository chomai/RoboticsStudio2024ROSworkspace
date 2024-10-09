#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/float64.hpp>


class MapLaserMatchingNode : public rclcpp::Node {
public:
    MapLaserMatchingNode() : Node("map_laser_matching_node") {
        // Parameters
        this->declare_parameter<std::string>("map_file", "/home/student/ros2_ws/sprint2_map.pgm");
        //this->declare_parameter<std::string>("map_file", "/home/student/ros2_ws/monday_map.pgm");
        this->declare_parameter<double>("yaw_threshold", 5.0);  // Degree threshold for localization NO LONGER USED
        this->get_parameter("map_file", map_file_);
        this->get_parameter("yaw_threshold", yaw_threshold_);

        // Load the map image (pgm file)
        map_image_ = cv::imread(map_file_, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image.");
            rclcpp::shutdown();
        }

        // Create subscribers for odometry and laser scan data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapLaserMatchingNode::odometryCallback, this, std::placeholders::_1));
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapLaserMatchingNode::laserScanCallback, this, std::placeholders::_1));

        // Create a publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //yaw_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/yaw_data", 10); // Create publisher
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_data", 10);  // Publish yaw angle
         // Timer to publish forward velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MapLaserMatchingNode::moveForward, this));
    
        

        //localized_ = false;  // Initialize localization state
    }

private:
    // Move the robot forward at a constant speed
    void moveForward() {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = -0.05; // Constant backwards speed

        // Publish the velocity command to move
        cmd_vel_pub_->publish(twist_msg);
    }
    // Callback for odometry messages
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;

        // Extract the section of the map around the robot
        cv::Mat map_section = extractMapSection();

        cv::imshow("Extracted Map Section", map_section);
        cv::waitKey(1);

        // Perform edge detection on the map section
        cv::Canny(map_section, map_edges_, 50, 150);
        cv::imshow("Map Edges", map_edges_);  // Display the edges
        cv::waitKey(1); // Wait 1 ms to display the image

        //inital pose - amcl
    } 

    // Callback for laser scan messages
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
       

        // Convert the laser scan to an image
        cv::Mat laser_image = laserScanToMat(msg);

        // Perform edge detection on the laser scan image
        cv::Mat laser_edges;
        cv::Canny(laser_image, laser_edges, 50, 150);
        //cv::imshow("Laser Scan Image", laser_image);
        cv::imshow("Laser Scan Edges", laser_edges);
        cv::waitKey(1);

        // Compare the edges of the map and laser scan to estimate the rotation
        estimateRotation(map_edges_, laser_edges);
    }


    // Extract a section of the map around the robot's current position
    cv::Mat extractMapSection() {
        int section_size = 150; 
        double x = current_pose_.position.x;
        double y = current_pose_.position.y;

        // Convert the robot's position to map coordinates
        // Flipping the X-axis and Y-axis directions
        int center_x = static_cast<int>(map_image_.cols / 2 + x / resolution_);
        int center_y = static_cast<int>(map_image_.rows / 2 - y / resolution_); // Flip Y-axis
    
        // Define the ROI and ensure it is within map bounds
        cv::Rect roi(center_x - section_size / 2, center_y - section_size / 2, section_size, section_size);
        roi &= cv::Rect(0, 0, map_image_.cols, map_image_.rows); // Ensure ROI is within map bounds

        return map_image_(roi).clone(); // Extract and return the map section
    }



    // Convert the laser scan data into a 2D image
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
        int center_x = img_size / 2;
        int center_y = img_size / 2;

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range < scan->range_max && range > scan->range_min) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>(center_x + range * cos(angle) * 100);
                int y = static_cast<int>(center_y + range * sin(angle) * 100);
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    img.at<uchar>(y, x) = 255; // Mark the point on the image
                }
            }
        }

        return img;
    }

        // Estimate yaw rotation by comparing the map and laser scan edges
    void estimateRotation(const cv::Mat& map_edges, const cv::Mat& laser_edges) {
        std::vector<cv::Point2f> map_points, laser_points;
        findFeaturePoints(map_edges, laser_edges, map_points, laser_points);

        if (map_points.size() >= 3 && laser_points.size() >= 3) {
            cv::Mat transform = cv::estimateAffinePartial2D(laser_points, map_points);
            if (transform.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix is empty.");
            } else {
                double angle = std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));
                double yaw_degrees = angle * (180.0 / CV_PI);
                RCLCPP_INFO(this->get_logger(), "Estimated Yaw Change: %.2f degrees", yaw_degrees);
                
                // Publish yaw data
                std_msgs::msg::Float64 yaw_msg;
                yaw_msg.data = yaw_degrees;
                yaw_pub_->publish(yaw_msg);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Not enough feature points: Map Points: %ld, Laser Points: %ld", map_points.size(), laser_points.size());
        }
    }



    // Detect and match features in the two edge images
    void findFeaturePoints(const cv::Mat& img1, const cv::Mat& img2,
                       std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        // Detect keypoints and compute descriptors
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Match descriptors using BFMatcher
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches by distance to get the best matches
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Keep the top 30% of matches
        const int numGoodMatches = static_cast<int>(matches.size() * 0.3);
        matches.erase(matches.begin() + numGoodMatches, matches.end());

        // Extract the matched points
        for (const auto& match : matches) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        // Draw matches between the laser scan and map images
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);

        // Show the matched features
        cv::imshow("Feature Matches", img_matches);
        cv::waitKey(1); // Update the display
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    
    cv::Mat map_image_;
    cv::Mat map_edges_;
    geometry_msgs::msg::Pose current_pose_;
    double resolution_ = 0.05; // Adjust based on map's resolution
    double yaw_threshold_;
    //bool localized_; // Flag to indicate whether the robot is localized

    std::string map_file_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLaserMatchingNode>());
    rclcpp::shutdown();
    return 0;
}
