#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>



class ScanMatchingLocalizer : public rclcpp::Node {
public:
    ScanMatchingLocalizer() : Node("scan_matching_localizer"), 
                              first_image_captured_(false), 
                              robot_initialized_(false) {
        // Parameters
        this->declare_parameter<std::string>("map_file", "/home/student/ros2_ws/world_map.pgm");
        this->declare_parameter<std::string>("laser_scan_topic", "/scan");
        this->declare_parameter<std::string>("odometry_topic", "/odom");

        // Get parameters
        std::string map_file;
        this->get_parameter("map_file", map_file);
        this->get_parameter("laser_scan_topic", laser_scan_topic_);
        this->get_parameter("odometry_topic", odometry_topic_);

        // Load map image
        map_image_ = cv::imread(map_file, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image.");
            rclcpp::shutdown();
        }

        // Initialize subscribers
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan_topic_, 10,
            std::bind(&ScanMatchingLocalizer::laserScanCallback, this, std::placeholders::_1));
        
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, 10,
            std::bind(&ScanMatchingLocalizer::odometryCallback, this, std::placeholders::_1));

        // Initialize OpenCV window
        cv::namedWindow("Laser Scan Image", cv::WINDOW_AUTOSIZE);
    }

private:    
	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	    // Update robot pose from odometry
	    current_pose_ = msg->pose.pose;
	    
	    // Create a tf2::Quaternion object
	    tf2::Quaternion q;
	    
	    // Convert the orientation from msg to tf2::Quaternion
	    q.setX(current_pose_.orientation.x);
	    q.setY(current_pose_.orientation.y);
	    q.setZ(current_pose_.orientation.z);
	    q.setW(current_pose_.orientation.w);
	    
	    // Optionally, you can use the quaternion for further calculations
	    double roll, pitch, yaw;
	    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

	    RCLCPP_INFO(this->get_logger(), "Updated robot pose.Orientation (Roll, Pitch, Yaw): %.2f, %.2f, %.2f", roll, pitch, yaw);

	    // Check if robot is initialized
	    if (!robot_initialized_) {
		RCLCPP_INFO(this->get_logger(), "Robot has been initialized.");
		robot_initialized_ = true;
	    }
	}

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (first_image_captured_) {
            // Check if the robot is localized
            if (isRobotLocalized()) {
                RCLCPP_INFO(this->get_logger(), "Robot is localized.");
                rclcpp::shutdown(); // Stop processing once robot is localized
                return; 
            }
        }

        // Step 2: Extract section of the map around the robot
        cv::Mat map_section = extractMapSection();

        // Step 3: Extract edges from the map section
        cv::Mat map_edges;
        cv::Canny(map_section, map_edges, 50, 150);

        // Step 4: Convert laser scan to image
        cv::Mat laser_image = laserScanToMat(msg);
        RCLCPP_INFO(this->get_logger(), "Map section size: %dx%d", map_section.cols, map_section.rows);
	RCLCPP_INFO(this->get_logger(), "Map edges size: %dx%d", map_edges.cols, map_edges.rows);
	RCLCPP_INFO(this->get_logger(), "Laser image size: %dx%d", laser_image.cols, laser_image.rows);


        if (first_image_captured_) {
        // Step 5: Compare map edges (Image B) and laser scan image (Image C)
		RCLCPP_INFO(this->get_logger(), "STEP5.");
		calculateYawChange(map_edges, laser_image);
	    } else {
		// For the first image, set the flag and store the image
		first_image_captured_ = true;
	    }

        // Update images
        previous_image_ = current_image_.clone();
        current_image_ = laser_image.clone();
        //first_image_captured_ = true;

        // Display the image using OpenCV
        if (current_image_.empty()) {
	    RCLCPP_WARN(this->get_logger(), "Current image is empty.");
	} else {
	    cv::imshow("Laser Scan Image", current_image_);
	    cv::waitKey(1); // Wait for 1 ms to update the display
	}

    }

    cv::Mat extractMapSection() {
        // Extract a section of the map around the robot
        int section_size = 200; // Define the size of the map section
        int center_x = static_cast<int>(current_pose_.position.x + map_image_.cols / 2);
        int center_y = static_cast<int>(current_pose_.position.y + map_image_.rows / 2);

        cv::Rect region(center_x - section_size / 2, center_y - section_size / 2, section_size, section_size);
        region &= cv::Rect(0, 0, map_image_.cols, map_image_.rows); // Ensure the region is within bounds

        return map_image_(region);
    }

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
                    img.at<uchar>(y, x) = 255;
                }
            }
        }

        return img;
    }

	void calculateYawChange(const cv::Mat& map_edges, const cv::Mat& laser_image) {
	    // Extract edges from laser scan image
	    cv::Mat scan_edges;
	    cv::Canny(laser_image, scan_edges, 50, 150);

	    // Find feature points
	    std::vector<cv::Point2f> map_points, scan_points;
	    findFeaturePoints(map_edges, scan_edges, map_points, scan_points);

	    RCLCPP_INFO(this->get_logger(), "Number of map points: %lu", map_points.size());
	    RCLCPP_INFO(this->get_logger(), "Number of scan points: %lu", scan_points.size());

	    if (map_points.size() >= 3 && scan_points.size() >= 3) {
		// Calculate transformation matrix
		cv::Mat transformation_matrix = cv::estimateAffinePartial2D(scan_points, map_points);
		if (!transformation_matrix.empty()) {
		    // Extract rotation angle
		    double angle_x = transformation_matrix.at<double>(0, 0);
		    double angle_y = transformation_matrix.at<double>(1, 0);
		    double angle_radians = std::atan2(angle_y, angle_x);
		    double angle_degrees = angle_radians * (180.0 / M_PI);

		    RCLCPP_INFO(this->get_logger(), "Estimated Yaw Change: %.2f degrees", angle_degrees);

		    // Update localization status based on the estimated yaw change
		    updateLocalizationStatus(angle_degrees);
		} else {
		    RCLCPP_WARN(this->get_logger(), "Transformation matrix is empty.");
		}
	    } else {
		RCLCPP_WARN(this->get_logger(), "Not enough feature points to estimate transformation.");
	    }
	}


    void findFeaturePoints(const cv::Mat& img1, const cv::Mat& img2,
                           std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    bool isRobotLocalized() {
        // Determine if the robot is sufficiently localized
        // Example criteria: if the yaw change is less than a threshold
        double localization_threshold = 5.0; // degrees
        return std::fabs(current_yaw_ - estimated_yaw_) < localization_threshold;
    }

    void updateLocalizationStatus(double estimated_yaw) {
        // Update the current yaw based on the estimated yaw
        current_yaw_ = estimated_yaw;
        RCLCPP_INFO(this->get_logger(), "Updated current yaw: %.2f degrees", current_yaw_);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    cv::Mat map_image_, current_image_, previous_image_;
    geometry_msgs::msg::Pose current_pose_;
    bool first_image_captured_;
    bool robot_initialized_;
    std::string laser_scan_topic_;
    std::string odometry_topic_;
    double current_yaw_;
    double estimated_yaw_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}


