#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
/**
 * @brief Node for detecting cylinders using laser scan data.
 *
 * This node subscribes to laser scan data, detects cylindrical objects,
 * and publishes markers for visualization. It also modifies a map image
 * to indicate detected cylinders.
 */

class CylinderDetectionNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Cylinder Detection Node.
     *
     * Initializes the node, sets up subscribers and publishers.
     */
    CylinderDetectionNode() : Node("cylinder_detection_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetectionNode::laserCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);
    }

private:
    /**
     * @brief Callback function for processing laser scan data.
     *
     * @param scan The received laser scan message.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        float cylinder_radius = 0.15; // 15 cm radius
        int cylinder_detected = -1;
        geometry_msgs::msg::Point cylinder_position;

        // Analyze laser scan data to detect a cylindrical object
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            if (scan->ranges[i] < scan->range_max && scan->ranges[i] > scan->range_min)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                float x = scan->ranges[i] * cos(angle);
                float y = scan->ranges[i] * sin(angle);
                 // Check if the point is within the cylinder radius
                if (fabs(sqrt(x * x + y * y) - cylinder_radius) < 0.05)
                {
                    cylinder_position.x = x;
                    cylinder_position.y = y;
                    cylinder_position.z = 0.0; // Assume it's on the ground level
                    cylinder_detected = i;
                    break;
                }
            }
        }
        // If a cylinder is detected, get its global position and publish a marker
        if (cylinder_detected != -1)
        {
            try
            {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);

                tf2::Transform tf_map_to_base;
                tf2::fromMsg(transform_stamped.transform, tf_map_to_base);

                tf2::Vector3 local_position(cylinder_position.x, cylinder_position.y, 0);
                tf2::Vector3 global_position = tf_map_to_base * local_position;

                // Log the global position of the cylinder
                RCLCPP_INFO(this->get_logger(), "Global position of cylinder: (%f, %f, %f)", global_position.x(), global_position.y(), global_position.z());
                
                // Mark the cylinder on RVIZ
                publishCylinderMarker(global_position.x(), global_position.y(), global_position.z());
                
                // Draw the cylinder on the map
                drawCylinderOnMap("/home/student/ros2_ws/sprint3_v1_carto_map.pgm", "/home/student/ros2_ws/sprint3_cylinder_map.pgm", global_position.x(), global_position.y());

            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            }
        }
    }
    /**
     * @brief Publish a marker for the detected cylinder.
     *
     * @param x X coordinate of the cylinder.
     * @param y Y coordinate of the cylinder.
     * @param z Z coordinate of the cylinder.
     */
    void publishCylinderMarker(float x, float y, float z)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom"; // Global frame
        marker.header.stamp = this->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker (cylinder)
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;  // Assuming cylinder is 1m tall
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker (diameter and height)
        marker.scale.x = 0.30; // 30cm diameter
        marker.scale.y = 0.30; // 30cm diameter
        marker.scale.z = 1.0;  // 1m height

        // Set the color (red)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Lifetime of the marker (set to 0 for permanent)
        marker.lifetime = rclcpp::Duration(std::chrono::seconds(0));

        marker_pub_->publish(marker);
    }

    
    /**
     * @brief Draw a cylinder on the map image.
     *
     * @param original_map_path Path to the original map image.
     * @param new_map_path Path to save the modified map image.
     * @param cylinder_x X coordinate of the cylinder in global coordinates.
     * @param cylinder_y Y coordinate of the cylinder in global coordinates.
     */
    void drawCylinderOnMap(const std::string& original_map_path, const std::string& new_map_path, float cylinder_x, float cylinder_y)
    {
        // Load the original map
        cv::Mat map_image = cv::imread(original_map_path, cv::IMREAD_UNCHANGED);
        if (map_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the map image!");
            return;
        }

        // Map origin (from /home/student/ros2_ws/sprint3_v1_carto_map.yaml)
        float origin_x = -11.3;  // From your YAML file
        float origin_y = -5.4;   // From your YAML file
        float map_resolution = 0.1;  // 10 cm per pixel, from your YAML file

        // Calculate pixel coordinates for the detected cylinder (global coordinates to map pixels)
        int cylinder_pixel_x = static_cast<int>((cylinder_x - origin_x) / 0.09);
        int cylinder_pixel_y = static_cast<int>((cylinder_y - origin_y) / 0.032);

        // Correct Y-coordinate for OpenCV (invert Y-axis)
        int corrected_cylinder_pixel_y = map_image.rows - cylinder_pixel_y;

        // Log the global and pixel positions
        RCLCPP_INFO(this->get_logger(), "Cylinder global position: (%f, %f)", cylinder_x, cylinder_y);
        RCLCPP_INFO(this->get_logger(), "Cylinder pixel position: (%d, %d)", cylinder_pixel_x, corrected_cylinder_pixel_y);

        // Draw the detected cylinder if within map bounds
        int radius = static_cast<int>(0.3/ 0.05);  // Radius of 0.3m converted to pixels
        if (cylinder_pixel_x >= 0 && cylinder_pixel_x < map_image.cols &&
            corrected_cylinder_pixel_y >= 0 && corrected_cylinder_pixel_y < map_image.rows)
        {
            cv::circle(map_image, cv::Point((cylinder_pixel_x), corrected_cylinder_pixel_y), radius, cv::Scalar(0, 255, 0), -1); // Green filled circle
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cylinder position is out of map bounds.");
        }

        // Draw a square at the origin for reference
        int origin_pixel_x = static_cast<int>((origin_x - origin_x) / map_resolution); // This should be 0
        int origin_pixel_y = static_cast<int>((origin_y - origin_y) / map_resolution); // This should also be 0
        // Log the global position of the square
        RCLCPP_INFO(this->get_logger(), "Square global position: (%f, %f)", origin_x, origin_y);
        // Define the size of the square in pixels
        int square_size = 10; // Size of the square in pixels

        // Calculate top-left corner of the square
        int square_top_left_x = origin_pixel_x; // Should be 0
        int square_top_left_y = map_image.rows - origin_pixel_y - square_size; // Correct for OpenCV's y-axis

        // Draw the square
        cv::rectangle(map_image, 
                    cv::Point(square_top_left_x, square_top_left_y), 
                    cv::Point(square_top_left_x + square_size, square_top_left_y + square_size), 
                    cv::Scalar(128),
                    -1);
        RCLCPP_INFO(this->get_logger(), "Square top left: (%d, %d)", square_top_left_x , square_top_left_y);

        // Save the modified map
        cv::imwrite(new_map_path, map_image);


        //isplay the new map to verify
        cv::imshow("Cylinder Map", map_image);
        cv::waitKey(0); // Wait for a key press to close the window
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
