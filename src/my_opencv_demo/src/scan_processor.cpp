#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>  // For M_PI

class ScanProcessor : public rclcpp::Node
{
public:
  ScanProcessor()
  : Node("scan_processor")
  {
    // Subscriber to the TurtleBot3 scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ScanProcessor::scan_callback, this, std::placeholders::_1));

    // Publisher for the subset of laser scan data
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Define the angle range in degrees for the subset
    float start_angle_degrees = 0;
    float end_angle_degrees = 270;

    // Define the angle for which you want the range reading
    float desired_angle_degrees = 0;

    // Convert degrees to radians for angle calculations
    float start_angle_radians = start_angle_degrees * (M_PI / 180.0);
    float end_angle_radians = end_angle_degrees * (M_PI / 180.0);
    float desired_angle_radians = desired_angle_degrees * (M_PI / 180.0);

    // Calculate the index for the start, end, and desired angles
    int index_start = cal_index(start_angle_radians, msg->angle_min, msg->angle_increment);
    int index_end = cal_index(end_angle_radians, msg->angle_min, msg->angle_increment);
    int index_desired = cal_index(desired_angle_radians, msg->angle_min, msg->angle_increment);

    // Ensure indices are within bounds
    if (index_start >= 0 && index_start < msg->ranges.size() &&
        index_end >= 0 && index_end < msg->ranges.size() &&
        index_desired >= 0 && index_desired < msg->ranges.size()) {

      // Get the range at the desired angle
      float range_at_desired_angle = msg->ranges[index_desired];

      // Create a new LaserScan message for the subset
      auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>();

      // Copy original LaserScan data to the new message
      *new_scan = *msg;

      // Extract the range values within the specified angle range
      new_scan->ranges.clear();
      for (int i = index_start; i <= index_end; ++i) {
        new_scan->ranges.push_back(msg->ranges[i]);
      }

      // Update the angle_min and angle_max for the new scan message
      new_scan->angle_min = start_angle_radians;
      new_scan->angle_max = end_angle_radians;

      // Adjust the number of ranges
      new_scan->angle_increment = (end_angle_radians - start_angle_radians) / new_scan->ranges.size();

      // Clear the intensities field if present
      new_scan->intensities.clear();

      // Publish the new scan data
      scan_pub_->publish(*new_scan);

      // Display the range at the desired angle
      RCLCPP_INFO(this->get_logger(), "Range at angle %.2f degrees: %.2f meters", desired_angle_degrees, range_at_desired_angle);
      RCLCPP_INFO(this->get_logger(), "Published scan subset from %.2f° to %.2f°", start_angle_degrees, end_angle_degrees);
    } else {
      RCLCPP_WARN(this->get_logger(), "Desired angle or range is out of bounds.");
    }
  }

  int cal_index(float angle, float angle_min, float angle_increment)
  {
    // Calculate index based on angle
    return static_cast<int>((angle - angle_min) / angle_increment);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanProcessor>());
  rclcpp::shutdown();
  return 0;
}
