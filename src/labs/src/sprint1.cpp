#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor()
        : Node("laser_scan_processor")
    {
        // Subscribe to the TurtleBot3 laser scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        // Publisher for the new topic with every 9th point
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_nth_points", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Create a new LaserScan message for the output
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        int nth = 10;
        // Prepare the new ranges array by selecting every 9th point
        std::vector<float> new_ranges;
        for (size_t i = 0; i < scan->ranges.size(); i += nth)
        {
            new_ranges.push_back(scan->ranges[i]);
        }
        // Update the new LaserScan message with the modified ranges
        new_scan->ranges = new_ranges;
        // Adjust angle_increment to account for the reduced data points
        new_scan->angle_increment = scan->angle_increment * nth;
        // Log the number of points in the new scan
        RCLCPP_INFO(this->get_logger(), "Publishing a scan with %zu points.", new_scan->ranges.size());
        // Clear the intensities field if present
        new_scan->intensities.clear();
        // Publish the modified scan
        scan_pub_->publish(*new_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
