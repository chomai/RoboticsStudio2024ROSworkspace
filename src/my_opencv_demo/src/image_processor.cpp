#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor()
  : Node("image_processor")
  {
    // Subscriber to the TurtleBot3 camera image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

    // Publisher for the modified image
    image_pub_ = image_transport::create_publisher(this, "/image_processed");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Get the center of the image
    int center_x = cv_ptr->image.cols / 2;
    int center_y = cv_ptr->image.rows / 2;

    // Draw a circle at the center of the image
    cv::circle(cv_ptr->image, cv::Point(center_x, center_y), 50, CV_RGB(255, 0, 0), 3);

    // Convert the modified OpenCV image back to ROS Image message and publish
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::Publisher image_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
