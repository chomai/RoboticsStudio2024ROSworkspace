#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Load images
    cv::Mat img1 = cv::imread("/home/student/ros2_ws/sprint3_v1_carto_map.pgm", cv::IMREAD_UNCHANGED);
    cv::Mat img2 = cv::imread("/home/student/robot_ws/map.pgm", cv::IMREAD_UNCHANGED);
    
    cv::imshow("Ground truth map", img2);
    
    cv::imshow("SLAM map", img1);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Could not open or find the images!" << std::endl;
        return -1;
    }

    // Calculate new output image size (larger to accommodate shift)
    int outputWidth = std::max(img1.cols, img2.cols) + 200;  // Add 100 pixels for shift space
    int outputHeight = std::max(img1.rows, img2.rows) + 20;

    // Create an output image initialized to zero (black)
    cv::Mat output = cv::Mat::zeros(outputHeight, outputWidth, img1.type());

    // Calculate the center positions for img1
    int offsetX1 = (outputWidth - img1.cols) / 2; // Center img1 horizontally
    int offsetY1 = (outputHeight - img1.rows) / 2; // Center img1 vertically

    // Shift img2 to the left by 50 pixels
    int offsetX2 = (outputWidth - img2.cols) / 2 - 40; // Shift img2 left by 50 pixels
    int offsetY2 = (outputHeight - img2.rows) / 2 + 5; // Center img2 vertically

    // Ensure img2 stays within bounds
    offsetX2 = std::max(0, offsetX2);

    // Overlay img1 at the center of the output image
    img1.copyTo(output(cv::Rect(offsetX1, offsetY1, img1.cols, img1.rows)));

    // Add transparency (50% blend for both images)
    double alpha = 0.5; // Transparency factor for img1
    double beta = 1.0 - alpha; // Transparency factor for img2

    // Overlay img2 on top with transparency and the left shift
    cv::Mat roi = output(cv::Rect(offsetX2, offsetY2, img2.cols, img2.rows));
    cv::addWeighted(roi, alpha, img2, beta, 0.0, roi);

    // Display the overlayed image with transparency
    cv::imshow("Overlay with Transparency and Shift", output);
    cv::waitKey(0);
    
    

    return 0;
}
