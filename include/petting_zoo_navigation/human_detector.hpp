#pragma once

#include <opencv2/objdetect.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"


class humanDetector : public rclcpp::Node{
    public:
        humanDetector();
    private:

        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr human_bounding_box_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;


        //OpenCV built-in pedestrian detector
        cv::HOGDescriptor hog_;

};
