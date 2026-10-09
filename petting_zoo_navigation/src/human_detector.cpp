#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"

#include "petting_zoo_navigation/human_detector.hpp"


humanDetector::humanDetector()
: Node("human_detector")
{
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    std::string image_topic = this->declare_parameter<std::string>("image_topic", "camera/image_raw");

    //bounding box publisher
    human_bounding_box_publisher_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "detections/humans",
        10
    );

    //camera image subscriber
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&humanDetector::image_callback, this, std::placeholders::_1)
    );

    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "human_detector/image",
        10
    );



}

void humanDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    cv_bridge::CvImagePtr frame;
    try{
        frame = cv_bridge::toCvCopy(msg, "bgr8");
    }catch(const cv_bridge::Exception & e){
        RCLCPP_WARN(this->get_logger(), "Could not convert image: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Rect> humans;
    hog_.detectMultiScale(gray, humans, 0.0, cv::Size(8,8), cv::Size(), 1.05, 2.0);

    if(!humans.empty()){
        //keep the biggest detection, it is the closest human
        cv::Rect biggest = humans[0];
        for(const cv::Rect & human : humans){
            if(human.area() > biggest.area()) biggest = human;
        }

        vision_msgs::msg::BoundingBox2D box;
        box.center.position.x = biggest.x + biggest.width / 2.0;
        box.center.position.y = biggest.y + biggest.height / 2.0;
        box.size_x = biggest.width;
        box.size_y = biggest.height;
        human_bounding_box_publisher_->publish(box);

        cv::rectangle(frame->image, biggest, cv::Scalar(0, 255, 0), 2);
    }

    debug_image_publisher_->publish(*frame->toImageMsg());
}





int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<humanDetector>());
    rclcpp::shutdown();
    return 0;

}
