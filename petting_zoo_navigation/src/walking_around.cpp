#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "petting_zoo_navigation/walking_around.hpp"


using namespace std::chrono_literals;

randomVelocityPublisher::randomVelocityPublisher()
: Node("walking_around")
{
    random_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "mirte_base_controller/cmd_vel_unstamped",
        10);

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&randomVelocityPublisher::scan_callback, this, std::placeholders::_1));
    
    enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "walking_around/enable", rclcpp::QoS(1).transient_local(),
        std::bind(&randomVelocityPublisher::enable_callback, this, std::placeholders::_1));

    pick_timer_ = this->create_wall_timer(
        2s,std::bind(&randomVelocityPublisher::pick_new_velocity,this)
    );

    publish_timer_ = this->create_wall_timer(
        50ms,std::bind(&randomVelocityPublisher::publish_velocity,this)
    );
    
    pick_new_velocity();
}

void randomVelocityPublisher::pick_new_velocity(){
    current_cmd_.linear.x  = 0.2; //lin_dist_(rng_);
    //current_cmd_.linear.y  = lin_dist_(rng_);
    current_cmd_.angular.z = ang_dist_(rng_);
}

void randomVelocityPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    double closest = std::numeric_limits<double>::infinity();
    for(size_t i = 0; i < msg->ranges.size(); ++i){
        double r = msg->ranges[i];
        if(!std::isfinite(r) || r < 0.25 ||  r > msg->range_max ) continue;
        double angle = msg->angle_min + i * msg->angle_increment - front_offset_;
        angle = std::atan2(std::sin(angle), std::cos(angle));
        if (std::abs(angle) < 0.5) closest = std::min(closest, r);
    }
    front_dist_ = closest;
}

void randomVelocityPublisher::enable_callback(const std_msgs::msg::Bool::SharedPtr msg){
    enabled_ = msg->data;
    if (!enabled_) random_velocity_publisher_->publish(geometry_msgs::msg::Twist());  // stop now
}

void randomVelocityPublisher::publish_velocity(){
    if(!enabled_) return;

    auto cmd = current_cmd_;
    if (front_dist_ < 0.5) {
        cmd.linear.x  = 0.0;
        cmd.angular.z = turn_dir_ * 0.8;
    }
    random_velocity_publisher_->publish(cmd);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<randomVelocityPublisher>());
    rclcpp::shutdown();
    return 0;

}
