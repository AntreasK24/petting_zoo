#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class randomVelocityPublisher : public rclcpp::Node{
    public:
    randomVelocityPublisher()
    : Node("walking_around")
    {
        random_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("mirte_base_controller/cmd_vel",10);
        pick_timer_ = this->create_wall_timer(
            2s,std::bind(&randomVelocityPublisher::pick_new_velocity,this)
        );

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&randomVelocityPublisher::scan_callback, this, std::placeholders::_1));

        publish_timer_ = this->create_wall_timer(
            50ms,std::bind(&randomVelocityPublisher::publish_velocity,this)
        );


        pick_new_velocity();


    }

    private:

        void pick_new_velocity(){
            current_cmd_.linear.x  = 0.2; //lin_dist_(rng_);
            //current_cmd_.linear.y  = lin_dist_(rng_);
            current_cmd_.angular.z = ang_dist_(rng_);
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
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

        void publish_velocity(){
            auto cmd = current_cmd_;
            if (front_dist_ < 0.5) {        
                cmd.linear.x  = 0.0;
                cmd.angular.z = turn_dir_ * 0.8;
            }
            random_velocity_publisher_->publish(cmd);
        }


    //Random walk
    std::mt19937 rng_{std::random_device{}()};
    std::uniform_real_distribution<double> lin_dist_{-0.3,0.3};
    std::uniform_real_distribution<double> ang_dist_{-1.0,1.0};
    

    //Ros Members
    rclcpp::TimerBase::SharedPtr pick_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr random_velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    geometry_msgs::msg::Twist current_cmd_;
    double front_dist_ = std::numeric_limits<double>::infinity();
    double front_offset_ = 0.0;
    double turn_dir_ = 1;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<randomVelocityPublisher>());
    rclcpp::shutdown();
    return 0;

}