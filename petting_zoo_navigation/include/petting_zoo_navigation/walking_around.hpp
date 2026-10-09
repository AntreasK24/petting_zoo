#pragma once

#include <random>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"



class randomVelocityPublisher : public rclcpp::Node{
    public:

        randomVelocityPublisher();

    private:

        void pick_new_velocity();
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void publish_velocity();

        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

        //Random walk
        std::mt19937 rng_{std::random_device{}()};
        std::uniform_real_distribution<double> lin_dist_{-0.3,0.3};
        std::uniform_real_distribution<double> ang_dist_{-1.0,1.0};
    

        //Ros Members
        rclcpp::TimerBase::SharedPtr pick_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr random_velocity_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_; 

        geometry_msgs::msg::Twist current_cmd_;

        double front_dist_ = std::numeric_limits<double>::infinity();
        double front_offset_ = 0.0;
        double turn_dir_ = 1;

        bool enabled_ = false;
};