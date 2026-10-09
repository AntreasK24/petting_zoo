#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"



class armTrackerController : public rclcpp::Node{
    public:
        armTrackerController();
    private:

        void publish_joint_states();
        void arm_target_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void human_bounding_box_callback(const vision_msgs::msg::BoundingBox2D::SharedPtr msg);


        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr human_bounding_box_subscriber_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr arm_target_subscriber_;

        std::vector<std::string> joint_names_{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};
        std::vector<double> target_{0.0, 0.0, -1.5, 0.0};
        std::vector<double> current_{0.0, 0.0, -1.5, 0.0};
        bool have_joint_states_ = false;

        int sends_left_ = 3;
        double move_time_ = 0.2;

        double image_width_ = 640.0;
        double image_height_ = 480.0;
        double rad_per_pixel_ = 1.047/640.0;
        double gain_ = 0.7;
        double deadband_px_ = 15.0;

};
