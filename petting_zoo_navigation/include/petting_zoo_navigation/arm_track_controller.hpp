#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "std_msgs/msg/bool.hpp"




class armTrackerController : public rclcpp::Node{
    public:
        armTrackerController();
    private:

        void publish_joint_states();
        void arm_target_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void human_bounding_box_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg);
        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);


        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr human_bounding_box_subscriber_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr arm_target_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_;

        std::vector<std::string> joint_names_{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};
        std::vector<double> start_pose_{0.0, 0.0, -1.5, 0.0};
        std::vector<double> target_ = start_pose_;
        std::vector<double> current_ = start_pose_;
        bool have_joint_states_ = false;
        bool at_start_ = false;

        int sends_left_ = 3;
        double move_time_ = 0.2;

        double image_width_ = 640.0;
        double image_height_ = 480.0;
        double rad_per_pixel_ = 1.047/640.0;
        double gain_ = 0.7;
        double deadband_px_ = 15.0;
        bool reached(const std::vector<double>& pose, double tol = 0.05) const;
        bool enabled_ = false;
};
