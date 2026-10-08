#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>


#include "petting_zoo_navigation/arm_track_controller.hpp"

using namespace std::chrono_literals;

armTrackerController::armTrackerController()
: Node("arm_track_controller")
{
    //joint trajectory publisher
    joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/mirte_master_arm_controller/joint_trajectory",
        10
    );

    //arm target subscriber
    arm_target_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "arm_target", 10,
        std::bind(&armTrackerController::arm_target_callback, this, std::placeholders::_1)
    );

    //joint state subscriber
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&armTrackerController::joint_state_callback, this, std::placeholders::_1)
    );

    //bounding box subscriber
    human_bounding_box_subscriber_ = this->create_subscription<vision_msgs::msg::BoundingBox2D>(
        "human_bounding_box", 10,
        std::bind(&armTrackerController::human_bounding_box_callback, this, std::placeholders::_1)
    );

    //timer
    publish_timer_ = this->create_wall_timer(
        200ms,std::bind(&armTrackerController::publish_joint_states,this)
    );


}

void armTrackerController::publish_joint_states(){
    if(sends_left_ <= 0) return;
    if(joint_trajectory_publisher_->get_subscription_count() == 0) return;

    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_;
    point.time_from_start = rclcpp::Duration::from_seconds(move_time_);

    msg.points.push_back(point);
    joint_trajectory_publisher_->publish(msg);
    --sends_left_;
    RCLCPP_INFO(this->get_logger(), "Sent arm target");
}


void armTrackerController::arm_target_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg){
    if(msg->positions.size() != joint_names_.size()){
        RCLCPP_WARN(this->get_logger(), "arm_target needs %zu positions", joint_names_.size());
        return;
    }
    for(size_t i = 0; i < target_.size(); ++i){
        target_[i] = std::clamp(msg->positions[i], -M_PI_2, M_PI_2);
    }
    sends_left_ = 1;
}

void armTrackerController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    for(size_t i = 0; i < msg->name.size(); ++i){
        for(size_t j = 0; j < joint_names_.size(); ++j){
            if(msg->name[i] == joint_names_[j]) current_[j] = msg->position[i];
        }
    }
    have_joint_states_ = true;
}

void armTrackerController::human_bounding_box_callback(const vision_msgs::msg::BoundingBox2D::SharedPtr msg){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Got bounding box: centre (%.0f, %.0f), size %.0f x %.0f",
    msg->center.position.x, msg->center.position.y, msg->size_x, msg->size_y);

    
    if(!have_joint_states_) return;

    double error_x = msg->center.position.x - image_width_ / 2.0;
    double error_y = msg->center.position.y - image_height_ / 2.0;

    if(std::abs(error_x) < deadband_px_) error_x = 0.0;
    if(std::abs(error_y) < deadband_px_) error_y = 0.0;
    if(error_x == 0.0 && error_y == 0.0) return;

    target_ = current_;
    target_[0] = std::clamp(target_[0] - gain_ * error_x * rad_per_pixel_, -M_PI_2, M_PI_2);
    target_[3] = std::clamp(target_[3] - gain_ * error_y * rad_per_pixel_, -M_PI_2, M_PI_2);
    sends_left_ = 1;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Error (%.0f, %.0f) px -> pan %.2f, wrist %.2f",
    error_x, error_y, target_[0], target_[3]);

}



int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<armTrackerController>());
    rclcpp::shutdown();
    return 0;

}
