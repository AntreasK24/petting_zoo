#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "petting_zoo_fsm/state.hpp"


class FSMNode : public rclcpp::Node {
    public:
        FSMNode();
        void switchState(const std::string& name);

        bool human_visible_ = false;
        rclcpp::Time last_seen_;
        vision_msgs::msg::BoundingBox2D last_box_;

        rclcpp::Time state_entered_;
        rclcpp::Time ignore_humans_until_;

        double lost_timeout_ = 10.0;
        double max_track_time_ = 30.0;
        double cooldown_ = 10.0;

        void setWalking(bool on) {std_msgs::msg::Bool m; m.data = on; walk_enable_pub_->publish(m);}
        void setTracking(bool on) {std_msgs::msg::Bool m; m.data = on; arm_enable_pub_->publish(m);}

    private:
        void tick();

        template <typename T>
        void addState(const std::string& name){
            states_[name] = std::make_unique<T>(this);
        }

        std::unordered_map<std::string, std::unique_ptr<State>> states_;
        State* current_ = nullptr;
        std::string current_name_;
        std::string pending_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr box_sub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr walk_enable_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;
};