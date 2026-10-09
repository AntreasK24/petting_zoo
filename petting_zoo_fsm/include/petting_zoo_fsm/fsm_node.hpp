#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "petting_zoo_fsm/state.hpp"

class FSMNode : public rclcpp::Node {
    public:
        FSMNode();
        void switchState(const std::string& name);

        bool human_visible_ = false;
        rclcpp::Time last_seen_;
        vision_msgs::msg::BoundingBox2D last_box_;

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
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr box_sub_;
};