#pragma once

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"

enum class StateId {Idle, Searching, Tracking};

struct Context {
    rclcpp::Node* node = nullptr;
    bool human_visible = false;
    rclcpp::Time last_seen;
    vision_msgs::msg::BoundingBox2D last_box;
};

class StateMachine{
    public:
        void add_state(StateId id, std::unique_ptr<State> state){
            states_[id] = 
        }


};