#include "petting_zoo_fsm/fsm_node.hpp"
#include "petting_zoo_fsm/states/state_searching.hpp"
#include "petting_zoo_fsm/states/state_tracking.hpp"

FSMNode::FSMNode() : Node("fsm_node"){
    last_seen_ = now();
    ignore_humans_until_ = now();
    state_entered_ = now();

    auto latched = rclcpp::QoS(1).transient_local();
    walk_enable_pub_ = create_publisher<std_msgs::msg::Bool>("walking_around/enable", latched);
    arm_enable_pub_  = create_publisher<std_msgs::msg::Bool>("arm_track_controller/enable", latched);

    //States
    addState<SearchingState>("searching");
    addState<TrackingState>("tracking");

    box_sub_ = create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "detections/humans",10,
        [this](vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){
            if(msg->boxes.empty()) return;   //detector also publishes empty arrays
            last_box_ = msg->boxes[0];
            human_visible_ = true;
            last_seen_ = now();
        }
    );

    switchState("searching");
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
}

void FSMNode::switchState(const std::string& name){
    if(states_.find(name) == states_.end()){
        RCLCPP_ERROR(get_logger(), "Unknown state '%s'", name.c_str());
        return;
    }
    pending_ = name;
}

void FSMNode::tick(){
    if (!pending_.empty()){
        if(current_) current_->onExit();
        current_ = states_.at(pending_).get();
        current_name_ = pending_;
        pending_.clear();
        state_entered_ = now();
        current_->onEnter();
    }
    if(current_) current_->execute();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FSMNode>());
    rclcpp::shutdown();
    return 0;
}