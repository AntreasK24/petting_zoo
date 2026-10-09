#include "petting_zoo_fsm/fsm_node.hpp"
#include "petting_zoo_fsm/states/state_searching.hpp"
#include "petting_zoo_fsm/states/state_tracking.hpp"

FSMNode::FSMNode() : Node("fsm_node"){
    last_seen_ = now();

    //States
    addState<SearchingState>("searching");
    addState<TrackingState>("tracking");

    box_sub_ = create_subscription<vision_msgs::msg::BoundingBox2D>(
        "human_bounding_box",10,
        [this](vision_msgs::msg::BoundingBox2D::SharedPtr msg){
            last_box_ = *msg;
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