#include "petting_zoo_fsm/states/state_tracking.hpp"
#include "petting_zoo_fsm/fsm_node.hpp"

TrackingState::TrackingState(FSMNode* fsm) : State(fsm) {}

void TrackingState::onEnter() {
    RCLCPP_INFO(fsm_->get_logger(), "Entering Tracking State");
}

void TrackingState::onExit() {
    RCLCPP_INFO(fsm_->get_logger(), "Exiting Tracking State");
}

void TrackingState::execute() {
    if ((fsm_->now() - fsm_->last_seen_).seconds() > 2.0) fsm_->switchState("searching");
}
