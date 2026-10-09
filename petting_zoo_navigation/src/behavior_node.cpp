#include "petting_zoo_navigation/state_machine/state_machine.hpp"
#include "petting_zoo_navigation/state_machine/states.hpp"

class BehaviorNode : public rclcpp::Node {
    public:
        BehaviorNode() : Node("behavior_node"){
            ctx_.node = this;
            ctx_.last_seen = now();

            sm_.add_state(StateId::Searching, std::make_unique<SearchingState>());
            sm_.add_state(StateId::Tracking, std::make_unique<TrackingState>());

            box_sub_ = create_subscription<vision_msgs::msg::BoundingBox2D>(
                
            )

        }
};