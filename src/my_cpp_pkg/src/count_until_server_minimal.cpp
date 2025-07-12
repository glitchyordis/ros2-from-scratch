#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;


// the C++ syntax is stricter than Python for actions
// you have to provide three callbacks (even if you don’t want to use them all)
// Goal callback, Cancel callback, and Execute callback
// Excecute callback is called the handle accepted callback in C++, but we named it execute callback to make the code similar to the Python one. 
// In this callback, we execute goals that have been accepted.
// this is intednded to be a minimal example of action server in C++
// the full mehtod to handle cancel mechanism is implemented later
class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("count_until_server")
    {
        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goalCallback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancelCallback, this, _1),
            std::bind(&CountUntilServerNode::executeCallback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started.");
    }

// you get a unique identifier for the goal and the goal itself (to be precise, this is a const shared pointer to the goal).
private:
    // Every new received goal will be processed here first
    // We can decide to accept or reject the incoming goal
    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal");
        if (goal->target_number <= 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting the goal, target number must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Any cancel request will be processed here, we can accept or reject it
    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // If a goal has been accepted, it will then be executed in this callback
    // After we are done with the goal execution we set a final state and provide the result
    // You’ve probably started to get used to seeing shared pointers everywhere, and here is no exception. 
    // We don’t create a result object, but a shared pointer to a result object.
    // In Python, we would set the goal’s final state first, and then return the result. In C++, we don’t return anything (note the void return type). 
    // We send the result at the same time as setting the goal state.
    void executeCallback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        int target_number = goal_handle->get_goal()->target_number;
        double delay = goal_handle->get_goal()->delay;
        auto result = std::make_shared<CountUntil::Result>();
        int counter = 0;

        // to handle the waiting time between each count iteration, we use a rclcpp::Rate object
        // In this rate object, we have to pass the rate—that is, the frequency we want for the loop. For example, if the delay is 0.5 seconds, the frequency would be 2.0 Hz.
        rclcpp::Rate loop_rate(1.0/delay);

        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        for (int i = 0; i < target_number; i++) {
            counter++;
            RCLCPP_INFO(this->get_logger(), "%d", counter);
            loop_rate.sleep();
        }

        result->reached_number = counter;
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}