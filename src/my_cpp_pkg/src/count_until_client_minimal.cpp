#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;


// As you can see (but that shouldn’t be a surprise anymore), 
// we store a shared pointer to the action client. When initializing it, 
// we provide the action interface, the object to bind to (this), 
// and the action name, which should be the same as the one defined in the server code.

// In Python, we would chain the callbacks after sending the goal. 
// In C++, you first need to create a SendGoalOptions object. 
// In this object, you can register the different callback methods for your client. 
// Here, we register the response and the result callback. 
// Then, you must pass this object to the async_send_goal() method. 
// This will register all the callbacks for when the node is spinning.

class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    // Wait for action server, send a goal, and register callbacks for the response and the result
    void sendGoal(int target_number, double delay)
    {
        count_until_client_->wait_for_action_server();
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.delay = delay;
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.goal_response_callback = std::bind(&CountUntilClientNode::goalResponseCallback, this, _1);
        options.result_callback = std::bind(&CountUntilClientNode::goalResultCallback, this, _1);
        count_until_client_->async_send_goal(goal, options);
    }

private:
    // Get the goal response and print it
    void goalResponseCallback(const CountUntilGoalHandle::SharedPtr &goal_handle)
    {
        // check if goal was accepted or rejected
        // If this returns false, we know the goal was rejected. If it returns true, 
        // there’s no need to do anything else in this callback as 
        // the result callback was already registered with the SendGoalOptions object.
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    // Get the goal result and print it
    void goalResultCallback(const CountUntilGoalHandle::WrappedResult &result)
    {
        // get goals's final state
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }

        // this gets the actual result
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->sendGoal(5, 0.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
