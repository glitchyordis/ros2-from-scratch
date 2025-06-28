#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        number_subscriber_ = 
        // for _1 to work we need `using namespace std::placeholders;` before it
        this->create_subscription<example_interfaces::msg::Int64>(
                "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1));
            
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        // The message we receive in the callback is also a (const) shared pointer. 
        //Hence, don’t forget to use -> when accessing the data field.
        counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
    }

    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}