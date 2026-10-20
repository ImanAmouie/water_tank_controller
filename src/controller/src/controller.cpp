#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "water_tank_interface/msg/tank_input.hpp"

using TankInput = water_tank_interface::msg::TankInput;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
        : Node("controller")
    {
        publisher_ = this->create_publisher<TankInput>(
            "/inlet_valve",
            10);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500),
            std::bind(&ControllerNode::timer_callback, this));
        RCLCPP_INFO(get_logger(), "Controller node Ready !!!");
    }

private:
    void timer_callback()
    {
        tank_input_.inlet_valve = 30.0; // sample output 
        publisher_->publish(tank_input_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TankInput>::SharedPtr publisher_;

    TankInput tank_input_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}