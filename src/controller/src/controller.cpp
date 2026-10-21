#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"

using TankInput = water_tank_interface::msg::TankInput;
using TankOutput = water_tank_interface::msg::TankOutput;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
        : Node("controller")
    {
        publisher_ = this->create_publisher<TankInput>(
            "/inlet_valve",
            10);

        sub_tank_output_ = this->create_subscription<TankOutput>(
            "/water_level",
            10,
            std::bind(&ControllerNode::tank_output_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(100),
            std::bind(&ControllerNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Controller node Ready !!!");
    }

private:

    float water_level {0};
    float error {0};
    float sum_error {0};
    const float d_t {0.1};
    float prev_error {0};
    float k_p {90.0};
    float k_I {0.00005};
    float k_D {0.0};

    void timer_callback()
    {
        tank_input_.inlet_valve = pid(4.15); // sample output
        publisher_->publish(tank_input_);
    }

    void tank_output_callback(const TankOutput::SharedPtr msg)
    {
        water_level = msg->level;
        // RCLCPP_INFO(get_logger(), "level -> %f", msg->level);
    }

    float pid(float referense){

        error = referense - water_level;
        sum_error += error * d_t;
        float derivate = error / d_t;





    RCLCPP_INFO(get_logger(), "sum error -> %f error each time -> %f", sum_error, error * d_t);

        prev_error = error;

        return (k_p * error) + (k_I * sum_error) + (derivate * k_D);

}

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TankInput>::SharedPtr publisher_;
    rclcpp::Subscription<TankOutput>::SharedPtr sub_tank_output_;

    TankInput tank_input_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}