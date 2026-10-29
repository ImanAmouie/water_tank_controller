#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <chrono>
#include <algorithm>
#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"
using namespace std::chrono_literals;
class Controller_Node : public rclcpp::Node{
  public:
    Controller_Node(): Node("controller_node"){
        RCLCPP_INFO(this->get_logger(),"Controller node is working.");
        inlet_valve_publisher = this->create_publisher<water_tank_interface::msg::TankInput>("/inlet_valve",10);
        timer_ = this->create_wall_timer(10ms, std::bind(&Controller_Node::timer_callback, this));
        start_ = std::chrono::steady_clock::now();
        water_level_subscriber = this->create_subscription<water_tank_interface::msg::TankOutput>("/water_level", 10, std::bind(&Controller_Node::h_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback(){
        inlet_valve_publisher->publish(valve);
    }
    void h_callback(const water_tank_interface::msg::TankOutput::SharedPtr h){
      auto now_ = std::chrono::steady_clock::now();
      float dt = std::chrono::duration<float>(now_ - start_).count();
      // if (dt <= 0.0f) {
      //     return;
      // }
      float error = 4.5f - h->level;
      p_term = k_p * error;
      derivative = (error - prev_error) / dt;
      float alpha = tau / (tau + dt);
      filtered_derivative =alpha * filtered_derivative+ (1.0f - alpha) * derivative;
      d_term = k_d * filtered_derivative;
      float prop_integral = integral + error * dt;
      float prop_output = p_term + k_i * prop_integral + d_term;
      if (!((prop_output > 100.0f && error > 0.0f) ||(prop_output < 0.0f && error < 0.0f))){
        integral = prop_integral;
      }
      i_term = k_i * integral;
      output = p_term + i_term + d_term;
      valve.inlet_valve = std::clamp(output, 0.0f, 100.0f);

      RCLCPP_INFO(this->get_logger(),"P: %f / I: %f / D: %f / Output: %f / Error: %f / Valve: %f / dt: %f",p_term,i_term,d_term,output,error,valve.inlet_valve,dt);
      prev_error = error;
      start_ = now_;
  }
    float k_p{300.0f} , k_i{64.0f} , k_d{0.0f};
    float derivative {0.0f} , integral{0.0f};
    float p_term {0.0f}, i_term{0.0f}, d_term{0.0f}; 
    float filtered_derivative{0.0f};
    float tau{0.2f};  
    float prev_error{0.0f};
    float output{0.0};
    std::chrono::steady_clock::time_point start_ ;
    rclcpp::TimerBase::SharedPtr timer_;
    water_tank_interface::msg::TankInput valve;
    rclcpp::Publisher<water_tank_interface::msg::TankInput>::SharedPtr inlet_valve_publisher;
    rclcpp::Subscription<water_tank_interface::msg::TankOutput>::SharedPtr water_level_subscriber;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller_Node>());
  rclcpp::shutdown();
  return 0;
}