#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "SessionManager.h"
#include "RouterClient.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"

// Assuming KortexMathUtil is included or you use basic math
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace k_api = Kinova::Api;

class KinovaJSPublisher : public rclcpp::Node
{
public:
  KinovaJSPublisher(std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic) 
  : Node("kinova_js_publisher"), base_cyclic_(base_cyclic)
  {
    // 1. Strongly-typed ROS 2 Parameters
    this->declare_parameter<std::string>("gripper_type", "2f85");
    std::string gripper_type = this->get_parameter("gripper_type").as_string();

    // 2. Setup standard arm joints
    arm_joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    // 3. Setup Gripper variables based on the parameter
    if (gripper_type == "2f140") {
        gripper_max_pos_ = 0.7;
        gripper_joint_names_ = {
            "finger_joint", "right_outer_knuckle_joint", "left_inner_knuckle_joint",
            "right_inner_knuckle_joint", "left_inner_finger_joint", "right_inner_finger_joint"
        };
    } else {
        gripper_max_pos_ = 0.8;
        gripper_joint_names_ = {
            "finger_joint", "right_knuckle_joint", "left_inner_knuckle_joint",
            "right_inner_knuckle_joint", "left_finger_tip_joint", "right_finger_tip_joint"
        };
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&KinovaJSPublisher::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "UDP Publisher Ready (Gripper: %s)", gripper_type.c_str());
  }

private:
  void timer_callback()
  {

    k_api::BaseCyclic::Feedback feedback;
    
    try {
        // 2. Attempt to pull the UDP data
        feedback = base_cyclic_->RefreshFeedback();
    } catch (const std::exception& e) {
        return; 
    }

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();

    // --- ARM JOINTS ---
    for (int i = 0; i < 6; ++i) {
      message.name.push_back(arm_joint_names_[i]);
      
      double rad_val = feedback.actuators(i).position() * M_PI / 180.0;
      
      // Translating your exact Python modulo logic for continuous joints
      if (i != 0 && i != 3 && i != 5) {
          rad_val = std::fmod(rad_val + M_PI, 2.0 * M_PI);
          if (rad_val < 0) rad_val += 2.0 * M_PI; // C++ fmod can return negative
          rad_val -= M_PI;
      }

      message.position.push_back(rad_val);
      message.velocity.push_back(feedback.actuators(i).velocity() * M_PI / 180.0);
      message.effort.push_back(feedback.actuators(i).current_motor());
    }

    // --- GRIPPER JOINTS ---
    float gripper_motor = 0.0;
    
    // Check if the interconnect has motor data (C++ alternative to try/except)
    if (feedback.interconnect().gripper_feedback().motor_size() > 0) {
       // Note the parentheses (): Protobuf in C++ uses getter functions, not properties!
       gripper_motor = feedback.interconnect().gripper_feedback().motor(0).position();
    }
    
    float gripper_pos = (gripper_motor / 100.0) * gripper_max_pos_;

    // Create the 6 mimic positions exactly like the Python array
    std::vector<float> gripper_positions = {
        gripper_pos, -gripper_pos, gripper_pos, -gripper_pos, -gripper_pos, gripper_pos
    };

    // Append them to the message
    for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
        message.name.push_back(gripper_joint_names_[i]);
        message.position.push_back(gripper_positions[i]);
        message.velocity.push_back(0.0); // Grippers usually don't need high-speed mimic velocity
        message.effort.push_back(0.0);
    }

    publisher_->publish(message);
  }

  // --- CLASS MEMBERS ---
  std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Store our parameters and arrays so we don't recreate them every millisecond
  float gripper_max_pos_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string ip_address = "192.168.50.9";

  // A. TCP Setup (For Authentication / Session Management)
  auto transport_tcp = new k_api::TransportClientTcp();
  auto router_tcp = new k_api::RouterClient(transport_tcp, [](k_api::KError){});
  transport_tcp->connect(ip_address, 10000);

  auto session_manager = new k_api::SessionManager(router_tcp);
  auto session_info = k_api::Session::CreateSessionInfo();
  session_info.set_username("admin");
  session_info.set_password("admin");
  session_manager->CreateSession(session_info);

  // B. UDP Setup (For Real-Time Cyclic Data)
  auto transport_udp = new k_api::TransportClientUdp();
  auto router_udp = new k_api::RouterClient(transport_udp, [](k_api::KError){});
  transport_udp->connect(ip_address, 10001);

  // FIX: C++ requires a dedicated SessionManager for the UDP router!
  auto session_manager_udp = new k_api::SessionManager(router_udp);
  session_manager_udp->CreateSession(session_info);

  // C. Create the BaseCyclic client using the UDP router
  auto base_cyclic_client = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(router_udp);

  // Instantiate and spin the node
  auto js_publisher = std::make_shared<KinovaJSPublisher>(base_cyclic_client);
  rclcpp::spin(js_publisher);

  // Clean up both sessions!
  session_manager_udp->CloseSession();
  session_manager->CloseSession();
  transport_tcp->disconnect();
  transport_udp->disconnect();
  
  rclcpp::shutdown();
  return 0;
}