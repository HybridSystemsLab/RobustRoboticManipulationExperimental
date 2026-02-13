#include <onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  onrobot_rg2ft_action_server::GripperParams params;
  // These defaults can be overridden via launch parameters
  params.min_angle_ = 0.0;
  params.max_angle_ = 0.93;
  params.min_effort_ = 3.0;
  params.max_effort_ = 40.0;
  params.default_effort_ = 10.0;
  params.control_topic_ = "gripper/ctrl";
  params.state_topic_ = "gripper/states";
  params.joint_states_topic_ = "joint_states";
  params.joint_name_ = "left_outer_knuckle_joint";

  auto node = std::make_shared<onrobot_rg2ft_action_server::OnRobotRG2FTActionServer>(
    "onrobot_rg2ft_action_server", params);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
