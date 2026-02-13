/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a DH Gripper
 * code modified from https://github.com/jr-robotics/robotiq/tree/noetic-devel/robotiq_2f_gripper_action_server
 */

#ifndef ONROBOT_RG2FT_ACTION_SERVER_H
#define ONROBOT_RG2FT_ACTION_SERVER_H

// STL
#include <string>
// ROS standard
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <onrobot_rg2ft_msgs/msg/rg2_ft_command.hpp>
#include "onrobot_rg2ft_msgs/msg/rg2_ft_state.hpp"
#include <mutex>
// Repo specific includes (ROS 2 interfaces)

namespace onrobot_rg2ft_action_server
{

// DH gripper control parameters
#define MIN_POSITION 0
#define MAX_POSITION 1000
#define MIN_FORCE 30
#define MAX_FORCE 400
#define GOAL_TOLERANCE 20

using GripperCtrl = onrobot_rg2ft_msgs::msg::RG2FTCommand;
using GripperState = onrobot_rg2ft_msgs::msg::RG2FTState;

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

struct BadArgumentsError {};

/**
 * @brief Structure containing the parameters necessary to translate
 *        GripperCommand actions to register-based commands to a
 *        particular gripper (and vice versa).
 *
 *        The min gap can be less than zero. This represents the case where the 
 *        gripper fingers close and then push forward.
 */
struct GripperParams
{
  double min_angle_; // radians
  double max_angle_;
  double min_effort_; // N / (Nm) ???
  double max_effort_;
  double default_effort_;
  std::string control_topic_;
  std::string state_topic_;
  std::string joint_states_topic_;
  std::string joint_name_;
};

/**
 * @brief The DHGripperActionServer class. Takes as arguments the name of the gripper it is to command,
 *        and a set of parameters that define the physical characteristics of the particular gripper.
 *        
 *        Listens for messages on input and publishes on output. Remap these.
 */
class OnRobotRG2FTActionServer : public rclcpp::Node
{
public:
  OnRobotRG2FTActionServer(const std::string & name, const GripperParams & params);

private:
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Subscription<GripperState>::SharedPtr state_sub_;
  rclcpp::Publisher<GripperCtrl>::SharedPtr goal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  std::atomic<int> position_goal;
  std::atomic<int> force_goal;
  std::atomic<bool> is_initialized;

  GripperParams gripper_params_;
  std::string action_name_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperCommand::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);

  GripperCtrl goalToGripperCtrl(const GripperCommand::Goal & goal);
  void publishJointStates(const GripperState & gripper_state);
  double mapRange(double val, double prev_min, double prev_max, double new_min, double new_max, bool reverse);
  void stateCB(const GripperState::SharedPtr msg);

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleGripper> active_goal_;
};

}
#endif
