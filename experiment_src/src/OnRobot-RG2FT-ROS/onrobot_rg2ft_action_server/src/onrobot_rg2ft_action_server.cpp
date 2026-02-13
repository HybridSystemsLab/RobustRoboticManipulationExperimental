/**
 * ROS 2 action server bridging control_msgs/action/GripperCommand to OnRobot topics
 */

#include <onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h>

namespace onrobot_rg2ft_action_server
{

OnRobotRG2FTActionServer::OnRobotRG2FTActionServer(const std::string & name, const GripperParams & params)
: rclcpp::Node(name),
  action_name_(this->declare_parameter<std::string>("action_server_name", "gripper_controller/gripper_cmd")),
  gripper_params_(params),
  is_initialized(false),
  position_goal(0),
  force_goal(0)
{
  // Declare parameters with defaults
  this->declare_parameter<std::string>("control_topic", gripper_params_.control_topic_);
  this->declare_parameter<std::string>("state_topic", gripper_params_.state_topic_);
  this->declare_parameter<std::string>("joint_states_topic", gripper_params_.joint_states_topic_);
  this->declare_parameter<std::string>("joint_name", gripper_params_.joint_name_);

  gripper_params_.control_topic_ = this->get_parameter("control_topic").as_string();
  gripper_params_.state_topic_ = this->get_parameter("state_topic").as_string();
  gripper_params_.joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
  gripper_params_.joint_name_ = this->get_parameter("joint_name").as_string();

  state_sub_ = this->create_subscription<GripperState>(
    gripper_params_.state_topic_, 10,
    std::bind(&OnRobotRG2FTActionServer::stateCB, this, std::placeholders::_1));
  goal_pub_ = this->create_publisher<GripperCtrl>(gripper_params_.control_topic_, 10);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(gripper_params_.joint_states_topic_, 10);

  action_server_ = rclcpp_action::create_server<GripperCommand>(
    this,
    action_name_,
    std::bind(&OnRobotRG2FTActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&OnRobotRG2FTActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&OnRobotRG2FTActionServer::handle_accepted, this, std::placeholders::_1)
  );
}

rclcpp_action::GoalResponse OnRobotRG2FTActionServer::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal> goal)
{
  (void)goal; // validate in execute
  RCLCPP_INFO(this->get_logger(), "Received new goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OnRobotRG2FTActionServer::handle_cancel(const std::shared_ptr<GoalHandleGripper>)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OnRobotRG2FTActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  active_goal_ = goal_handle;
  const auto goal = goal_handle->get_goal();
  try {
    auto ctrl_msg = goalToGripperCtrl(*goal);
    goal_pub_->publish(ctrl_msg);
    position_goal = static_cast<int>(ctrl_msg.target_width);
    force_goal = static_cast<int>(ctrl_msg.target_force);
  } catch (BadArgumentsError &) {
    auto result = std::make_shared<GripperCommand::Result>();
    result->position = 0.0;
    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = false;
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(), "%s bad goal issued to gripper", action_name_.c_str());
    active_goal_.reset();
  }
}

void OnRobotRG2FTActionServer::stateCB(const GripperState::SharedPtr msg)
{
  publishJointStates(*msg);

  std::shared_ptr<GoalHandleGripper> goal_handle;
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    goal_handle = active_goal_;
  }
  if (!goal_handle) {
    return;
  }
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<GripperCommand::Result>();
    result->position = static_cast<double>(msg->actual_gripper_width);
    result->effort = static_cast<double>(force_goal);
    result->stalled = false;
    result->reached_goal = false;
    goal_handle->canceled(result);
  return;
  }

  if (
    msg->actual_gripper_width > position_goal - GOAL_TOLERANCE &&
    msg->actual_gripper_width < position_goal + GOAL_TOLERANCE &&
    msg->gripper_busy == 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s succeeded (reached target position)", action_name_.c_str());
    auto result = std::make_shared<GripperCommand::Result>();
    result->position = static_cast<double>(msg->actual_gripper_width);
    result->effort = static_cast<double>(force_goal);
    result->stalled = false;
    result->reached_goal = true;
    goal_handle->succeed(result);
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      active_goal_.reset();
    }
  }
  else if (msg->grip_detected == 1 && msg->gripper_busy == 0)
  {
    RCLCPP_INFO(this->get_logger(), "%s succeeded (gripped object and stalled)", action_name_.c_str());
    auto result = std::make_shared<GripperCommand::Result>();
    result->position = static_cast<double>(msg->actual_gripper_width);
    result->effort = static_cast<double>(force_goal);
    result->stalled = true;
    result->reached_goal = false;
    goal_handle->succeed(result);
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      active_goal_.reset();
    }
  }
  else
  {
    auto feedback = std::make_shared<GripperCommand::Feedback>();
    feedback->position = static_cast<double>(msg->actual_gripper_width);
    feedback->effort = static_cast<double>(force_goal);
    feedback->stalled = false;
    feedback->reached_goal = false;
    goal_handle->publish_feedback(feedback);
  }
}

GripperCtrl OnRobotRG2FTActionServer::goalToGripperCtrl(const GripperCommand::Goal & goal)
{
  double angle = goal.command.position;
  double max_effort = goal.command.max_effort;
  if (max_effort == 0) {
    max_effort = gripper_params_.default_effort_;
  }
  if (
    angle < gripper_params_.min_angle_ ||
    angle > gripper_params_.max_angle_ ||
    max_effort < gripper_params_.min_effort_ ||
    max_effort > gripper_params_.max_effort_)
  {
    throw BadArgumentsError();
  }

  double gripper_ctrl_position = mapRange(
    angle,
    gripper_params_.min_angle_,
    gripper_params_.max_angle_,
    MIN_POSITION,
    MAX_POSITION,
    true);

  double gripper_ctrl_effort = mapRange(
    max_effort,
    gripper_params_.min_effort_,
    gripper_params_.max_effort_,
    MIN_FORCE,
    MAX_FORCE,
    false);

  GripperCtrl ctrl_msg;
  ctrl_msg.target_width = static_cast<uint16_t>(gripper_ctrl_position);
  ctrl_msg.target_force = static_cast<uint16_t>(gripper_ctrl_effort);
  ctrl_msg.control = 1;
  return ctrl_msg;
}

void OnRobotRG2FTActionServer::publishJointStates(const GripperState & gripper_state)
{
  double position_radians = mapRange(
    gripper_state.actual_gripper_width,
    MIN_POSITION,
    MAX_POSITION,
    gripper_params_.min_angle_,
    gripper_params_.max_angle_,
    true);

  sensor_msgs::msg::JointState msg;
  msg.name.resize(1);
  msg.position.resize(1);
  msg.header.frame_id = "";
  msg.header.stamp = this->now();
  msg.position.at(0) = position_radians;
  msg.name.at(0) = gripper_params_.joint_name_;
  joint_states_pub_->publish(msg);
}

double OnRobotRG2FTActionServer::mapRange(double val, double prev_min, double prev_max, double new_min, double new_max, bool reverse)
{
  double ret = (val - prev_min) * (new_max - new_min) / (prev_max - prev_min) + new_min;
  if (reverse) {
    ret = new_max - ret;
  }
  return ret;
}

} // namespace onrobot_rg2ft_action_server
