#include <cw2_class.h>

// ---------------------------------------------------------------------------
// makeDownwardPose
// ---------------------------------------------------------------------------
geometry_msgs::msg::Pose
cw2::makeDownwardPose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // 180 deg around X — end-effector pointing straight down
  tf2::Quaternion base_q(1.0, 0.0, 0.0, 0.0);
  tf2::Quaternion yaw_q;
  yaw_q.setRPY(0.0, 0.0, yaw);
  tf2::Quaternion result_q = yaw_q * base_q;
  result_q.normalize();

  pose.orientation = tf2::toMsg(result_q);
  return pose;
}

// ---------------------------------------------------------------------------
// moveArmToPose
// ---------------------------------------------------------------------------
bool cw2::moveArmToPose(const geometry_msgs::msg::Pose & target_pose)
{
  arm_group_->setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    success = (arm_group_->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "moveArmToPose: planning or execution failed");
  }
  return success;
}

// ---------------------------------------------------------------------------
// moveArmToNamedTarget
// ---------------------------------------------------------------------------
bool cw2::moveArmToNamedTarget(const std::string & target_name)
{
  arm_group_->setNamedTarget(target_name);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    success = (arm_group_->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(),
                "moveArmToNamedTarget('%s'): planning or execution failed",
                target_name.c_str());
  }
  return success;
}

// ---------------------------------------------------------------------------
// setGripper
// ---------------------------------------------------------------------------
bool cw2::setGripper(double width)
{
  hand_group_->setJointValueTarget("panda_finger_joint1", width / 2.0);
  hand_group_->setJointValueTarget("panda_finger_joint2", width / 2.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (hand_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    success = (hand_group_->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "setGripper: planning or execution failed");
  }
  return success;
}

// ---------------------------------------------------------------------------
// pickObject
// ---------------------------------------------------------------------------
bool cw2::pickObject(const geometry_msgs::msg::PointStamped & object_point,
                     double grasp_yaw)
{
  double x = object_point.point.x;
  double y = object_point.point.y;

  bool ok = true;

  // 1. Move to ready
  ok = moveArmToNamedTarget("ready") && ok;

  // 2. Open gripper
  ok = setGripper(GRIPPER_OPEN) && ok;

  // 3. Pre-grasp pose — 15 cm above shape
  geometry_msgs::msg::Pose pre_grasp =
    makeDownwardPose(x, y, GROUND_Z + SHAPE_HEIGHT + 0.15, grasp_yaw);
  ok = moveArmToPose(pre_grasp) && ok;

  // 4. Descend via Cartesian path
  geometry_msgs::msg::Pose grasp_pose =
    makeDownwardPose(x, y, GROUND_Z + SHAPE_HEIGHT + 0.005, grasp_yaw);

  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints = {grasp_pose};
  double fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction >= 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "pickObject: descend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 5. Close gripper
  ok = setGripper(GRIPPER_CLOSED) && ok;

  // 6. Wait
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 7. Lift via Cartesian path
  waypoints = {pre_grasp};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction >= 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "pickObject: lift Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 8. Return to ready
  ok = moveArmToNamedTarget("ready") && ok;

  return ok;
}

// ---------------------------------------------------------------------------
// placeObject
// ---------------------------------------------------------------------------
bool cw2::placeObject(const geometry_msgs::msg::PointStamped & goal_point)
{
  double x = goal_point.point.x;
  double y = goal_point.point.y;

  bool ok = true;

  // 1. Pre-place — 20 cm above basket
  geometry_msgs::msg::Pose pre_place =
    makeDownwardPose(x, y, GROUND_Z + 0.20);
  ok = moveArmToPose(pre_place) && ok;

  // 2. Descend into basket via Cartesian
  geometry_msgs::msg::Pose place_pose =
    makeDownwardPose(x, y, GROUND_Z + 0.08);
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints = {place_pose};
  double fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction >= 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "placeObject: descend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 3. Release
  ok = setGripper(GRIPPER_OPEN) && ok;
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // 4. Ascend via Cartesian
  waypoints = {pre_place};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction >= 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "placeObject: ascend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  return ok;
}

// ---------------------------------------------------------------------------
// addGroundCollisionPlane
// ---------------------------------------------------------------------------
void cw2::addGroundCollisionPlane()
{
  addBoxCollisionObject("ground_plane", 0.0, 0.0, -0.005, 3.0, 3.0, 0.01);
}

// ---------------------------------------------------------------------------
// addBoxCollisionObject
// ---------------------------------------------------------------------------
void cw2::addBoxCollisionObject(const std::string & id,
                                double x, double y, double z,
                                double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "panda_link0";
  obj.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObject(obj);
}

// ---------------------------------------------------------------------------
// removeCollisionObject
// ---------------------------------------------------------------------------
void cw2::removeCollisionObject(const std::string & id)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_interface_.applyCollisionObject(obj);
}
