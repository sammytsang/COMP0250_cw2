#include <cw2_class.h>

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 2 Started =====");

  // Helper lambda: move overhead a point, capture cloud, classify shape
  auto classifyAtPoint = [this](const geometry_msgs::msg::PointStamped & pt_stamped) -> std::string
  {
    const auto & pt = pt_stamped.point;
    auto view_pose = makeDownwardPose(pt.x, pt.y, ARM_OVERHEAD_Z);
    if (!moveArmToPose(view_pose)) {
      RCLCPP_WARN(node_->get_logger(), "classifyAtPoint: move failed, trying anyway");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    PointCPtr raw   = waitForFreshCloud(3000);
    PointCPtr base  = transformCloudToBaseFrame(raw);
    PointCPtr crop  = cropBoxAroundPoint(base, pt.x, pt.y, pt.z, 0.15);
    PointCPtr shape = removeGroundPlane(crop, 0.015f);
    std::string type = classifyShape(shape);
    RCLCPP_INFO(node_->get_logger(),
      "Shape at (%.2f, %.2f) → %s", pt.x, pt.y, type.c_str());
    return type;
  };

  // Classify all three shapes
  std::string ref1_type    = classifyAtPoint(request->ref_object_points[0]);
  std::string ref2_type    = classifyAtPoint(request->ref_object_points[1]);
  std::string mystery_type = classifyAtPoint(request->mystery_object_point);

  RCLCPP_INFO(node_->get_logger(),
    "Ref1=%s | Ref2=%s | Mystery=%s",
    ref1_type.c_str(), ref2_type.c_str(), mystery_type.c_str());

  // Match mystery to reference
  if (mystery_type == ref1_type) {
    response->mystery_object_num = 1;
  } else if (mystery_type == ref2_type) {
    response->mystery_object_num = 2;
  } else {
    // Fallback: if classifier returned "unknown" for mystery or it didn't
    // match either reference, default to 1.
    RCLCPP_WARN(node_->get_logger(),
      "Mystery type '%s' did not match ref1='%s' or ref2='%s'. Defaulting to 1.",
      mystery_type.c_str(), ref1_type.c_str(), ref2_type.c_str());
    response->mystery_object_num = 1;
  }

  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(),
    "===== Task 2 Complete: answer=%ld =====", response->mystery_object_num);
}
