#include <cw2_class.h>
#include <algorithm>

void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> /*request*/,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Started =====");

  // ── PHASE 1: Scan scene from multiple overhead viewpoints ────────────
  // Cover the full reachable workspace: x in [-0.6, 0.7], y in [-0.55, 0.55]
  const std::vector<std::array<double, 2>> scan_xys = {
    { 0.45,  0.00}, { 0.50,  0.35}, { 0.50, -0.35},
    { 0.10,  0.50}, { 0.10, -0.50}, {-0.10,  0.00},
    { 0.25,  0.45}, { 0.25, -0.45}, {-0.20,  0.30},
    {-0.20, -0.30}
  };

  PointCPtr full_scene(new PointC);
  for (const auto & xy : scan_xys) {
    auto pose = makeDownwardPose(xy[0], xy[1], ARM_OVERHEAD_Z);
    if (!moveArmToPose(pose)) {
      RCLCPP_WARN(node_->get_logger(), "Scan: could not reach (%.2f, %.2f), skipping", xy[0], xy[1]);
      continue;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    PointCPtr raw  = waitForFreshCloud(2000);
    PointCPtr base = transformCloudToBaseFrame(raw);
    if (base && !base->empty()) {
      *full_scene += *base;
    }
  }
  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(), "Scan complete. Raw points: %zu", full_scene->size());

  // ── PHASE 2: Downsample and remove ground ───────────────────────────
  PointCPtr downsampled   = voxelDownsample(full_scene, 0.005f);
  PointCPtr above_ground  = removeGroundPlane(downsampled, 0.015f);
  RCLCPP_INFO(node_->get_logger(), "After ground removal: %zu points", above_ground->size());

  if (above_ground->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Task 3: no points above ground — aborting.");
    response->total_num_shapes      = 0;
    response->num_most_common_shape = 0;
    return;
  }

  // ── PHASE 3: Colour-based separation ────────────────────────────────
  // Obstacles: very dark / black
  PointCPtr obstacle_cloud = filterByColourRange(above_ground,
    0.0f, 0.25f, 0.0f, 0.25f, 0.0f, 0.25f);

  // Basket: brownish — red dominant, low green and blue
  PointCPtr basket_cloud = filterByColourRange(above_ground,
    0.30f, 0.75f, 0.05f, 0.35f, 0.05f, 0.35f);

  // Shapes: everything that is not black, not brown, not green (ground tile colour)
  PointCPtr shape_cloud(new PointC);
  for (const auto & pt : above_ground->points) {
    if (!std::isfinite(pt.x)) continue;
    float r = pt.r / 255.0f;
    float g = pt.g / 255.0f;
    float b = pt.b / 255.0f;
    bool is_black  = (r < 0.25f && g < 0.25f && b < 0.25f);
    bool is_brown  = (r > 0.30f && r > g * 1.4f && r > b * 1.4f && g < 0.40f);
    bool is_green  = (g > r * 1.25f && g > b * 1.25f);
    if (!is_black && !is_brown && !is_green) {
      shape_cloud->push_back(pt);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "Colour split — obstacles: %zu | basket: %zu | shapes: %zu",
    obstacle_cloud->size(), basket_cloud->size(), shape_cloud->size());

  // ── PHASE 4: Add obstacles to MoveIt! planning scene ────────────────
  int obs_id = 0;
  if (obstacle_cloud->size() > 30) {
    auto obs_clusters = euclideanCluster(obstacle_cloud, 0.04f, 30, 5000);
    for (const auto & obs : obs_clusters) {
      Eigen::Vector4f c = getCloudCentroid(obs);
      // Conservative 25x25x15cm box per obstacle
      addBoxCollisionObject(
        "t3_obstacle_" + std::to_string(obs_id++),
        c[0], c[1], c[2], 0.25, 0.25, 0.15);
    }
    RCLCPP_INFO(node_->get_logger(), "Added %d obstacle collision objects", obs_id);
  }

  // ── PHASE 5: Cluster and classify shapes ────────────────────────────
  std::vector<ShapeInfo> shapes;
  int nought_count = 0;
  int cross_count  = 0;

  if (shape_cloud->size() > 30) {
    auto clusters = euclideanCluster(shape_cloud, 0.03f, 40, 8000);
    RCLCPP_INFO(node_->get_logger(), "Found %zu shape clusters", clusters.size());

    for (const auto & cluster : clusters) {
      Eigen::Vector4f centroid = getCloudCentroid(cluster);
      std::string type = classifyShape(cluster);
      float yaw = detectShapeYaw(cluster);

      if (type == "unknown") {
        RCLCPP_WARN(node_->get_logger(),
          "Cluster at (%.2f, %.2f) returned 'unknown' — skipping", centroid[0], centroid[1]);
        continue;
      }

      shapes.push_back({centroid, type, yaw});

      if (type == "nought") nought_count++;
      else                  cross_count++;

      RCLCPP_INFO(node_->get_logger(),
        "Shape at (%.3f, %.3f, %.3f) → %s (yaw=%.2f)",
        centroid[0], centroid[1], centroid[2], type.c_str(), yaw);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "shape_cloud too small to cluster");
  }

  // ── PHASE 6: Fill response ───────────────────────────────────────────
  int total        = nought_count + cross_count;
  int most_common  = std::max(nought_count, cross_count);

  response->total_num_shapes      = total;
  response->num_most_common_shape = most_common;
  for (int i = 0; i < total; ++i) {
    response->most_common_shape_vector.push_back(
      static_cast<int64_t>(most_common));
  }

  RCLCPP_INFO(node_->get_logger(),
    "Shapes: total=%d, noughts=%d, crosses=%d, most_common=%d",
    total, nought_count, cross_count, most_common);

  // ── PHASE 7: Pick and place most common shape ────────────────────────
  if (shapes.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No shapes detected — skipping pick and place.");
    return;
  }

  std::string target_type = (nought_count >= cross_count) ? "nought" : "cross";
  RCLCPP_INFO(node_->get_logger(), "Will pick a '%s'", target_type.c_str());

  // Find basket centroid
  Eigen::Vector4f basket_centroid;
  if (basket_cloud && basket_cloud->size() > 10) {
    basket_centroid = getCloudCentroid(basket_cloud);
    RCLCPP_INFO(node_->get_logger(),
      "Basket detected at (%.3f, %.3f, %.3f)",
      basket_centroid[0], basket_centroid[1], basket_centroid[2]);
  } else {
    // Fallback: use one of the two known basket positions from world_spawner.py
    basket_centroid = Eigen::Vector4f(-0.41f, -0.36f, static_cast<float>(GROUND_Z), 0.0f);
    RCLCPP_WARN(node_->get_logger(),
      "Basket not detected — using fallback position (%.2f, %.2f)",
      basket_centroid[0], basket_centroid[1]);
  }

  // Add basket walls as collision objects
  double bx = basket_centroid[0], by = basket_centroid[1];
  addBoxCollisionObject("t3_basket_N", bx + 0.19, by,         GROUND_Z + 0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t3_basket_S", bx - 0.19, by,         GROUND_Z + 0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t3_basket_E", bx,         by + 0.19, GROUND_Z + 0.025, 0.37, 0.02, 0.06);
  addBoxCollisionObject("t3_basket_W", bx,         by - 0.19, GROUND_Z + 0.025, 0.37, 0.02, 0.06);

  // Find the first shape of the target type
  const ShapeInfo * target = nullptr;
  for (const auto & s : shapes) {
    if (s.type == target_type) { target = &s; break; }
  }

  if (target != nullptr) {
    geometry_msgs::msg::PointStamped obj_stamped;
    obj_stamped.header.frame_id = "panda_link0";
    obj_stamped.header.stamp    = node_->get_clock()->now();
    obj_stamped.point.x = target->centroid[0];
    obj_stamped.point.y = target->centroid[1];
    obj_stamped.point.z = target->centroid[2];

    geometry_msgs::msg::PointStamped goal_stamped;
    goal_stamped.header.frame_id = "panda_link0";
    goal_stamped.header.stamp    = node_->get_clock()->now();
    goal_stamped.point.x = basket_centroid[0];
    goal_stamped.point.y = basket_centroid[1];
    goal_stamped.point.z = basket_centroid[2];

    RCLCPP_INFO(node_->get_logger(),
      "Picking %s at (%.3f, %.3f) → placing in basket at (%.3f, %.3f)",
      target_type.c_str(),
      target->centroid[0], target->centroid[1],
      basket_centroid[0], basket_centroid[1]);

    bool pick_ok = pickObject(obj_stamped, target->yaw_angle);
    if (pick_ok) {
      placeObject(goal_stamped);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Task 3: pick failed!");
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "Task 3: could not find any shape of type '%s' to pick!", target_type.c_str());
  }

  // ── PHASE 8: Cleanup ─────────────────────────────────────────────────
  removeCollisionObject("t3_basket_N");
  removeCollisionObject("t3_basket_S");
  removeCollisionObject("t3_basket_E");
  removeCollisionObject("t3_basket_W");
  for (int i = 0; i < obs_id; ++i) {
    removeCollisionObject("t3_obstacle_" + std::to_string(i));
  }
  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
