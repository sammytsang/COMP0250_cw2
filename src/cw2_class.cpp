#include <cw2_class.h>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
cw2::cw2(rclcpp::Node::SharedPtr node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  g_cloud_ptr(new PointC)
{
  // Services
  t1_service_ = node_->create_service<cw2_world_spawner::srv::Task1Service>(
    "task1_start",
    std::bind(&cw2::t1_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  t2_service_ = node_->create_service<cw2_world_spawner::srv::Task2Service>(
    "task2_start",
    std::bind(&cw2::t2_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  t3_service_ = node_->create_service<cw2_world_spawner::srv::Task3Service>(
    "task3_start",
    std::bind(&cw2::t3_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Parameters
  node_->declare_parameter<std::string>(
    "pointcloud_topic", "/r200/camera/depth_registered/points");
  node_->declare_parameter<bool>("pointcloud_qos_reliable", true);

  pointcloud_topic_ = node_->get_parameter("pointcloud_topic")
                            .get_parameter_value().get<std::string>();
  pointcloud_qos_reliable_ = node_->get_parameter("pointcloud_qos_reliable")
                                   .get_parameter_value().get<bool>();

  // Callback group and QoS
  pointcloud_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = pointcloud_callback_group_;

  rclcpp::QoS qos(10);
  if (pointcloud_qos_reliable_) {
    qos.reliable();
  } else {
    qos.best_effort();
  }

  color_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, qos,
    std::bind(&cw2::cloud_callback, this, std::placeholders::_1),
    sub_options);

  // MoveIt groups
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_, "hand");

  // Arm configuration
  arm_group_->setPlanningTime(15.0);
  arm_group_->setNumPlanningAttempts(10);
  arm_group_->setMaxVelocityScalingFactor(0.4);
  arm_group_->setMaxAccelerationScalingFactor(0.4);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.05);

  // Collision environment
  addGroundCollisionPlane();

  RCLCPP_INFO(node_->get_logger(), "cw2 node initialised");
}

// ---------------------------------------------------------------------------
// cloud_callback
// ---------------------------------------------------------------------------
void cw2::cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Convert to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  PointCPtr cloud(new PointC);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  g_input_pc_frame_id_ = msg->header.frame_id;
  g_cloud_ptr = std::move(cloud);
  ++g_cloud_sequence_;
}

// ---------------------------------------------------------------------------
// Task callback stubs (implementations are in cw2_task1/2/3.cpp)
// ---------------------------------------------------------------------------
void cw2::t1_callback(
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> req,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_WARN(node_->get_logger(), "Task 1 callback: not yet implemented");
}

void cw2::t2_callback(
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> req,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_WARN(node_->get_logger(), "Task 2 callback: not yet implemented");
}

void cw2::t3_callback(
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> req,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_WARN(node_->get_logger(), "Task 3 callback: not yet implemented");
}
