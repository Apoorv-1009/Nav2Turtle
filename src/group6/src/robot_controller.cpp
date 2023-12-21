#include "robot_controller.hpp"

void robot_controller::controller::aruco_markers_cb(
    const ros2_aruco_interfaces::msg::ArucoMarkers msg) {
  if (msg.poses.size() != 0) {
    aruco_marker_data_ = msg.marker_ids[0];

    // Stop subscribing
    aruco_marker_subscriber_ = nullptr;

    std::string aruco_param = "aruco_" + std::to_string(aruco_marker_data_);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "aruco marker reading: " << aruco_param);

    RCLCPP_INFO(this->get_logger(), "Waypoints: ");

    // Extract waypoints from the aruco marker
    for (int i = 1; i <= 5; i++) {
      std::string type;
      std::string color;
      std::string wp = ".wp" + std::to_string(i);

      // aruco_param.wpi.type
      type = this->get_parameter(aruco_param + wp + "." + "type").as_string();
      // aruco_param.wpi.color
      color = this->get_parameter(aruco_param + wp + "." + "color").as_string();
      // RCLCPP_INFO_STREAM(this->get_logger(),"type : " << type);
      RCLCPP_INFO_STREAM(this->get_logger(), "color : " << color);

      waypoints_.push_back(std::make_tuple(type, color));
    }
  }
}

void robot_controller::controller::camera1_cb(
    mage_msgs::msg::AdvancedLogicalCameraImage msg) {
  // Publish detected object in camera1 frame
  geometry_msgs::msg::TransformStamped tf_object;
  tf_object.header.stamp = time;
  tf_object.header.frame_id = "camera1_frame";
  tf_object.child_frame_id = "camera1_object";

  tf_object.transform.translation.x = msg.part_poses[0].pose.position.x;
  tf_object.transform.translation.y = msg.part_poses[0].pose.position.y;
  tf_object.transform.translation.z = msg.part_poses[0].pose.position.z;

  tf_object.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
  tf_object.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
  tf_object.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
  tf_object.transform.rotation.w = msg.part_poses[0].pose.orientation.w;

  tf_broadcaster_camera1->sendTransform(tf_object);

  // Sleep to let the tf tree update
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  // Get the transform between detected object to map frame
  geometry_msgs::msg::TransformStamped tf_object_map;

  const std::string& fromFrameRel = "map";
  const std::string& toFrameRel = "camera1_object";
  try {
    tf_object_map = tf_buffer_camera1->lookupTransform(
        fromFrameRel, toFrameRel, tf2::TimePointZero, 50ms);
  } catch (const tf2::TransformException& ex) {
    // RCLCPP_INFO(
    // this->get_logger(), "Could not transform %s to %s: %s",
    // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  std::string detcted_color = colors_[msg.part_poses[0].part.color];

  // Use color as key and store position
  object_x_[detcted_color] = tf_object_map.transform.translation.x;
  object_y_[detcted_color] = tf_object_map.transform.translation.y;
  object_z_[detcted_color] = tf_object_map.transform.translation.z;
  object_quat_x_[detcted_color] = tf_object_map.transform.rotation.x;
  object_quat_y_[detcted_color] = tf_object_map.transform.rotation.y;
  object_quat_z_[detcted_color] = tf_object_map.transform.rotation.z;
  object_quat_w_[detcted_color] = tf_object_map.transform.rotation.w;

  // Unsubscribe to topic
  camera_1_subscriber_ = nullptr;
}

void robot_controller::controller::camera2_cb(
    mage_msgs::msg::AdvancedLogicalCameraImage msg) {
  // Publish detected object in camera2 frame
  geometry_msgs::msg::TransformStamped tf_object;
  tf_object.header.stamp = time;
  tf_object.header.frame_id = "camera2_frame";
  tf_object.child_frame_id = "camera2_object";

  tf_object.transform.translation.x = msg.part_poses[0].pose.position.x;
  tf_object.transform.translation.y = msg.part_poses[0].pose.position.y;
  tf_object.transform.translation.z = msg.part_poses[0].pose.position.z;

  tf_object.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
  tf_object.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
  tf_object.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
  tf_object.transform.rotation.w = msg.part_poses[0].pose.orientation.w;

  tf_broadcaster_camera2->sendTransform(tf_object);

  // Sleep to let the tf tree update
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  // Get the transform between detected object to map frame
  geometry_msgs::msg::TransformStamped tf_object_map;

  const std::string& fromFrameRel = "map";
  const std::string& toFrameRel = "camera2_object";
  try {
    tf_object_map = tf_buffer_camera2->lookupTransform(
        fromFrameRel, toFrameRel, tf2::TimePointZero, 50ms);
  } catch (const tf2::TransformException& ex) {
    // RCLCPP_INFO(
    // this->get_logger(), "Could not transform %s to %s: %s",
    // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  std::string detcted_color = colors_[msg.part_poses[0].part.color];

  // Use color as key and store position
  object_x_[detcted_color] = tf_object_map.transform.translation.x;
  object_y_[detcted_color] = tf_object_map.transform.translation.y;
  object_z_[detcted_color] = tf_object_map.transform.translation.z;
  object_quat_x_[detcted_color] = tf_object_map.transform.rotation.x;
  object_quat_y_[detcted_color] = tf_object_map.transform.rotation.y;
  object_quat_z_[detcted_color] = tf_object_map.transform.rotation.z;
  object_quat_w_[detcted_color] = tf_object_map.transform.rotation.w;

  // Unsubscribe to topic
  camera_2_subscriber_ = nullptr;
}

void robot_controller::controller::camera3_cb(
    mage_msgs::msg::AdvancedLogicalCameraImage msg) {
  // Publish detected object in camera3 frame
  geometry_msgs::msg::TransformStamped tf_object;
  tf_object.header.stamp = time;
  tf_object.header.frame_id = "camera3_frame";
  tf_object.child_frame_id = "camera3_object";

  tf_object.transform.translation.x = msg.part_poses[0].pose.position.x;
  tf_object.transform.translation.y = msg.part_poses[0].pose.position.y;
  tf_object.transform.translation.z = msg.part_poses[0].pose.position.z;

  tf_object.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
  tf_object.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
  tf_object.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
  tf_object.transform.rotation.w = msg.part_poses[0].pose.orientation.w;

  tf_broadcaster_camera3->sendTransform(tf_object);

  // Sleep to let the tf tree update
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  // Get the transform between detected object to map frame
  geometry_msgs::msg::TransformStamped tf_object_map;

  const std::string& fromFrameRel = "map";
  const std::string& toFrameRel = "camera3_object";
  try {
    tf_object_map = tf_buffer_camera3->lookupTransform(
        fromFrameRel, toFrameRel, tf2::TimePointZero, 50ms);
  } catch (const tf2::TransformException& ex) {
    // RCLCPP_INFO(
    // this->get_logger(), "Could not transform %s to %s: %s",
    // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  std::string detcted_color = colors_[msg.part_poses[0].part.color];

  // Use color as key and store position
  object_x_[detcted_color] = tf_object_map.transform.translation.x;
  object_y_[detcted_color] = tf_object_map.transform.translation.y;
  object_z_[detcted_color] = tf_object_map.transform.translation.z;
  object_quat_x_[detcted_color] = tf_object_map.transform.rotation.x;
  object_quat_y_[detcted_color] = tf_object_map.transform.rotation.y;
  object_quat_z_[detcted_color] = tf_object_map.transform.rotation.z;
  object_quat_w_[detcted_color] = tf_object_map.transform.rotation.w;

  // Unsubscribe to topic
  camera_3_subscriber_ = nullptr;
}

void robot_controller::controller::camera4_cb(
    mage_msgs::msg::AdvancedLogicalCameraImage msg) {
  // Publish detected object in camera4 frame
  geometry_msgs::msg::TransformStamped tf_object;
  tf_object.header.stamp = time;
  tf_object.header.frame_id = "camera4_frame";
  tf_object.child_frame_id = "camera4_object";

  tf_object.transform.translation.x = msg.part_poses[0].pose.position.x;
  tf_object.transform.translation.y = msg.part_poses[0].pose.position.y;
  tf_object.transform.translation.z = msg.part_poses[0].pose.position.z;

  tf_object.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
  tf_object.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
  tf_object.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
  tf_object.transform.rotation.w = msg.part_poses[0].pose.orientation.w;

  tf_broadcaster_camera4->sendTransform(tf_object);

  // Sleep to let the tf tree update
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  // Get the transform between detected object to map frame
  geometry_msgs::msg::TransformStamped tf_object_map;

  const std::string& fromFrameRel = "map";
  const std::string& toFrameRel = "camera4_object";
  try {
    tf_object_map = tf_buffer_camera4->lookupTransform(
        fromFrameRel, toFrameRel, tf2::TimePointZero, 50ms);
  } catch (const tf2::TransformException& ex) {
    // RCLCPP_INFO(
    // this->get_logger(), "Could not transform %s to %s: %s",
    // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  std::string detcted_color = colors_[msg.part_poses[0].part.color];

  // Use color as key and store position
  object_x_[detcted_color] = tf_object_map.transform.translation.x;
  object_y_[detcted_color] = tf_object_map.transform.translation.y;
  object_z_[detcted_color] = tf_object_map.transform.translation.z;
  object_quat_x_[detcted_color] = tf_object_map.transform.rotation.x;
  object_quat_y_[detcted_color] = tf_object_map.transform.rotation.y;
  object_quat_z_[detcted_color] = tf_object_map.transform.rotation.z;
  object_quat_w_[detcted_color] = tf_object_map.transform.rotation.w;

  // Unsubscribe to topic
  camera_4_subscriber_ = nullptr;
}

void robot_controller::controller::camera5_cb(
    mage_msgs::msg::AdvancedLogicalCameraImage msg) {
  // Publish detected object in camera5 frame
  geometry_msgs::msg::TransformStamped tf_object;
  tf_object.header.stamp = time;
  tf_object.header.frame_id = "camera5_frame";
  tf_object.child_frame_id = "camera5_object";

  tf_object.transform.translation.x = msg.part_poses[0].pose.position.x;
  tf_object.transform.translation.y = msg.part_poses[0].pose.position.y;
  tf_object.transform.translation.z = msg.part_poses[0].pose.position.z;

  tf_object.transform.rotation.x = msg.part_poses[0].pose.orientation.x;
  tf_object.transform.rotation.y = msg.part_poses[0].pose.orientation.y;
  tf_object.transform.rotation.z = msg.part_poses[0].pose.orientation.z;
  tf_object.transform.rotation.w = msg.part_poses[0].pose.orientation.w;

  tf_broadcaster_camera5->sendTransform(tf_object);

  // Sleep to let the tf tree update
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  // Get the transform between detected object to map frame
  geometry_msgs::msg::TransformStamped tf_object_map;

  const std::string& fromFrameRel = "map";
  const std::string& toFrameRel = "camera5_object";
  try {
    tf_object_map = tf_buffer_camera5->lookupTransform(
        fromFrameRel, toFrameRel, tf2::TimePointZero, 50ms);
  } catch (const tf2::TransformException& ex) {
    // RCLCPP_INFO(
    // this->get_logger(), "Could not transform %s to %s: %s",
    // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

  std::string detcted_color = colors_[msg.part_poses[0].part.color];

  // Use color as key and store position
  object_x_[detcted_color] = tf_object_map.transform.translation.x;
  object_y_[detcted_color] = tf_object_map.transform.translation.y;
  object_z_[detcted_color] = tf_object_map.transform.translation.z;
  object_quat_x_[detcted_color] = tf_object_map.transform.rotation.x;
  object_quat_y_[detcted_color] = tf_object_map.transform.rotation.y;
  object_quat_z_[detcted_color] = tf_object_map.transform.rotation.z;
  object_quat_w_[detcted_color] = tf_object_map.transform.rotation.w;

  // Unsubscribe to topic
  camera_5_subscriber_ = nullptr;
}

void robot_controller::controller::state_printer() {
  // // Arcuo Marker data
  // RCLCPP_INFO_STREAM(this->get_logger(),"Aruco data: " <<
  // aruco_marker_data_);

  // // Waypoints
  // for(auto i : waypoints_){
  //     RCLCPP_INFO_STREAM(this->get_logger(),"type : " << std::get<0>(i));
  //     RCLCPP_INFO_STREAM(this->get_logger(),"color : " << std::get<1>(i));
  //     RCLCPP_INFO_STREAM(this->get_logger(),"\n");
  // }

  // // Positions of objects
  // auto i = object_x_.begin();
  // while(i != object_x_.end()){
  //     RCLCPP_INFO_STREAM(this->get_logger(),"color: " << i->first);
  //     RCLCPP_INFO_STREAM(this->get_logger(),"x : " << i->second);
  //     i++;
  // }
  // auto j = object_y_.begin();
  // while(j != object_y_.end()){
  //     RCLCPP_INFO_STREAM(this->get_logger(),"color: " << j->first);
  //     RCLCPP_INFO_STREAM(this->get_logger(),"y : " << j->second);
  //     j++;
  // }
  // auto k = object_z_.begin();
  // while(k != object_z_.end()){
  //     RCLCPP_INFO_STREAM(this->get_logger(),"color: " << k->first);
  //     RCLCPP_INFO_STREAM(this->get_logger(),"z : " << k->second);
  //     k++;
  // }
}

void robot_controller::controller::odom_cb(const nav_msgs::msg::Odometry msg) {
  // Publish this message to /initialpose
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  // initial_pose.header = msg.header;
  initial_pose.header.frame_id = "map";
  initial_pose.pose = msg.pose;
  // RCLCPP_INFO_STREAM(this->get_logger(),"x : " << msg.pose.pose.position.x);
  // RCLCPP_INFO_STREAM(this->get_logger(),"y : " << msg.pose.pose.position.y);
  // RCLCPP_INFO_STREAM(this->get_logger(),"z : " << msg.pose.pose.position.z);

  intial_pose_publisher_->publish(initial_pose);

  // Sleep for 5s to let the tree update
  // rclcpp::sleep_for(std::chrono::milliseconds(5000));

  // Unsubscribe from odom
  odom_subscriber_ = nullptr;
}

void robot_controller::controller::clock_cb(
    const rosgraph_msgs::msg::Clock msg) {
  time = msg.clock;
}

void robot_controller::controller::navigation() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting Navigation...");

  auto goal_msg = FollowWaypoints::Goal();

  // Iterate through all the waypoints
  int j{0};
  goal_msg.poses.resize(5);
  for (auto i : waypoints_) {
    // Get the ith color from the waypoints list
    goal_msg.poses[j].header.frame_id = "map";
    goal_msg.poses[j].pose.position.x = object_x_[std::get<1>(i)];
    goal_msg.poses[j].pose.position.y = object_y_[std::get<1>(i)];
    goal_msg.poses[j].pose.position.z = object_z_[std::get<1>(i)];

    goal_msg.poses[j].pose.orientation.x = object_quat_x_[std::get<1>(i)];
    goal_msg.poses[j].pose.orientation.y = object_quat_y_[std::get<1>(i)];
    goal_msg.poses[j].pose.orientation.z = object_quat_z_[std::get<1>(i)];
    goal_msg.poses[j].pose.orientation.w = object_quat_w_[std::get<1>(i)];
    j++;
  }

  RCLCPP_INFO(this->get_logger(), "Sending waypoints");

  auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &controller::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&controller::feedback_callback, this, std::placeholders::_1,
                std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&controller::result_callback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO_STREAM(this->get_logger(), "Waypoints Executed");

  // Disable timer of navigation
  timer_navigation = nullptr;
}

void robot_controller::controller::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void robot_controller::controller::feedback_callback(
    rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
  // RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

void robot_controller::controller::result_callback(
    const GoalHandleNavigation::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::controller>("controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}