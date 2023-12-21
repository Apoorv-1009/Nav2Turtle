#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <map>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tuple>
#include <vector>

#include "cmath"
#include "geometry_msgs/msg/twist.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

/**
 * @brief Namespace robot_controller
 *
 */
namespace robot_controller {

class controller : public rclcpp::Node {
 private:
  /**
   * @brief Shorten FollowWaypoints and GoalHandleNavigation
   *
   */
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  //##### STATE VARIABLES #####//
  /**
   * @brief Store the aruco marker data
   *
   */
  int64_t aruco_marker_data_{0};

  /**
   * @brief Store the waypoints related to each aruco marker
   *
   */
  std::vector<std::tuple<std::string, std::string>> waypoints_;

  /**
   * @brief Store battery positions in a map
   *
   */
  std::map<std::string, double> object_x_;
  std::map<std::string, double> object_y_;
  std::map<std::string, double> object_z_;
  std::map<std::string, double> object_quat_x_;
  std::map<std::string, double> object_quat_y_;
  std::map<std::string, double> object_quat_z_;
  std::map<std::string, double> object_quat_w_;

  /**
   * @brief Store colours of objects
   *
   */
  std::map<u_int8_t, std::string> colors_;

  /**
   * @brief Store types of objects
   *
   */
  std::map<u_int8_t, std::string> objects_;

  /**
   * @brief Time of the ROS clock
   *
   */
  rclcpp::Time time;

  //##### PUBLISHER POINTERS #####//
  /**
   * @brief Publishes initial pose of turtlebot
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      intial_pose_publisher_;

  //##### Subscriber Pointers #####//

  /**
   * @brief Subscribes to aruco_markers
   *
   */
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr
      aruco_marker_subscriber_;

  /**
   * @brief Subscriber to /mage/camera1/image, gets camera1_frame
   *
   */
  rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      camera_1_subscriber_;

  /**
   * @brief Subscriber to /mage/camera2/image, gets camera2_frame
   *
   */
  rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      camera_2_subscriber_;

  /**
   * @brief Subscriber to /mage/camera3/image, gets camera3_frame
   *
   */
  rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      camera_3_subscriber_;

  /**
   * @brief Subscriber to /mage/camera4/image, gets camera4_frame
   *
   */
  rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      camera_4_subscriber_;
 public:
  /**
   * @brief Subscriber to /mage/camera5/image, gets camera5_frame
   *
   */
  rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      camera_5_subscriber_;

  /**
   * @brief Subscriber to odom
   *
   */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  /**
   * @brief Subscribes to clock
   *
   */
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;

  //##### CALLBACKS #####//

  /**
   * @brief Callback to subscribe to aruco_markers
   * @param msg message type: ros2_aruco_interfaces::msg::ArucoMarkers as a
   * frame
   *
   */
  void aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers);

  /**
   * @brief Callback to subscribe to mage/camera1/image
   * @param msg message type: mage_msgs::msg::AdvancedLogicalCameraImage as a
   * frame
   */
  void camera1_cb(const mage_msgs::msg::AdvancedLogicalCameraImage);

  /**
   * @brief Callback to subscribe to mage/camera2/image
   * @param msg message type: mage_msgs::msg::AdvancedLogicalCameraImage as a
   * frame
   */
  void camera2_cb(const mage_msgs::msg::AdvancedLogicalCameraImage);

  /**
   * @brief Callback to subscribe to mage/camera3/image
   * @param msg message type: mage_msgs::msg::AdvancedLogicalCameraImage as a
   * frame
   */
  void camera3_cb(const mage_msgs::msg::AdvancedLogicalCameraImage);

  /**
   * @brief Callback to subscribe to mage/camera4/image
   * @param msg message type: mage_msgs::msg::AdvancedLogicalCameraImage as a
   * frame
   */
  void camera4_cb(const mage_msgs::msg::AdvancedLogicalCameraImage);

  /**
   * @brief Callback to subscribe to mage/camera5/image
   * @param msg message type: mage_msgs::msg::AdvancedLogicalCameraImage as a
   * frame
   */
  void camera5_cb(const mage_msgs::msg::AdvancedLogicalCameraImage);

  /**
   * @brief Callback to subscribe to odom
   * @param msg message type: nav_msgs::msg::Odometry
   */
  void odom_cb(const nav_msgs::msg::Odometry);

  /**
   * @brief Callback to subscribe to clock
   * @param msg System time
   *
   */
  void clock_cb(const rosgraph_msgs::msg::Clock);

  /**
   * @brief Callback response from the server after receiving the goal
   *
   */
  void goal_response_callback(
      std::shared_future<GoalHandleNavigation::SharedPtr> future);

  /**
   * @brief Callback to receive feedback while action is being executed
   *
   * @param feedback
   */
  void feedback_callback(
      GoalHandleNavigation::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback);

  /**
   * @brief Callback to receive result of the action
   *
   * @param result
   */
  void result_callback(const GoalHandleNavigation::WrappedResult& result);

  //##### METHODS #####//

  /**
   * @brief Method to print state variables, used for debugging
   *
   */
  void state_printer();

  /**
   * @brief Method to move through the poses obtained from the aruco marker
   *
   */
  void navigation();

  //##### TIMERS #####//
  /**
   * @brief Timer to print state variables of the turtlebot, used for debugging
   *
   */
  rclcpp::TimerBase::SharedPtr timer_state_printer;

  /**
   * @brief Timer to call the navigation function
   *
   */
  rclcpp::TimerBase::SharedPtr timer_navigation;

  //##### FRAME POINTERS #####//

  /**
   * @brief Listener for objects detected by camera1
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_camera1{nullptr};

  /**
   * @brief Buffer for camera 1
   *
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_camera1;

  /**
   * @brief Broadcaster for camera 1
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_camera1;

  /**
   * @brief Listener for objects detected by camera2
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_camera2{nullptr};

  /**
   * @brief Buffer for camera 2
   *
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_camera2;

  /**
   * @brief Broadcaster for camera 2
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_camera2;

  /**
   * @brief Listener for objects detected by camera3
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_camera3{nullptr};

  /**
   * @brief Buffer for camera 3
   *
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_camera3;

  /**
   * @brief Broadcaster for camera 3
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_camera3;

  /**
   * @brief Listener for objects detected by camera4
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_camera4{nullptr};

  /**
   * @brief Buffer for camera 4
   *
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_camera4;

  /**
   * @brief Broadcaster for camera 4
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_camera4;

  /**
   * @brief Listener for objects detected by camera5
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_camera5{nullptr};

  /**
   * @brief Buffer for camera 5
   *
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_camera5;

  /**
   * @brief Broadcaster for camera 5
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_camera5;

  //##### ACTION CLIENT #####//
  rclcpp_action::Client<FollowWaypoints>::SharedPtr nav2_client_;

 public:
  /**
   * @brief Constructor
   * @param node_name Name of the node
   */
  controller(std::string node_name) : Node(node_name) {
    // Initialise colors
    colors_[0] = "red";
    colors_[1] = "green";
    colors_[2] = "blue";
    colors_[3] = "orange";
    colors_[4] = "purple";

    // Initialse objects
    objects_[10] = "battery";
    objects_[11] = "pump";
    objects_[12] = "sensor";
    objects_[13] = "regulator";

    //##### DECLARE PARAMETERS #####//
    this->declare_parameter("aruco_0.wp1.type", "battery");
    this->declare_parameter("aruco_0.wp1.color", "green");
    this->declare_parameter("aruco_0.wp2.type", "battery");
    this->declare_parameter("aruco_0.wp2.color", "red");
    this->declare_parameter("aruco_0.wp3.type", "battery");
    this->declare_parameter("aruco_0.wp3.color", "orange");
    this->declare_parameter("aruco_0.wp4.type", "battery");
    this->declare_parameter("aruco_0.wp4.color", "purple");
    this->declare_parameter("aruco_0.wp5.type", "battery");
    this->declare_parameter("aruco_0.wp5.color", "blue");

    this->declare_parameter("aruco_1.wp1.type", "battery");
    this->declare_parameter("aruco_1.wp1.color", "blue");
    this->declare_parameter("aruco_1.wp2.type", "battery");
    this->declare_parameter("aruco_1.wp2.color", "green");
    this->declare_parameter("aruco_1.wp3.type", "battery");
    this->declare_parameter("aruco_1.wp3.color", "orange");
    this->declare_parameter("aruco_1.wp4.type", "battery");
    this->declare_parameter("aruco_1.wp4.color", "red");
    this->declare_parameter("aruco_1.wp5.type", "battery");
    this->declare_parameter("aruco_1.wp5.color", "purple");

    //##### PUBLISHERS #####//

    // To initialpose
    intial_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

    //##### SUBSCRIBERS #####//

    // To aruco_markers
    aruco_marker_subscriber_ =
        this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10,
            std::bind(&controller::aruco_markers_cb, this,
                      std::placeholders::_1));

    // To mage/camera1/image
    camera_1_subscriber_ =
        this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&controller::camera1_cb, this, std::placeholders::_1));

    // To mage/camera2/image
    camera_2_subscriber_ =
        this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera2/image", rclcpp::SensorDataQoS(),
            std::bind(&controller::camera2_cb, this, std::placeholders::_1));

    // To mage/camera3/image
    camera_3_subscriber_ =
        this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera3/image", rclcpp::SensorDataQoS(),
            std::bind(&controller::camera3_cb, this, std::placeholders::_1));

    // To mage/camera4/image
    camera_4_subscriber_ =
        this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera4/image", rclcpp::SensorDataQoS(),
            std::bind(&controller::camera4_cb, this, std::placeholders::_1));

    // To mage/camera5/image
    camera_5_subscriber_ =
        this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera5/image", rclcpp::SensorDataQoS(),
            std::bind(&controller::camera5_cb, this, std::placeholders::_1));

    // To odom
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&controller::odom_cb, this, std::placeholders::_1));

    // To clock
    clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10,
        std::bind(&controller::clock_cb, this, std::placeholders::_1));

    //##### TIMERS ######//

    timer_state_printer =
        this->create_wall_timer(std::chrono::milliseconds(1000),
                                std::bind(&controller::state_printer, this));

    timer_navigation =
        this->create_wall_timer(std::chrono::milliseconds(6000),
                                std::bind(&controller::navigation, this));

    //##### TF BROADCASTERS #####//

    // Camera 1
    tf_broadcaster_camera1 =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_camera1 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_camera1->setUsingDedicatedThread(true);
    tf_listener_camera1 =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_camera1);

    // Camera 2
    tf_broadcaster_camera2 =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_camera2 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_camera2 =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_camera2);
    tf_buffer_camera2->setUsingDedicatedThread(true);

    // Camera 3
    tf_broadcaster_camera3 =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_camera3 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_camera3 =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_camera3);
    tf_buffer_camera3->setUsingDedicatedThread(true);

    // Camera 4
    tf_broadcaster_camera4 =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_camera4 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_camera4 =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_camera4);
    tf_buffer_camera4->setUsingDedicatedThread(true);

    // Camera 5
    tf_broadcaster_camera5 =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_camera5 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_camera5 =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_camera5);
    tf_buffer_camera5->setUsingDedicatedThread(true);

    //##### CLIENT #####//
    
    nav2_client_ =
        rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  }
};
}  // namespace robot_controller