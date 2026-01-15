#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "ros2_kdl_package/action/execute_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ObjectDispatcher : public rclcpp::Node
{
public:
  ObjectDispatcher()
  : Node("object_dispatcher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    trajectory_client_ =
      rclcpp_action::create_client<ros2_kdl_package::action::ExecuteTrajectory>(
        this, "execute_trajectory");

    gripper_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/gripper_controller/follow_joint_trajectory");

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "dispatch_object",
      std::bind(&ObjectDispatcher::dispatch_cb, this, std::placeholders::_1, std::placeholders::_2));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/iiwa_cam/image_raw", 10,
      std::bind(&ObjectDispatcher::imageCallback, this, std::placeholders::_1));

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/iiwa_cam/camera_info", 10,
      std::bind(&ObjectDispatcher::camInfoCallback, this, std::placeholders::_1));

    while (!trajectory_client_->wait_for_action_server(5s) && rclcpp::ok()) {}
    while (!gripper_client_->wait_for_action_server(5s) && rclcpp::ok()) {}

    RCLCPP_INFO(this->get_logger(), "ObjectDispatcher ready");
  }

private:
  rclcpp_action::Client<ros2_kdl_package::action::ExecuteTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool exploring_ = false;
  bool target_detected_ = false;
  bool caminfo_ready_ = false;

  double fx_, fy_, cx_, cy_;
  double target_x_world_ = 0.0;
  double target_y_world_ = 0.0;
  double object_plane_z_ = 0.01;

  std::string camera_frame_ = "camera_link";
  std::string world_frame_  = "world";

  void dispatch_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    exploring_ = true;
    target_detected_ = false;

    RCLCPP_INFO(this->get_logger(), "Service called: waiting for target...");

    auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    
    *timer = this->create_wall_timer(100ms, [this, res, timer]() {
      if (target_detected_) {
        run_sequence();
        exploring_ = false;
        res->message = "IIWA pick & place executed";
        (*timer)->cancel();
      }
    });
  }

  void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (caminfo_ready_) return;

    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    caminfo_ready_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!caminfo_ready_ || !exploring_) return;

    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_green(35, 100, 50);
    cv::Scalar upper_green(85, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);


    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto &c : contours)
    {
      if (cv::contourArea(c) < 100) continue;

      cv::Point target_point;
      std::string point_type;
      cv::Rect bbox = cv::boundingRect(c);
      target_point.x = bbox.x + bbox.width / 2.0;
      target_point.y = bbox.y + bbox.height / 2.0;
      point_type = "bbox_center";

      double u = target_point.x;
      double v = target_point.y;
      double obj_h = 0.06;

      double X_distance = getCameraHeight()- obj_h/2.0;
      
      geometry_msgs::msg::PointStamped p_cam;
      p_cam.header.frame_id = camera_frame_;
      p_cam.header.stamp = rclcpp::Time(0);
      
      p_cam.point.x = X_distance;
      p_cam.point.y = -(u - cx_) * X_distance / fx_;
      p_cam.point.z = -(v - cy_) * X_distance / fy_;

      try {
        auto p_world = tf_buffer_.transform(p_cam, world_frame_);
        target_x_world_ = p_world.point.x;
        target_y_world_ = p_world.point.y;
        target_detected_ = true;
        
        RCLCPP_INFO(this->get_logger(),
          "Target position: (%.3f, %.3f, %.3f)", 
          target_x_world_, target_y_world_, p_world.point.z);
          
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }
      break;
    }
  }

  double getCameraHeight()
  {
    try {
      auto tf = tf_buffer_.lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
      return tf.transform.translation.z - object_plane_z_;
    } catch (...) {
      return 0.30;
    }
  }

  void run_sequence()
  {
      geometry_msgs::msg::Pose pre_grasp = make_pose(target_x_world_, target_y_world_, 0.30);
      geometry_msgs::msg::Pose grasp     = make_pose(target_x_world_, target_y_world_, 0.11);
      geometry_msgs::msg::Pose lift      = make_pose(target_x_world_, target_y_world_, 0.65);
      geometry_msgs::msg::Pose bin       = make_pose(0.75, -0.65, 0.65);
      geometry_msgs::msg::Pose wait_pos  = make_pose(0.20, 0.0, 0.8375);

      send_trajectory(pre_grasp, "velocity_ctrl", "linear", 7.0, 2.0, 7.0, 0.3, 300, "trapezoidal", "velocity", 
          [this, grasp, lift, bin, wait_pos]() {
              send_trajectory(grasp, "velocity_ctrl", "linear", 3.0, 1.0, 3.0, 0.5, 300, "trapezoidal", "velocity",
                  [this, lift, bin, wait_pos]() {
                      send_gripper({0.03, 0.03}, 
                          [this, lift, bin, wait_pos]() {
                              send_trajectory(lift, "velocity_ctrl", "linear", 5.0, 1.0, 5.0, 0.5, 300, "trapezoidal", "velocity",
                                  [this, bin, wait_pos]() {
                                      send_trajectory(bin, "velocity_ctrl", "linear", 7.0, 2.0, 7.0, 0.3, 300, "trapezoidal", "velocity",
                                          [this, wait_pos]() {
                                              send_gripper({0.0, 0.0}, 
                                                  [this, wait_pos]() {
                                                      send_trajectory(wait_pos, "velocity_ctrl", "linear", 7.0, 2.0, 7.0, 0.3, 300, "trapezoidal", "velocity",
                                                          []() {
                                                              RCLCPP_INFO(rclcpp::get_logger("object_dispatcher"), "Sequence completed!");
                                                          }
                                                      );
                                                  }
                                              );
                                          }
                                      );
                                  }
                              );
                          }
                      );
                  }
              );
          }
      );
  }



  void send_trajectory(
    const geometry_msgs::msg::Pose &pose,
    const std::string &ctrl,
    const std::string &traj_type,
    double traj_duration,
    double acc_duration,
    double total_time,
    double kp,
    int trajectory_len,
    const std::string &s_type,
    const std::string &cmd_interface,
    std::function<void()> on_complete)
  {
    ros2_kdl_package::action::ExecuteTrajectory::Goal goal;
    goal.end_position_x = pose.position.x;
    goal.end_position_y = pose.position.y;
    goal.end_position_z = pose.position.z;
    goal.ctrl = ctrl;
    goal.traj_type = traj_type;
    goal.traj_duration = traj_duration;
    goal.acc_duration = acc_duration;
    goal.total_time = total_time;
    goal.kp = kp;
    goal.trajectory_len = trajectory_len;
    goal.s_type = s_type;
    goal.cmd_interface = cmd_interface;

    auto send_goal_options = rclcpp_action::Client<ros2_kdl_package::action::ExecuteTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [on_complete](auto) {
        if (on_complete) on_complete();
    };

    trajectory_client_->async_send_goal(goal, send_goal_options);
  }

  void send_gripper(const std::vector<double> &positions, std::function<void()> on_complete)
  {
    using FollowJT = control_msgs::action::FollowJointTrajectory;
    FollowJT::Goal goal;
    goal.trajectory.joint_names = {"joint_a8_left_finger_joint", "joint_a8_right_finger_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = positions;
    p.time_from_start = rclcpp::Duration(2s);
    goal.trajectory.points.push_back(p);

    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    send_goal_options.result_callback = [on_complete](auto) {
        if (on_complete) on_complete();
    };

    gripper_client_->async_send_goal(goal, send_goal_options);
  }

  geometry_msgs::msg::Pose make_pose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    return p;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDispatcher>());
  rclcpp::shutdown();
  return 0;
}