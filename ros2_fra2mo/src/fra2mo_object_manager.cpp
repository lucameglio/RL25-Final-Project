#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class Fra2MoObjectManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowJT = control_msgs::action::FollowJointTrajectory;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Fra2MoObjectManager() : Node("fra2mo_object_manager")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fra2mo/image_raw", 10,
            std::bind(&Fra2MoObjectManager::imageCallback, this, std::placeholders::_1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10,
            std::bind(&Fra2MoObjectManager::laserCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/fra2mo/odometry", 10,
            std::bind(&Fra2MoObjectManager::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        gripper_client_ = rclcpp_action::create_client<FollowJT>(
            this, "fra2mo/fra2mo_gripper_controller/follow_joint_trajectory");

        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        dispatcher_client_ = this->create_client<std_srvs::srv::Trigger>("dispatch_object");

        while (!nav2_client_->wait_for_action_server(10s) && rclcpp::ok()) {}
        while (!gripper_client_->wait_for_action_server(10s) && rclcpp::ok()) {}
        while (!dispatcher_client_->wait_for_service(10s) && rclcpp::ok()) {}

        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&Fra2MoObjectManager::visualServoControl, this));

        state_ = State::EXPLORING_ROTATE;
        target_detected_ = false;
        target_direction_ = 0.0;
        target_distance_ = 0.0;
        target_pixel_x_ = 0;
        last_log_time_ = 0.0;
        rotation_start_time_ = this->now();
        exploration_point_index_ = 0;
        
        exploration_points_ = {
            {2.5, 0.5},
            {2.5, 6.0},
            {-2.5, 6.0},
            {-2.5, 0.5}
        };
        
        RCLCPP_INFO(this->get_logger(), "Fra2Mo Object Manager ready - Advanced exploration mode");
    }

private:
    enum class State {
        EXPLORING_ROTATE,    
        EXPLORING_NAVIGATE,  
        APPROACHING,         
        GRASPING,          
        NAVIGATING_TO_DROP, 
        DROPPING,          
        CALLING_DISPATCHER
    };

   
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<FollowJT>::SharedPtr gripper_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr dispatcher_client_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<GoalHandleNav> current_nav_goal_handle_;

    State state_;
    bool target_detected_;
    float target_direction_;
    float target_distance_;
    int target_pixel_x_;
    nav_msgs::msg::Odometry current_odom_;
    double last_log_time_;

    rclcpp::Time rotation_start_time_;
    std::vector<std::pair<double, double>> exploration_points_;
    size_t exploration_point_index_;

    const double ROTATION_DURATION = 10.0;
    const float APPROACH_DISTANCE = 0.15f;
    const float MAX_LINEAR_VEL = 0.40f;
    const float MAX_ANGULAR_VEL = 0.3f;
    const float EXPLORE_ANGULAR_VEL = 0.628f;
    const float Kp_angular = 2.5f;
    const float Kp_linear = 0.6f;
    const float ANGULAR_THRESHOLD = 0.1f;
    int last_target_pixel_x_ = -1;
    const int MAX_PIXEL_JUMP = 30;


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (state_ != State::EXPLORING_ROTATE &&
            state_ != State::EXPLORING_NAVIGATE &&
            state_ != State::APPROACHING)
            return;

        cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat hsv;
        cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_green(35, 100, 50);
        cv::Scalar upper_green(85, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool found = false;
        double best_area = 0.0;
        int best_pixel_x = -1;

        for (auto &c : contours)
        {
            double area = cv::contourArea(c);
            if (area <= 50) continue;

            cv::Moments M = cv::moments(c);
            if (M.m00 == 0) continue;

            int px = int(M.m10 / M.m00);

            if (last_target_pixel_x_ >= 0 &&
                std::abs(px - last_target_pixel_x_) > MAX_PIXEL_JUMP)
                continue;

            if (area > best_area)
            {
                best_area = area;
                best_pixel_x = px;
                found = true;
            }
        }

        if (found)
        {
            target_pixel_x_ = best_pixel_x;
            last_target_pixel_x_ = target_pixel_x_;

            float image_center_x = cv_image.cols / 2.0f;
            float fov_h = 1.57f;

            float pixel_error = target_pixel_x_ - image_center_x;
            target_direction_ = -(pixel_error / image_center_x) * (fov_h / 2.0f);
        }
        else
        {
            last_target_pixel_x_ = -1;
        }

        bool was_detected = target_detected_;
        target_detected_ = found;

        if ((state_ == State::EXPLORING_ROTATE || state_ == State::EXPLORING_NAVIGATE) &&
            target_detected_ && !was_detected)
        {
            if (state_ == State::EXPLORING_NAVIGATE && current_nav_goal_handle_)
            {
                nav2_client_->async_cancel_goal(current_nav_goal_handle_);
                current_nav_goal_handle_.reset();
            }

            state_ = State::APPROACHING;
        }
    }


    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!target_detected_ || state_ != State::APPROACHING) return;

        int center_index = std::round((target_direction_ - msg->angle_min) / msg->angle_increment);
        int window = 10;
        float min_distance = msg->range_max;

        for (int i = center_index - window; i <= center_index + window; i++)
        {
            if (i >= 0 && i < (int)msg->ranges.size() && 
                !std::isnan(msg->ranges[i]) && !std::isinf(msg->ranges[i]))
            {
                min_distance = std::min(min_distance, msg->ranges[i]);
            }
        }
        target_distance_ = min_distance;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
    }

    void visualServoControl()
    {
        geometry_msgs::msg::Twist cmd;

        switch (state_)
        {
            case State::EXPLORING_ROTATE:
                exploreRotate(cmd);
                break;

            case State::EXPLORING_NAVIGATE:
                return;

            case State::APPROACHING:
                approachTarget(cmd);
                break;

            case State::GRASPING:
            case State::NAVIGATING_TO_DROP:
            case State::DROPPING:
            case State::CALLING_DISPATCHER:
                return;
        }

        cmd_vel_pub_->publish(cmd);
    }

    void exploreRotate(geometry_msgs::msg::Twist &cmd)
    {
        double elapsed = (this->now() - rotation_start_time_).seconds();
        
        if (elapsed < ROTATION_DURATION)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = EXPLORE_ANGULAR_VEL;
            
            double current_time = this->now().seconds();
            if (current_time - last_log_time_ > 2.0)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "EXPLORING_ROTATE: %.1f/%.1f seconds", elapsed, ROTATION_DURATION);
                last_log_time_ = current_time;
            }
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            
            RCLCPP_INFO(this->get_logger(), 
                "360° rotation completed, no target found. Moving to exploration point...");
            
            state_ = State::EXPLORING_NAVIGATE;
            navigateToExplorationPoint();
        }
    }


    void navigateToExplorationPoint()
    {
        auto [x, y] = exploration_points_[exploration_point_index_];
        exploration_point_index_ = (exploration_point_index_ + 1) % exploration_points_.size();
        
        RCLCPP_INFO(this->get_logger(), 
            "Navigating to exploration point: (%.2f, %.2f)", x, y);
        
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](std::shared_ptr<GoalHandleNav> goal_handle)
            {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    current_nav_goal_handle_ = goal_handle;
                    RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");
                }
            };
        
        send_goal_options.result_callback = 
            [this](const GoalHandleNav::WrappedResult & result)
            {
                current_nav_goal_handle_.reset();
                
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), 
                            "Reached exploration point! Starting new 360° rotation...");
                        state_ = State::EXPLORING_ROTATE;
                        rotation_start_time_ = this->now();
                        break;
                    
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), 
                            "Navigation cancelled (target detected!)");
                        break;
                    
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), 
                            "Navigation aborted! Trying next exploration point...");
                        state_ = State::EXPLORING_NAVIGATE;
                        navigateToExplorationPoint();
                        break;
                    
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
            };

        nav2_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void approachTarget(geometry_msgs::msg::Twist &cmd)
    {
		if (!target_detected_)
		{
			RCLCPP_WARN(this->get_logger(),
				"Target lost, returning to exploration");
			state_ = State::EXPLORING_ROTATE;
			return;
		}

        float angular_error = target_direction_;
        cmd.angular.z = std::clamp(Kp_angular * angular_error, 
                                   -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

        float distance_error = target_distance_ - APPROACH_DISTANCE;
        
		float alignment_factor = std::exp(-std::abs(angular_error) * 3.0);

		if (distance_error > 0.08 || std::abs(angular_error) > 0.1)
		{
			cmd.linear.x = std::clamp(Kp_linear * distance_error * alignment_factor,
									0.03f, MAX_LINEAR_VEL);
		}
		else
		{
			cmd.linear.x = 0.0;
			cmd.linear.z = 0.0;
			RCLCPP_INFO(this->get_logger(), "Target reached at %.2f m! Starting grasp sequence", target_distance_); 
			state_ = State::GRASPING; 
			startGraspSequence(); 
			return;
		}

        double current_time = this->now().seconds();
        if (current_time - last_log_time_ > 0.5)
        {
            RCLCPP_INFO(this->get_logger(), 
                "APPROACHING: dist=%.2fm, angle=%.2frad, dist_err=%.2f, ang_err=%.2f",
                target_distance_, target_direction_, distance_error, angular_error);
            last_log_time_ = current_time;
        }
    }

    void startGraspSequence()
    {
        RCLCPP_INFO(this->get_logger(), "Starting grasp sequence...");
        
        sendGripperGoalAsync(0.05, 0.05, 0.0, [this]()
        {
            sendGripperGoalAsync(0.05, 0.05, 0.15, [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Object grasped! Navigating to drop point...");
                state_ = State::NAVIGATING_TO_DROP;
                navigateToDropPoint();
            });
        });
    }

    void navigateToDropPoint()
    {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = now();
        goal_pose.pose.position.x = 0.1;
        goal_pose.pose.position.y = 3.0;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = [this](auto){
            RCLCPP_INFO(this->get_logger(), "Arrived at drop point! Starting drop sequence...");
            state_ = State::DROPPING;
            startDropSequence();
        };

        nav2_client_->async_send_goal(goal_msg, options);
    }

    void startDropSequence()
    {
        sendGripperGoalAsync(0.05, 0.05, 0.0, [this]()
        {
            sendGripperGoalAsync(0.0, 0.0, 0.0, [this]()
            {
                backOffFromIiwa(0.30); 
                state_ = State::CALLING_DISPATCHER;
                callDispatcherService();
            });
        });
    }

    void callDispatcherService()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        dispatcher_client_->async_send_request(request);
        
        std::this_thread::sleep_for(std::chrono::seconds(15));
        
        RCLCPP_INFO(this->get_logger(), 
            "Dispatcher completed. Resetting and resuming exploration...");
        state_ = State::EXPLORING_ROTATE;
        rotation_start_time_ = this->now();
        target_detected_ = false;
        target_distance_ = 0.0;
        target_direction_ = 0.0;
    }

    void sendGripperGoalAsync(double left, double right, double lift, std::function<void()> on_done)
    {
        FollowJT::Goal goal;
        goal.trajectory.joint_names = {"fra2mo_left_finger_joint", "fra2mo_right_finger_joint", "fra2mo_lift_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {left, right, lift};
        point.time_from_start = rclcpp::Duration(2s);

        goal.trajectory.points.push_back(point);

        auto options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
        options.result_callback = [on_done](auto){ if(on_done) on_done(); };

        gripper_client_->async_send_goal(goal, options);
    }
    
    void backOffFromIiwa(double distance = 0.30)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -0.15;
        cmd.angular.z = 0.0;

        double duration = distance / std::abs(cmd.linear.x);
        auto start_time = this->now();

        rclcpp::Rate rate(20);
        while ((this->now() - start_time).seconds() < duration && rclcpp::ok())
        {
            cmd_vel_pub_->publish(cmd);
            rate.sleep();
        }

        cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Fra2MoObjectManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}