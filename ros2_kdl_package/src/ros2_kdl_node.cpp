// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
                 
        using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;
        rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"),
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {

            declare_parameter("traj_duration", 4.0);
            declare_parameter("acc_duration", 0.5);
            declare_parameter("total_time", 5.0);
            declare_parameter("trajectory_len", 150);
            declare_parameter("Kp", 5.0);
            declare_parameter("end_position_x", 0.28);
            declare_parameter("end_position_y", -0.01);
            declare_parameter("end_position_z", 0.03);
            declare_parameter("ctrl", "velocity_ctrl");
            declare_parameter("cmd_interface", "velocity");
            declare_parameter("traj_type", "linear"); 
            declare_parameter("s_type", "trapezoidal");
            declare_parameter("only_server", true);

            
 
            get_parameter("traj_duration", traj_duration_);
            get_parameter("acc_duration", acc_duration_);
            get_parameter("total_time", total_time_);
            get_parameter("trajectory_len", trajectory_len_);
            get_parameter("Kp", Kp_);
            get_parameter("end_position_x", end_pos_x_);
            get_parameter("end_position_y", end_pos_y_);
            get_parameter("end_position_z", end_pos_z_);
            get_parameter("ctrl", ctrl_);
            get_parameter("cmd_interface", cmd_interface_); 
            get_parameter("traj_type", traj_type_);
            get_parameter("s_type", s_type_);
            get_parameter("only_server", only_server);


            if(!only_server){
                RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

                if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
                {
                    RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
                }
                
                RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
                if (!(traj_type_ == "linear" || traj_type_ == "circular"))
                {
                    RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
                }

                RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
                if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
                {
                    RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
                }
                RCLCPP_INFO(get_logger(), "Selected control mode: '%s'", ctrl_.c_str());
            }



            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(10s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

         
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            

            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;  
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;   
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

          
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

           
            vision_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10,
                std::bind(&Iiwa_pub_sub::vision_target_callback, this, std::placeholders::_1)
            );

            
            while(!joint_state_available_){
                rclcpp::spin_some(node_handle_);
            }

          
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
          
            init_cart_pose_ = robot_->getEEFrame();

            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            
            controller_ = std::make_shared<KDLController>(*robot_);

            if(only_server){
                action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
                this,
                "execute_trajectory",
                std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
                );
                cmd_pub_position_ = create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);

                cmd_pub_velocity_ = create_publisher<FloatArray>("/velocity_controller/commands", 10);

                cmd_pub_effort_ = create_publisher<FloatArray>("/effort_controller/commands", 10);
            }else{
                
                Eigen::Vector3d init_position(robot_->getEEFrame().p.data);

                Eigen::Vector3d end_position;
                end_position << end_pos_x_, end_pos_y_, end_pos_z_;

                double traj_duration = traj_duration_, acc_duration = acc_duration_, traj_radius = 0.15;

                if(traj_type_ == "linear"){
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); 
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }
                    
                if(cmd_interface_ == "position"){

                    cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){

                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                

                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){

                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                

                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 


            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            }
        }

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &,
            std::shared_ptr<const ExecuteTrajectory::Goal> goal)
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

        }

        rclcpp_action::CancelResponse handle_cancel(
            [[maybe_unused]] const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
        {
            std::thread{std::bind(&Iiwa_pub_sub::execute_trajectory, this, goal_handle)}.detach();
        }

        void vision_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            vision_target_pos_ << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;

            Eigen::Quaterniond q(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            );

            last_aruco_msg_time_ = this->now().seconds(); 
            vision_target_available_ = true;
        }






    private:


        void cmd_publisher(){

            iteration_ = iteration_ + 1;


            double total_time = total_time_; 
            int trajectory_len = trajectory_len_; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            double Kp = Kp_;
            if ((this->now().seconds() - last_aruco_msg_time_) > aruco_timeout_) {
            vision_target_available_ = false;
            }

            t_+=dt;

            if (t_<total_time_ || ctrl_=="vision_ctrl"){

               
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }


                KDL::Frame cartpos = robot_->getEEFrame();           


                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 


                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                if(ctrl_ != "vision_ctrl"){
                    std::cout << "The error norm is : " << error.norm() << std::endl;
                }


                if(cmd_interface_ == "position"){

                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 


                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){

                    if(ctrl_ == "velocity_ctrl"){

                    Vector6d cartvel; cartvel << p_.vel + Kp*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    }
                    else if(ctrl_ == "velocity_ctrl_null"){

                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(p_.pos, Kp_, joint_positions_.data);
                    }
                else if (ctrl_ == "vision_ctrl")
                {
                     if(vision_target_available_)
                    {

                        joint_velocities_cmd_.data = controller_->velocity_ctrl_vision(
                            vision_target_pos_,  
                            joint_positions_.data, 
                            Kp_                   
                        );
                    }
                    else{
                        joint_velocities_cmd_.data.setZero(joint_velocities_cmd_.data.size());
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No poses available!");
                    }


                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = toStdVector(joint_velocities_cmd_.data);
                    cmdPublisher_->publish(cmd_msg);
                    
                }


                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }


                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){

                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){

                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){

                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 


                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

              
            }
            else{
                if(cmd_interface_ == "position"){

                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){

                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){

                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }

        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < 7; i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }


        void execute_trajectory(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
            auto result = std::make_shared<ExecuteTrajectory::Result>();

            double traj_duration = goal->traj_duration;
            double acc_duration = goal->acc_duration;
            double total_time = goal->total_time;
            int trajectory_len = goal->trajectory_len;
            double Kp = goal->kp;

            double end_pos_x = goal->end_position_x;
            double end_pos_y = goal->end_position_y;
            double end_pos_z = goal->end_position_z;

            std::string cmd_interface = goal->cmd_interface;
            rclcpp::Publisher<FloatArray>::SharedPtr active_pub;
            if (cmd_interface == "position")
                active_pub = cmd_pub_position_;
            else if (cmd_interface == "velocity")
                active_pub = cmd_pub_velocity_;
            else if (cmd_interface == "effort")
                active_pub = cmd_pub_effort_;
            
            std::string ctrl = goal->ctrl;

            std::string traj_type = goal->traj_type;
            std::string s_type = goal->s_type;

            double t = 0.0;
            double dt = total_time / trajectory_len;

            Eigen::Vector3d init_position(robot_->getEEFrame().p.data);
            Eigen::Vector3d end_position(end_pos_x, end_pos_y, end_pos_z);

            Eigen::VectorXd q_nominal(7);
            Eigen::VectorXd K_null(7);
            q_nominal << 0.0, -0.2854, 0.0, 1.2962, 0.0, 0.0, 0.0;
            K_null << 3.0, 3.0, 3.0, 3.0, 3.0, 0.0, 3.0;



            KDLPlanner planner;

            double traj_radius = 0.15;

            if (traj_type == "linear") {
                planner = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
            }
            else if (traj_type == "circular") {
                planner = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
            }

            while (rclcpp::ok() && ( t < total_time || ctrl == "vision_ctrl")) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Goal canceled";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                trajectory_point p;
                if(traj_type == "linear") {
                    if(s_type == "trapezoidal") {
                        p = planner.linear_traj_trapezoidal(t);
                    } else if(s_type == "cubic") {
                        p = planner.linear_traj_cubic(t);
                    }
                } else if(traj_type == "circular") {
                    if(s_type == "trapezoidal") {
                        p = planner.circular_traj_trapezoidal(t);
                    } else if(s_type == "cubic") {
                        p = planner.circular_traj_cubic(t);
                    }
                }

                KDL::Frame cartpos = robot_->getEEFrame();
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

                feedback->position_error = error.norm();
                goal_handle->publish_feedback(feedback);

                if(cmd_interface == "position") {
                    KDL::Frame nextFrame;
                    nextFrame.M = cartpos.M;
                    nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(Kp * error)) * dt;

                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);

                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface == "velocity") {
                    if(ctrl == "velocity_ctrl") {
                        Eigen::MatrixXd J = robot_->getEEJacobian().data;       
                        Eigen::MatrixXd J_pinv = pseudoinverse(J.topRows(3));  
                        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(7,7) - J_pinv * J.topRows(3);

                        Eigen::VectorXd q_current = joint_positions_.data;
                        Eigen::VectorXd qdot_task = J_pinv * (p.vel + Kp * error);
                        Eigen::VectorXd qdot_null = -K_null.array() * (q_current - q_nominal).array();

                        joint_velocities_cmd_.data = qdot_task + N * qdot_null;
                    }
                    else if(ctrl == "velocity_ctrl_null") {
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(p.pos, Kp, joint_positions_.data);
                    }
                    else if(ctrl == "vision_ctrl") {
                        if(vision_target_available_) {
                            joint_velocities_cmd_.data = controller_->velocity_ctrl_vision(
                                vision_target_pos_,
                                joint_positions_.data,
                                Kp
                            );
                        } else {
                            joint_velocities_cmd_.data.setZero(joint_velocities_cmd_.data.size());
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No poses available!");
                        }
                    }

                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface == "effort") {
                    joint_efforts_cmd_.data[0] = 0.1 * std::sin(2 * M_PI * t / total_time);
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                active_pub->publish(cmd_msg);

                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

                t += dt;
                rclcpp::Rate rate(1.0 / dt);
                rate.sleep();
            }

            result->success = true;
            result->message = "Trajectory completed successfully";
            goal_handle->succeed(result);

            if(cmd_interface == "position"){

                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){

                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
            }
            else if(cmd_interface_ == "effort"){

                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            active_pub->publish(cmd_msg);
            
        }


        
        Eigen::Vector3d vision_target_pos_;
        bool vision_target_available_ = false;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_target_sub_;
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmd_pub_position_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmd_pub_velocity_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmd_pub_effort_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        double last_aruco_msg_time_;
        double aruco_timeout_ = 0.5; 

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string ctrl_;
        bool only_server;
        

        KDL::Frame init_cart_pose_;

        double traj_duration_;
        double acc_duration_;
        double total_time_;
        int trajectory_len_;
        double Kp_;
        double end_pos_x_, end_pos_y_, end_pos_z_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}
