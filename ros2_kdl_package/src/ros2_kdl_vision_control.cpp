// Copyright  (C)  2007  Francois Cauwe
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include <Eigen/Geometry> // For Eigen::Quaterniond

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
        : Node("ros2_kdl_vision_control")
    {
        // Declare the parameter cmd_interface (position, velocity, effort)
        declare_parameter("cmd_interface", "position"); // Default is "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            return;
        }

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_pose_available_ = false;

        // Declare the parameter "task"
        declare_parameter("task", "positioning"); // Default is "positioning"
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(), "Current task is: '%s'", task_.c_str());

        // Validate the task parameter
        if (!(task_ == "positioning" || task_ == "look-at-point"))
        {
            RCLCPP_ERROR(get_logger(), "Invalid task selected! Use 'positioning' or 'look-at-point' instead.");
            return;
        }

        // Retrieve the robot_description parameter
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Create the KDLRobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
        {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Create the joint arrays
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96; // TODO: Read from URDF file
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;         // TODO: Read from URDF file
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscribe to joint states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Subscribe to the ArUco marker pose
        aruco_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

        // Wait for the joint_states topic
        while (!joint_state_available_)
        {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Update the KDLRobot object
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Calculate the EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute inverse kinematics
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize the controller
        KDLController controller_(*robot_);

        // Initial trajectory EE position (offset only)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1));

        // Final trajectory EE position (dynamic)
        Eigen::Vector3d end_position;

        // Set end_position based on the task
        if (task_ == "positioning")
        {
            end_position << 0.6, -0.5, 0.5;
            end_position[0] += 0.3; // Add an offset of +0.3 in x
        }
        else
        {
            end_position << init_position[0], -init_position[1], init_position[2];
        }

        // Define initial orientation (assumed identity)
        Eigen::Quaterniond orientationInit = Eigen::Quaterniond::Identity();

        // Define final orientation (tag orientation in RPY)
        double roll = 0.0;
        double pitch = -2.2; // Offset
        double yaw = 0.0;

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond orientationEnd = yawAngle * pitchAngle * rollAngle;

        // Plan the trajectory
        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, orientationInit, orientationEnd);

        // Retrieve the first trajectory point
        trajectory_point p = planner_.compute_trajectory(t);

        // Compute errors
        Eigen::Vector3d pos_error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));

        // Add an attribute for the initial position
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        Eigen::VectorXd q_init_; // Initial joint configuration

        // Create the publisher for commands based on the interface
        if (cmd_interface_ == "position")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }
        else if (cmd_interface_ == "velocity")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        }
        else if (cmd_interface_ == "effort")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // Create the timer for the control loop
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&Iiwa_pub_sub::cmd_publisher, this));

        // Initialize the desired commands
        desired_commands_.resize(nj, 0.0);
    }

private:
    void cmd_publisher()
    {
        iteration_ += 1;
        // Define trajectory
        double total_time = 1.5;  // Total trajectory duration
        int trajectory_len = 150; // Length of the trajectory
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;

        if (task_ == "positioning")
        {
            // Code for the positioning task
            t_ += dt;

            if (t_ < total_time)
            {
                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);

                // Compute the EE frame
                KDL::Frame cartpos = robot_->getEEFrame();

                // Compute the desired frame
                KDL::Frame desFrame;
                desFrame.M = KDL::Rotation::Quaternion(
                    p.orientation.x(), p.orientation.y(), p.orientation.z(), p.orientation.w());
                desFrame.p = toKDL(p.pos);

                // Compute errors
                Eigen::Vector3d pos_error = computeLinearError(toEigen(desFrame.p), toEigen(cartpos.p));
                Eigen::Vector3d ori_error = computeOrientationError(toEigen(desFrame.M), toEigen(cartpos.M));
                std::cout << "The error norm is: " << pos_error.norm() << std::endl;

                // Combine errors for control
                Vector6d total_error;
                total_error.head<3>() = pos_error;
                total_error.tail<3>() = ori_error;

                // Control gains (adjust these values appropriately)
                double Kp_pos = 5.0;
                double Kp_ori = 2.0;

                if (cmd_interface_ == "velocity")
                {
                    // Use Jacobian to compute joint velocities to reduce error
                    Vector6d cart_vel;
                    cart_vel.head<3>() = p.vel + Kp_pos * pos_error;
                    cart_vel.tail<3>() = Kp_ori * ori_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;

                    for (size_t i = 0; i < joint_velocities_cmd_.rows(); ++i)
                    {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }

                // Update the KDL robot structure
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

                // Create the message and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

                // Send joint velocity commands to zero
                for (size_t i = 0; i < joint_velocities_.rows(); ++i)
                {
                    desired_commands_[i] = 0.0;
                }

                // Create the message and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }
        else if (task_ == "look-at-point")
        {
            // Call the function to compute the control law
            compute_look_at_point_control();
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < msg->position.size(); i++)
        {
            joint_positions_.data[i] = msg->position[i];
            joint_velocities_.data[i] = msg->velocity[i];
        }
    }

void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Store the pose of the ArUco marker
    aruco_pose_ = KDL::Frame(
        KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
        KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)
    );

    // Update a variable indicating that the pose is available
    aruco_pose_available_ = true;
    q_init_ = robot_->getJntValues(); // Dimension (nj)

    // Store the position of the object relative to the camera
    Po_w_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "ArUco Position: [x: %.3f, y: %.3f, z: %.3f]",
        Po_w_.x(), Po_w_.y(), Po_w_.z());

    RCLCPP_INFO(this->get_logger(), "ArUco pose updated");
}

void compute_look_at_point_control()
{
    if (!aruco_pose_available_)
    {
        RCLCPP_WARN(this->get_logger(), "ArUco pose not available.");
        return;
    }

    // Update the KDL robot structure
    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

    // Get the current pose of the camera (end-effector)
    KDL::Frame T_c_w = robot_->getEEFrame(); // Current pose of the end-effector in the world frame
    
    Eigen::Vector3d Pc_w = toEigen(T_c_w.p); // Camera position in the world frame (located on the EE)

    // Orientation of the EE is different from the camera's; calculate camera orientation
    Eigen::Matrix3d R_ee_to_camera;
    R_ee_to_camera << 
        0,  0, -1,
        0, -1,  0,
       -1,  0,  0;

    Eigen::Matrix3d R_ee = toEigen(T_c_w.M);
    Eigen::Matrix3d Rc = R_ee * R_ee_to_camera;

    // Calculate the object's position in the camera frame
    Eigen::Vector3d Po_c = Po_w_;

    // Transform the marker position into the world frame
    Eigen::Vector3d Po_w = Rc * Po_c + Pc_w;

    // Calculate the normalized look-at vector
    Eigen::Vector3d s = (Po_w - Pc_w).normalized();

    /*debug
    RCLCPP_INFO(this->get_logger(), "Object Position in Camera Frame (Po_c): [x: %.3f, y: %.3f, z: %.3f]",
        Po_c.x(), Po_c.y(), Po_c.z());
    RCLCPP_INFO(this->get_logger(), "Look-At Vector (s): [x: %.3f, y: %.3f, z: %.3f]",
        s.x(), s.y(), s.z()); */

    // Compute S(s), the antisymmetric operator
    Eigen::Matrix3d S_s;
    S_s <<     0, -s(2),  s(1),
             s(2),     0, -s(0),
            -s(1),  s(0),     0;

    // Compute L(s), the mapping matrix
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 6> Ls;
    Ls.block<3,3>(0,0) = - (1.0 / Po_c.norm()) * (I - s * s.transpose());
    Ls.block<3,3>(0,3) = S_s;

    // Compute R
    Eigen::Matrix<double, 6, 6> R;
    R.setZero();
    R.block<3,3>(0,0) = Rc;
    R.block<3,3>(3,3) = Rc;

    // Compute L
    Eigen::Matrix<double, 3, 6> L = Ls * R;

    // Compute J_c
    Eigen::MatrixXd Jc = robot_->getEEJacobian().data;

    // Compute L J_c
    Eigen::MatrixXd LJc = L * Jc;

    // Compute the error e
    Eigen::Vector3d sd(0.0, 0.0, 1.0); // Desired vector (camera's z-axis)
    Eigen::Vector3d e = sd.cross(s);
    double error_norm = e.norm();
    // Condizione di arresto
    /*if (error_norm < 0.01) {
        RCLCPP_INFO(this->get_logger(), "Error below threshold. Stopping movement.");
	    for (size_t i = 0; i < 7; ++i) {	
		desired_commands_[i] = 0.0;  }
        return;
    }*/

    // Compute (L J_c)^dagger
    Eigen::MatrixXd LJc_pinv = pseudoinverse(LJc);

    // Compute N
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(robot_->getNrJnts(), robot_->getNrJnts()) - LJc_pinv * LJc;

    double k_0 = 5;
    Eigen::VectorXd q_current = robot_->getJntValues();
    Eigen::VectorXd q_dot_0 = -k_0 * (q_current - q_init_);

    // std::cout << "q_current values: " << q_current.transpose() << std::endl;
    // std::cout << "q_init_ values: " << q_init_.transpose() << std::endl;

    double k = 2; // Proportional gain
    Eigen::VectorXd q_dot = k * LJc_pinv * e + N * q_dot_0;

    double max_speed = 0.5; // Limit joint speeds
    for (size_t i = 0; i < q_dot.size(); ++i)
    {
        if (q_dot(i) > max_speed) q_dot(i) = max_speed;
        else if (q_dot(i) < -max_speed) q_dot(i) = -max_speed;
    }

    // RCLCPP_INFO(this->get_logger(), "Desired Look-At Vector (sd): [x: %.3f, y: %.3f, z: %.3f]",
    //             sd.x(), sd.y(), sd.z());
    // RCLCPP_INFO(this->get_logger(), "Error Vector (e = sd - s): [x: %.3f, y: %.3f, z: %.3f]",
    //             e.x(), e.y(), e.z());
    RCLCPP_INFO(this->get_logger(), "Error Norm: %.6f", error_norm);

    for (size_t i = 0; i < q_dot.size(); ++i)
    {
        desired_commands_[i] = q_dot(i);
        RCLCPP_INFO(this->get_logger(), "q_dot[%zu]: %.6f", i, q_dot(i));
    }

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = desired_commands_;
    cmdPublisher_->publish(cmd_msg);
}

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_subscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> desired_commands_;
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    int iteration_;
    double t_;
    std::string cmd_interface_;

    // Additional variables
    KDL::Frame aruco_pose_;
    Eigen::Vector3d Po_w_;
    Eigen::VectorXd q_init_;
    bool joint_state_available_;
    bool aruco_pose_available_;   
    std::string task_;
    KDL::Frame init_cart_pose_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}

