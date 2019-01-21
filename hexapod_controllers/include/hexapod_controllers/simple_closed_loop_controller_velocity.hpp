
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  Copyright (c) 2018, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef SIMPLE_CLOSED_LOOP_CONTROLLER_VELOCITY_H
#define SIMPLE_CLOSED_LOOP_CONTROLLER_VELOCITY_H
#define TF_EULER_DEFAULT_ZYX
#include <boost/date_time.hpp>
#include <chrono>
#include <cmath>
#include <controller_interface/controller.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hexapod_controller/hexapod_controller_imu.hpp>
#include <iostream>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <math.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf_conversions/tf_eigen.h>
#include <trac_ik/trac_ik.hpp>
#include <vector>
// Trac_ik and KDL solver
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>
// Generate a trajectory using KDL
#include <Eigen/Dense>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <tf/transform_broadcaster.h>

namespace simple_closed_loop_controller_velocity {

    struct NoSafetyConstraints;

    /**
     * \brief Speed command controller for hexapod servos
     *
     * This class forwards the command signal down to a set of joints, if they
     * do not infringe some user-defined safety constraints.
     *
     * This controller is from the first part of the paper :
     *  Guillaume Sartoretti, Samuel Shaw, Katie Lam, Naixin Fan, Matthew J. Travers, Howie Choset:
     *  Central Pattern Generator With Inertial Feedback for Stable Locomotion and Climbing in Unstructured Terrain. ICRA 2018: 1-5$
     *
     * This controller uses on board IMU (Xsense)
     *
     * \tparam T class implementing the safety constraints
     *
     * \section ROS interface
     *
     * \param type hardware interface type.
     * \param joints Names of the joints to control.
     *
     * Subscribes to:
     * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
     */
    template <class SafetyConstraint = NoSafetyConstraints, int NLegs = 6>
    class SClControllerV : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        SClControllerV();
        ~SClControllerV(){};

        /**
         * \brief Tries to recover n_joints JointHandles
         */
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh);
        /**
         * \brief Starting is called one time after init and switch to update. It is enough  to read data but not to set servos position.
         * This needs to be done in update
         */
        void starting(const ros::Time& time){};
        /**
         * \brief This function is the control loop itself, called periodically by the controller_manager
         */
        void update(const ros::Time& /*time*/, const ros::Duration& period);

        /** Get joint angles from end-effector pose.

              @param q_init joint angles used to initialise the inverse kinematics
              @param frame target poses
              @param q_out resulting joint angles as found by inverse kinematics
              @param stop_on_failure if true, the function will return as soon as
                  the inverse kinematic computation fails for one leg (we walk
                  through them in ascending leg number).

              @return status of the computation of each leg; failed if < 0
          **/
        std::array<int, NLegs> cartesian_to_joint(const std::array<KDL::JntArray, NLegs>& q_init, const std::array<KDL::Frame, NLegs>& frame, std::array<KDL::JntArray, NLegs>& q_out, bool stop_on_failure = false);

        /*!  Name of the joints defined in urdf and .yaml config file. Make the correspondance with ros_control JointHandles  */
        std::vector<std::string> joint_names;
        /*!  JointHandle objects recovered thanks to joint_names*/
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;

        /*!  number of joints */
        unsigned int n_joints;
        /*!  has_init will be set to true when all joint positions are at 0 +- e*/
        bool has_init;
        /*! e is the error accepted at init */
        float e;

        int _mode;
        float _duration;
        int _isRunning;
        bool _isFinish;
        bool _isAvailable;
        bool _firstTraj;
        bool _secondTraj;
        double _time_count;
        double _current_time;
        double _loop_frequency;
        double _t;
        std::vector<double> _ctrl;

        /*!  cpg object go to cpg.hpp for more details*/
        cpg::CPG cpg_;
        //  hexapod_controller::HexapodControllerImu controller(ctrl, std::vector<int>());

        /*!    angles of the soulder joints in the axial plane at init*/
        std::vector<float> X, Xprev, Xcommand, XcommandSmoothed;
        /*!    angles of the soulder joints in the sagittal plane at init*/
        std::vector<float> Y, Yprev, Ycommand, YcommandSmoothed;
        /*!    imu buffer  to get imu data from a topic ros*/
        realtime_tools::RealtimeBuffer<std::vector<double>> imu_buffer;
        /*!    tf buffer  to get tf data from a topic ros*/
        realtime_tools::RealtimeBuffer<std::vector<double>> tf_buffer;
        /* quat is used to recover the data from the imu as quaternions */
        tf::Quaternion quat;
        /* P is the rotation matrix associated tp quat */
        tf::Matrix3x3 P;
        /* R is the rotation that brings the robot orientation from its current orientation P to the desired orientation (keeping the body level)*/
        tf::Matrix3x3 R;
        /*_R_eigen the conversion of R in eigen format */
        Eigen::Matrix<float, 3, 3> _R_eigen;
        /*_P_eigen the conversion of R in eigen format */
        Eigen::Matrix<float, 3, 3> _P_eigen;
        /* rpy : current roll pitch and yaw, extracted from raw P */
        geometry_msgs::Vector3 rpy;
        /*end_effectors_transform are the transformations between the base_link and the end effectors (tip of the legs)*/
        geometry_msgs::Vector3 rpy_tmp;
        std::vector<tf::Transform> end_effectors_transform;
        /*leg_end_effector_static_transforms are the end transforms between the last servo and the tip of the leg. Those are constant*/
        tf::Transform leg_end_effector_static_transforms;
        /*_tracik_solvers inverse kinematic solvers from trac_ik. There are NlLegs solvers, from base_link to the tip of the leg */
        std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, NLegs> _tracik_solvers;
        std::vector<KDL::ChainFkSolverPos_recursive> _kdl_fk_solvers;
        /*_bounds for _tracik_solvers init */
        KDL::Twist _bounds;
        /*_q_init are the current angles of each servos, for each leg */
        std::array<KDL::JntArray, NLegs> _q_init;
        /*_q_out are the desired angle position of each servos, for each leg */
        std::array<KDL::JntArray, NLegs> _q_out;
        /*_q_out are the desired angle position of each servos, for each leg */
        std::array<KDL::JntArray, NLegs> _q_out2;
        /*_frame are the desired end effector position and orientation to keep the body level */
        std::array<KDL::Frame, NLegs> _frame;
        /*_leg_map_to_paper is mapping the numbering of the legs from the cpg paper to the servo order taht we have on our platform*/
        std::vector<int> _leg_map_to_paper;
        /*end effector position for each leg, in the base_link frame (robot body frame) */
        Eigen::Matrix<float, 3, NLegs> _r;
        /* desired end effector position for each leg, in the base_link frame (robot body frame) */
        Eigen::Matrix<float, 3, NLegs> _r_tilde;
        /* _kp is the proportional gain to go to the desired joint angle position */
        float _kp;
        bool init2;
        //For debug only for the imu feedback convention
        //tf::TransformBroadcaster br;
        Eigen::Matrix<float, 1, NLegs> integrate_delta_thetax;
        Eigen::Matrix<float, 1, NLegs> integrate_delta_thetay;
        Eigen::Matrix<float, 3, NLegs> _delta_theta_e;
        std::vector<float> _kpitch;
        bool integration_has_diverged;
        std::vector<float> _kroll;
        std::vector<float> error;
        std::vector<float> error_prev;
        std::vector<float> error_derivated;
        std::vector<float> error_integrated;
        std::string param_name;

    private:
        SafetyConstraint _constraint;
        hexapod_controller::HexapodControllerImu _controller;
        /**
         * \brief Put the servos at the position 0 at init
         */
        void initJointPosition();
        void zero(float duration);
        void reset(float duration);
        void move(float duration, std::vector<double> ctrl);
        void relax(float duration);

        /*!  Subscriber to recover imu data*/
        ros::Subscriber _sub_imu;
        /*!  Subscriber to recover tf data*/
        ros::Subscriber _sub_tf;

        ros::NodeHandle _nh;
        /**
         * \brief callback to recover imu data
         */
        void imuCB(const sensor_msgs::ImuConstPtr& msg);
        /**
         * \brief callback to recover end effector position
         */
        void tfCB(const tf2_msgs::TFMessageConstPtr& msg);
    }; // class SClCpgControllerV

    /**
     * \brief Constructor
     */
    template <class SafetyConstraint, int NLegs>
    SClControllerV<SafetyConstraint, NLegs>::SClControllerV()
    {
        e = 0.05;
        has_init = false;
        X.resize(NLegs, 0.0);
        Xprev.resize(NLegs, 0.0);
        Xcommand.resize(NLegs, 0.0);
        // Xcommand = {0.01, 0.0, 0.0, 0.01, 0.01, 0};
        Y.resize(NLegs, 0.0);
        Yprev.resize(NLegs, 0);
        Ycommand.resize(NLegs, 0.0);
        XcommandSmoothed.resize(NLegs, 0.0);
        YcommandSmoothed.resize(NLegs, 0.0);

        _kpitch = {4, 0, -4, -4, 0, 4};
        _kroll = {4, 4, 4, -4, -4, -4};
        init2 = false;
        integration_has_diverged = false;
        end_effectors_transform.resize(NLegs, tf::Transform());
        leg_end_effector_static_transforms = tf::Transform(tf::Quaternion(0.707106781188, 0.707106781185, -7.31230107717e-14, -7.3123010772e-14), tf::Vector3(0.0, 0.03825, -0.115));
        _leg_map_to_paper = {0, 2, 4, 5, 3, 1};

        _kp = 1;

        _mode = -1;
        _duration = 0;
        _isRunning = false;
        _isFinish = false;
        _firstTraj = true;
        _secondTraj = false;
        _time_count = 0;
        _current_time = 0;
        _loop_frequency = 1000;
        _t = 0;
    } // namespace simple_closed_loop_cpg_controller_velocity

    /**
     * \brief Tries to recover n_joints JointHandles
     */
    template <class SafetyConstraint, int NLegs>
    bool SClControllerV<SafetyConstraint, NLegs>::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
    {
        _nh = nh;
        // List of controlled joints
        param_name = "joints";
        if (!nh.getParam(param_name, joint_names)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }
        n_joints = joint_names.size();
        error.resize(n_joints, 0.0);
        error_derivated.resize(n_joints, 0.0);
        error_integrated.resize(n_joints, 0.0);
        error_prev.resize(n_joints, 0.0);

        if (n_joints == 0) {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }
        for (unsigned int i = 0; i < n_joints; i++) {
            try {
                joints.push_back(
                    std::make_shared<hardware_interface::JointHandle>(
                        hw->getHandle(joint_names[i])));
                ROS_DEBUG_STREAM("Joint number " << i << ": " << joint_names[i]);
            }
            catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }
        }

        param_name = "mode";
        if (!nh.getParam(param_name, _mode)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }
        param_name = "duration";
        if (!nh.getParam(param_name, _duration)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }
        param_name = "isRunning";
        if (!nh.getParam(param_name, _isRunning)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }

        param_name = "ctrl";
        if (!nh.getParam(param_name, _ctrl)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }

        // Safety Constraint
        if (!_constraint.init(joints, nh)) {
            ROS_ERROR_STREAM("Initialisation of the safety contraint failed");
            return false;
        }
        // ros subscriber to recover imu data
        _sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &SClControllerV<SafetyConstraint, NLegs>::imuCB, this);
        // ros subscriber to recover end effector positions (tip of the legs positions)
        _sub_tf = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &SClControllerV<SafetyConstraint, NLegs>::tfCB, this);
        return true;
    }

    /**
     * \brief This function is the control loop itself, called periodically by the controller_manager
     */
    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::update(const ros::Time& /*time*/, const ros::Duration& period)
    {
        _current_time = _time_count * _loop_frequency;
        int prev_mode = _mode;
        param_name = "mode";
        if (!_nh.getParam(param_name, _mode)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        }

        param_name = "duration";
        if (!_nh.getParam(param_name, _duration)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        }
        param_name = "isRunning";
        if (!_nh.getParam(param_name, _isRunning)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        }

        param_name = "ctrl";
        if (!_nh.getParam(param_name, _ctrl)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        }
        for (unsigned int i = 0; i < n_joints; i++) {
            error_prev[i] = error[i];
        }
        //Read the imu buffer to recover imu data
        std::vector<double>& imu_quat = *imu_buffer.readFromRT();

        if (imu_quat.size() == 4) {
            //Recover current orientation
            quat = tf::Quaternion(imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]);
            P = tf::Matrix3x3(quat);
            P.getRPY(rpy.x, rpy.y, rpy.z);
        }

        if (_mode != prev_mode) {
            _isRunning = true;
            _nh.setParam("isRunning", _isRunning);
            _firstTraj = true;
        }

        switch (_mode) {
        case 0:
            std::cout << "zero " << std::endl;
            zero(_duration);
            break;
        case 1:
            std::cout << "reset " << std::endl;
            reset(_duration);
            break;
        case 2:
            std::cout << "move " << std::endl;
            move(_duration, _ctrl);
            break;
        case 3:
            std::cout << "relax " << std::endl;
            relax(_duration);
            break;
        }

        //Set the servo positions to 0 at the beginning because the init and starting function are not called long enough to do that
        // if (has_init == false) {
        //     initJointPosition();
        //     //_controller.set_parameters(_ctrl);
        // }
        // //Then carry on the imu control
        // else {
        //
        //     std::vector<float> joint_position;
        //     std::vector<float> joint_position_map;
        //
        //     bool is_broken = false; // We suppose at first that no leg is missing
        //     int i_broken = 0;
        //     /* Read the position values from the servos not used here*/
        //     for (unsigned int i = 0; i < n_joints; i++) {
        //         joint_position.push_back(joints[i]->getPosition());
        //     }
        //     /* remap the position values from the servos */
        //     for (size_t i = 0; i < 6; i++) {
        //         is_broken = false;
        //         //Check if the i th  leg is broker
        //         for (size_t j : _broken_legs) {
        //             if (i == j) {
        //                 is_broken = true;
        //             }
        //         }
        //         if (is_broken == false) {
        //             //if the leg is not broken fill in the joint_position in the right orger
        //             joint_position_map[3 * i] = joint_position[3 * i_broken]; //joint_position[i] is the first servo of leg i (shoulder joint)
        //             joint_position_map[3 * i + 1] = joint_position[3 * i_broken + 6]; //joint_position[i+6] is the second servo of leg i
        //             joint_position_map[3 * i + 2] = joint_position[3 * i_broken + 12]; //joint_position[i+12] is the third servo of leg i
        //             i_broken++;
        //         }
        //     }
        //     auto angles = _controller.pos(ros::Time);
        //     for (size_t i = 0; i < angles.size(); i++)
        //         _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];
        //     std::vector<float> cmd;
        //     /*compute CPG cmd*/
        //
        //     cpg_.computeCPGcmd();
        //     cmd = cpg_.computeErrors(-rpy.y, rpy.x, joint_position);
        //
        //     //  cmd.resize(3 * NLegs, 0.0);
        //
        //     /*send cmd*/
        //     int index = 0;
        //     for (unsigned int i = 0; i < NLegs; i++) {
        //         // XcommandSmoothed[_leg_map_to_paper[i]] = (sign * XYdot[_leg_map_to_paper[i]].first + XcommandSmoothed[_leg_map_to_paper[i]]) / 2;
        //         // YcommandSmoothed[_leg_map_to_paper[i]] = (XYdot[_leg_map_to_paper[i]].second + YcommandSmoothed[_leg_map_to_paper[i]]) / 2;
        //         joints[i]->setCommand(cmd[index]);
        //         index++;
        //         joints[6 + i]->setCommand(cmd[index]);
        //         index++;
        //         joints[12 + i]->setCommand(cmd[index]);
        //         index++;
        //     }
        // }
        _time_count++;

        _constraint.enforce(period);
    } // namespace simple_closed_loop_cpg_controller_velocity

    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::zero(float duration)
    {

        if (_isRunning == true) {
            unsigned int count = 0;
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(-_kp * (joints[i]->getPosition()));
                if (std::abs(joints[i]->getPosition()) < e) {
                    count++; //when the position 0 is reached do count++
                }
                if (count == n_joints) {
                    _isRunning = false;
                    _nh.setParam("isRunning", _isRunning);
                }
            }
        }
        else {
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(0);
            }
        }
    }

    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::reset(float duration)
    {
        if (_isRunning == true) {

            unsigned int count = 0;
            if (_firstTraj == true) {
                for (unsigned int i = 0; i < NLegs; i++) {
                    joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                    joints[i + 6]->setCommand(_kp * (M_PI_2 - joints[i + 6]->getPosition()));
                    joints[i + 12]->setCommand(_kp * ((256 * M_PI / 2048.0) - joints[i + 12]->getPosition()));
                    for (unsigned int j = 0; j < NLegs; j++) {
                        if ((std::abs(joints[j]->getPosition()) < e) && (std::abs(M_PI_2 - joints[j + 6]->getPosition()) < e) && (std::abs((256 * M_PI / 2048.0) - joints[j + 12]->getPosition()) < e)) {
                            count++; //when the position 0 is reached do count++
                            std::cout << count << std::endl;
                        }
                    }
                    if (count == NLegs) {
                        _firstTraj = false;
                        _secondTraj = true;
                    }
                }
            }
            count = 0;
            if (_secondTraj == true) {
                for (unsigned int i = 0; i < NLegs; i++) {
                    joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                    joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                    joints[i + 12]->setCommand(_kp * ((256 * M_PI / 2048.0) - joints[i + 12]->getPosition()));
                    for (unsigned int j = 0; j < NLegs; j++) {
                        if ((std::abs(joints[j]->getPosition()) < e) && (std::abs(M_PI_2 - joints[j + 6]->getPosition()) < e) && (std::abs((256 * M_PI / 2048.0) - joints[j + 12]->getPosition()) < e)) {
                            count++; //when the position 0 is reached do count++
                        }
                    }
                    if (count == n_joints) {
                        _secondTraj = false;
                        _isRunning = false;
                        _nh.setParam("isRunning", _isRunning);
                    }
                }
            }
        }
        else {
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(0);
            }
        }
    }

    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::move(float duration, std::vector<double> ctrl)
    {
        if (_isRunning == true) {
            _t += 0.1;
            ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
            unsigned int count = 0;
            hexapod_controller::HexapodControllerImu _controller(ctrl, std::vector<int>());
            std::vector<double> pos = _controller.pos(_t);

            for (unsigned int i = 0; i < NLegs; i++) {
                joints[i]->setCommand(_kp * (pos[3 * i] - joints[i]->getPosition()));
                joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
                joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));

                for (unsigned int j = 0; j < NLegs; j++) {
                    if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                        count++; //when the position 0 is reached do count++
                        std::cout << count << std::endl;
                    }
                }
            }
            if (_t > 5) {
                _isRunning = false;
                _nh.setParam("isRunning", _isRunning);
                _t = 0;
            }
        }

    } // namespace simple_closed_loop_controller_velocity

    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::relax(float duration)
    {
        duration = 2;
        if (_t > duration) {
            _t = duration;
        }
        if (_isRunning == true) {
            unsigned int count = 0;
            double a = (duration - _t) / duration;
            double b = _t / duration;
            _t += 0.001;
            for (unsigned int i = 0; i < NLegs; i++) {
                joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                joints[i + 6]->setCommand(_kp * ((a * M_PI_4 / 6.0 + b * (M_PI_2 * 1.2)) - joints[i + 6]->getPosition()));
                joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                for (unsigned int j = 0; j < NLegs; j++) {
                    if ((std::abs(joints[j]->getPosition()) < e) && (std::abs((a * M_PI_4 / 6.0 + b * (M_PI_2 * 1.2)) - joints[j + 6]->getPosition()) < e) && (std::abs(0 - joints[j + 12]->getPosition()) < e)) {
                        count++; //when the position 0 is reached do count++
                        std::cout << count << std::endl;
                    }
                }
                if (count == NLegs) {
                    _isRunning = false;
                    _nh.setParam("isRunning", _isRunning);
                    _t = 0;
                }
            }
        }
        else {
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(0);
                _t = 0;
            }
        }
    }
    /**
    * \brief Put the servos at the position 0 at init
    */
    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::initJointPosition()
    {

        if (_isRunning == 0) {
            unsigned int count = 0;
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(-_kp * (joints[i]->getPosition()));
                if (std::abs(joints[i]->getPosition()) < e) {
                    count++; //when the position 0 is reached do count++
                }
                if (count == n_joints) {
                    has_init = true;
                    _isRunning = 1;
                    for (unsigned int i = 0; i < n_joints; i++) {
                        joints[i]->setCommand(0);
                    }
                    //When every servo is around the 0 position stop the init phase and go on with cpg control
                }
            }
        }
        else {
            for (unsigned int i = 0; i < n_joints; i++) {
                joints[i]->setCommand(0);
            }
        }
    }
    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::imuCB(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_buffer.writeFromNonRT({msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
    }

    template <class SafetyConstraint, int NLegs>
    void SClControllerV<SafetyConstraint, NLegs>::tfCB(const tf2_msgs::TFMessageConstPtr& msg)
    {
        std::vector<tf::Transform> transformStamped(msg->transforms.size(), tf::Transform());
        for (unsigned int i = 0; i < msg->transforms.size(); i++) {
            tf::transformMsgToTF(msg->transforms[i].transform, transformStamped[i]);
        }
        for (unsigned int j = 0; j < NLegs; j++) {
            end_effectors_transform[j] = transformStamped[j] * transformStamped[6 + 2 * j] * transformStamped[7 + 2 * j] * leg_end_effector_static_transforms;
        }
    }

    /** Get joint angles from end-effector pose.

          @param q_init joint angles used to initialise the inverse kinematics
          @param frame target poses
          @param q_out resulting joint angles as found by inverse kinematics
          @param stop_on_failure if true, the function will return as soon as
              the inverse kinematic computation fails for one leg (we walk
              through them in ascending leg number).

          @return status of the computation of each leg; failed if < 0
      **/
    template <class SafetyConstraint, int NLegs>
    std::array<int, NLegs> SClControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(
        const std::array<KDL::JntArray, NLegs>& q_init,
        const std::array<KDL::Frame, NLegs>& frame,
        std::array<KDL::JntArray, NLegs>& q_out,
        bool stop_on_failure)
    {
        std::array<int, NLegs> status;

        for (size_t leg = 0; leg < NLegs; ++leg) {
            status[leg] = _tracik_solvers[leg]->CartToJnt(q_init[leg], frame[leg], q_out[leg], _bounds);
            if (stop_on_failure && status[leg] < 0)
                break;
        }

        return status;
    }
    /** \cond HIDDEN_SYMBOLS */
    struct NoSafetyConstraints {
        bool init(const std::vector<std::shared_ptr<hardware_interface::JointHandle>>& joints,
            ros::NodeHandle& nh)
        {
            return true;
        }
        bool enforce(const ros::Duration& period)
        {
            return true;
        }
        double consult(const ros::Duration& period)
        {
            return std::numeric_limits<double>::max();
        }
    };
    /** \endcond */

} // namespace simple_closed_loop_controller_velocity

#endif
