
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

#ifndef CLOSED_LOOP_CPG_CONTROLLER_VELOCITY_H
#define CLOSED_LOOP_CPG_CONTROLLER_VELOCITY_H

#include "cpg.hpp"
#include <boost/date_time.hpp>
#include <chrono>
#include <cmath>
#include <controller_interface/controller.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/joint_command_interface.h>
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
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>

// Generate a trajectory using KDL
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <Eigen/Dense>

namespace closed_loop_cpg_controller_velocity {

    struct NoSafetyConstraints;

    /**
     * \brief Speed command controller for hexapod servos
     *
     * This class forwards the command signal down to a set of joints, if they
     * do not infringe some user-defined safety constraints.
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
    class ClCpgControllerV : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        ClCpgControllerV();
        ~ClCpgControllerV(){};

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

        std::array<int, NLegs> cartesian_to_joint(
            const std::array<KDL::JntArray, NLegs>& q_init,
            const std::array<KDL::Frame, NLegs>& frame,
            std::array<KDL::JntArray, NLegs>& q_out,
            bool stop_on_failure = false);

        /*!  Name of the joints defined in urdf and .yaml config file. Make the correspondance with ros_control JointHandles  */
        std::vector<std::string> joint_names;
        /*!  JointHandle objects recovered thanks to joint_names*/
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        /*!  number of joints */
        unsigned int n_joints;
        /*!  has_init will be set to true when all joint positions are at 0 +- e*/
        bool has_init;
        float e;
        /*!  cpg object go to cpg.hpp for more details*/
        cpg::CPG cpg_;

        /*!    angles of the soulder joints in the axial plane at init*/
        std::vector<float> X, Xprev, Xcommand;
        /*!    angles of the soulder joints in the sagittal plane at init*/
        std::vector<float> Y, Yprev, Ycommand;
        /*!    imu buffer  to get imu data from a topic ros*/
        realtime_tools::RealtimeBuffer<std::vector<double>> imu_buffer;
        /*!    tf buffer  to get tf data from a topic ros*/
        realtime_tools::RealtimeBuffer<std::vector<double>> tf_buffer;
        tf::Quaternion quat;
        tf::Matrix3x3 P;
        tf::Matrix3x3 R;
        Eigen::Matrix<float, 3, 3> _R_eigen;
        Eigen::Matrix<float, 3, 3> _P_eigen;
        Eigen::Matrix<float, 3, NLegs> _delta_theta_e;
        geometry_msgs::Vector3 rpy;
        tf::Vector3 x_robot_orientation;
        float theta_z;
        std::vector<tf::Transform> end_effectors_transform;
        tf::Transform leg_end_effector_static_transforms;

        std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, NLegs> _tracik_solvers;
        // used for forward kinematics
        // It is initialized only when make_fk_solvers is first called
        std::array<std::shared_ptr<KDL::ChainFkSolverPos_recursive>, NLegs> _fk_solvers;

        KDL::Twist _bounds;

        std::array<KDL::JntArray, NLegs> _q_init;
        std::array<KDL::JntArray, NLegs> _q_out;
        std::array<KDL::Frame, NLegs> _frame;
        std::vector<int>
            _leg_map_to_paper;

        Eigen::Matrix<float, 3, NLegs> _r;
        Eigen::Matrix<float, 3, NLegs> _r_tilde;

    private:
        SafetyConstraint _constraint;

        /**
         * \brief Put the servos at the position 0 at init
         */
        void initJointPosition();
        /*!  Subscriber to recover imu data*/
        ros::Subscriber _sub_imu;
        /*!  Subscriber to recover tf data*/
        ros::Subscriber _sub_tf;
        /**
         * \brief callback to recover imu data
         */
        void imuCB(const sensor_msgs::ImuConstPtr& msg);
        /**
         * \brief callback to recover end effector position
         */
        void tfCB(const tf2_msgs::TFMessageConstPtr& msg);
    }; // class ClCpgControllerV

    /**
     * \brief Constructor
     */
    template <class SafetyConstraint, int NLegs>
    ClCpgControllerV<SafetyConstraint, NLegs>::ClCpgControllerV()
    {
        e = 0.05;
        has_init = false;
        X.resize(NLegs, 0.0);
        Xprev.resize(NLegs, 0.0);
        Xcommand.resize(NLegs, 0.0);
        Y.resize(NLegs, 0.0);
        Yprev.resize(NLegs, 0.0);
        Ycommand.resize(NLegs, 0.0);
        end_effectors_transform.resize(NLegs, tf::Transform());
        leg_end_effector_static_transforms = tf::Transform(tf::Quaternion(0.707106781188, 0.707106781185, -7.31230107717e-14, -7.3123010772e-14), tf::Vector3(0.0, 0.03825, -0.115));
        _leg_map_to_paper = {0, 2, 4, 5, 3, 1};
        std::string chain_start = "base_link";
        std::vector<std::string> chain_ends;
        float timeout = 0.005;
        float eps = 1e-5;
        std::string urdf = "robot_description";

        // Construct the objects doing inverse kinematic computations
        for (size_t leg = 0; leg < NLegs; ++leg) {
            chain_ends.push_back("force_sensor_" + std::to_string(_leg_map_to_paper[leg]));
            std::cout << "force_sensor_" + std::to_string(_leg_map_to_paper[leg]) << std::endl;
            // This constructor parses the URDF loaded in rosparm urdf_param into the
            // needed KDL structures.
            _tracik_solvers[leg] = std::make_shared<TRAC_IK::TRAC_IK>(
                chain_start, chain_ends[leg], urdf, timeout, eps);
        }

        // Set rotational bounds to max, so that the kinematic ignores the rotation
        // part of the target pose
        _bounds = KDL::Twist::Zero();
        _bounds.rot.x(std::numeric_limits<float>::max());
        _bounds.rot.y(std::numeric_limits<float>::max());
        _bounds.rot.z(std::numeric_limits<float>::max());

    } // namespace closed_loop_cpg_controller_velocity

    /**
     * \brief Tries to recover n_joints JointHandles
     */
    template <class SafetyConstraint, int NLegs>
    bool ClCpgControllerV<SafetyConstraint, NLegs>::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
    {
        // List of controlled joints
        std::string param_name = "joints";
        if (!nh.getParam(param_name, joint_names)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }
        n_joints = joint_names.size();

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
        // Safety Constraint
        if (!_constraint.init(joints, nh)) {
            ROS_ERROR_STREAM("Initialisation of the safety contraint failed");
            return false;
        }
        _sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &ClCpgControllerV<SafetyConstraint, NLegs>::imuCB, this);
        _sub_tf = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &ClCpgControllerV<SafetyConstraint, NLegs>::tfCB, this);
        return true;
    }

    /**
     * \brief This function is the control loop itself, called periodically by the controller_manager
     */
    template <class SafetyConstraint, int NLegs>
    void ClCpgControllerV<SafetyConstraint, NLegs>::update(const ros::Time& /*time*/, const ros::Duration& period)
    {
        std::vector<double>& imu_quat = *imu_buffer.readFromRT();

        if (imu_quat.size() == 4) {
            //Recover quaternion current orientation
            quat = tf::Quaternion(imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]);
            //Recover orientation matrix of the robot in the world frame
            P = tf::Matrix3x3(quat);
            P.getRPY(rpy.x, rpy.y, rpy.z);
            R = tf::Matrix3x3(tf::Quaternion(tf::Vector3(0, 0, 1), rpy.z)) * P.transpose();
            // std::cout << " a\n";
            for (unsigned int i = 0; i < 3; i++) {
                for (unsigned int j = 0; j < 3; j++) {
                    _P_eigen(i, j) = P[i].m_floats[j];
                    _R_eigen(i, j) = R[i].m_floats[j];
                    // std::cout << "_R_eigen" << _R_eigen(i, j) << std::endl;
                    // std::cout << "_P_eigen" << _P_eigen(i, j) << std::endl;
                }
            }
        }

        //Set the servo positions to 0 at the beginning because the init and starting function are not called long enough to do that
        if (has_init == false) {
            initJointPosition();
        }
        //Then carry on the cpg control
        else {

            /* Read the position values from the servos*/
            KDL::JntArray array(3);
            int sign = 1;
            for (unsigned int i = 0; i < NLegs; i++) {
                sign = (i < 3) ? 1 : -1;
                X[_leg_map_to_paper[i]] = sign * joints[i]->getPosition();
                Y[_leg_map_to_paper[i]] = joints[12 + i]->getPosition();
                array(0) = sign * joints[i]->getPosition();
                array(1) = joints[6 + i]->getPosition();
                array(2) = joints[12 + i]->getPosition();
                _q_init.at(_leg_map_to_paper[i]) = array;
                _r(0, i) = end_effectors_transform[i].getOrigin().m_floats[0];
                _r(1, i) = end_effectors_transform[i].getOrigin().m_floats[1];
                _r(2, i) = end_effectors_transform[i].getOrigin().m_floats[2];
                // std::cout << _r(0, i) << " " << _r(1, i) << " " << _r(2, i) << " " << i << " \n";
            }
            _r_tilde = _P_eigen.transpose() * _R_eigen * _P_eigen * _r;
            // std ::cout << _r_tilde << std::endl;
            std::array<KDL::Frame, NLegs> _r_tilde_frame;
            for (size_t leg = 0; leg < 6; leg++) {
                _r_tilde_frame.at(leg) = KDL::Frame(KDL::Vector(_r_tilde(0, leg), _r_tilde(1, leg), _r_tilde(2, leg)));
                // std::cout << _r(0, leg) - _r_tilde(0, leg) << " " << _r(1, leg) - _r_tilde(1, leg) << " " << _r(2, leg) - _r_tilde(2, leg) << " " << leg << " \n";
            }

            std::array<int, NLegs> status = ClCpgControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(_q_init, _r_tilde_frame, _q_out, false);

            for (unsigned int j = 0; j < NLegs; j++) {

                _delta_theta_e(0, j) = _q_init.at(j)(0) - _q_out.at(j)(0);
                _delta_theta_e(1, j) = _q_init.at(j)(1) - _q_out.at(j)(1);
                _delta_theta_e(2, j) = _q_init.at(j)(2) - _q_out.at(j)(2);
                std::cout << "_delta_theta_e(0," << j << ") :" << _delta_theta_e(0, j) << std::endl;
                std::cout << "_delta_theta_e(1," << j << ") :" << _delta_theta_e(1, j) << std::endl;
                std::cout << "_delta_theta_e(2," << j << ") :" << _delta_theta_e(2, j) << std::endl;
                std::cout << status[j] << std::endl;
                std::cout << "\n";
            }
            /*compute X,Y derivatives*/
            std::vector<std::pair<float, float>>
                XYdot = cpg_.computeXYdot(X, Y);

            for (unsigned int i = 0; i < XYdot.size(); i++) {

                if (std::abs(XYdot[i].first) > 0.5) {
                    std::cout << (std::abs(XYdot[i].first) / XYdot[i].first) << std::endl;
                    XYdot[i].first = 0.5 * (std::abs(XYdot[i].first) / XYdot[i].first);
                }

                if (std::abs(XYdot[i].second) > 0.5) {
                    XYdot[i].second = 0.5 * (std::abs(XYdot[i].second) / XYdot[i].second);
                }
            }
            /* Send velocity command */
            for (unsigned int i = 0; i < NLegs; i++) {
                sign = (i < 3) ? 1 : -1;
                joints[i]->setCommand(sign * XYdot[_leg_map_to_paper[i]].first);
                joints[6 + i]->setCommand(1 * XYdot[_leg_map_to_paper[i]].second);
                joints[12 + i]->setCommand(1 * XYdot[_leg_map_to_paper[i]].second);
            }
        }
        _constraint.enforce(period);
    }
    /**
    * \brief Put the servos at the position 0 at init
    */
    template <class SafetyConstraint, int NLegs>
    void ClCpgControllerV<SafetyConstraint, NLegs>::initJointPosition()
    {

        unsigned int count = 0;
        for (unsigned int i = 0; i < n_joints; i++) {
            joints[i]->setCommand(-0.3 * (joints[i]->getPosition()));
            if (std::abs(joints[i]->getPosition()) < e) {
                count++; //when the position 0 is reached do count++
            }
            if (count == n_joints) {
                has_init = true; //When every servo is around the 0 position stop the init phase and go on with cpg control
            }
        }
    }
    template <class SafetyConstraint, int NLegs>
    void ClCpgControllerV<SafetyConstraint, NLegs>::imuCB(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_buffer.writeFromNonRT({msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
    }

    template <class SafetyConstraint, int NLegs>
    void ClCpgControllerV<SafetyConstraint, NLegs>::tfCB(const tf2_msgs::TFMessageConstPtr& msg)
    {
        std::vector<tf::Transform> transformStamped(msg->transforms.size(), tf::Transform());
        for (unsigned int i = 0; i < msg->transforms.size(); i++) {
            tf::transformMsgToTF(msg->transforms[i].transform, transformStamped[i]);
        }
        for (unsigned int j = 0; j < NLegs; j++) {
            end_effectors_transform[_leg_map_to_paper[j]] = transformStamped[j] * transformStamped[6 + 2 * j] * transformStamped[7 + 2 * j] * leg_end_effector_static_transforms;
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
    std::array<int, NLegs> ClCpgControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(
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

} // namespace closed_loop_cpg_controller_velocity

#endif
