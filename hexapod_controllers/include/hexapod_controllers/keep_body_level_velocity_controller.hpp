
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

#ifndef keep_body_level_velocity_controller_H
#define keep_body_level_velocity_controller_H
#define TF_EULER_DEFAULT_ZYX
#include "cpg.hpp"
#include <boost/date_time.hpp>
#include <chrono>
#include <cmath>
#include <controller_interface/controller.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hexapod_controller/hexapod_controller_simple.hpp>
#include <iostream>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <math.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf_conversions/tf_eigen.h>
#include <trac_ik/trac_ik.hpp>
#include <vector>
// Trac_ik and KDL solver
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <trac_ik/trac_ik.hpp>
// Generate a trajectory using KDL
#include <Eigen/Dense>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

namespace keep_body_level_velocity_controller {

    struct NoSafetyConstraints;

    /**
     * \brief Velocity controller for the hexapod. This controller keep the hexapod body level as much as possible while keeping the tip of the legs at the same position.
     *
     * This class forwards the command signal down to a set of joints, if they
     * do not infringe some user-defined safety constraints.
     *
     * This controller is from the first part of the paper :
     *  Guillaume Sartoretti, Samuel Shaw, Katie Lam, Naixin Fan, Matthew J. Travers, Howie Choset:
     *  Central Pattern Generator With Inertial Feedback for Stable Locomotion and Climbing in Unstructured Terrain. ICRA 2018: 1-5$
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
    class KBLControllerV : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        KBLControllerV();
        ~KBLControllerV(){};

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

        // std::array<int, NLegs> cartesian_to_joint(std::array<KDL::ChainIkSolverPos_NR, NLegs> kdl_leg_chains, const std::array<KDL::JntArray, NLegs>& q_init, const std::array<KDL::Frame, NLegs>& frame, std::array<KDL::JntArray, NLegs>& q_out, bool stop_on_failure);
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
        /*!  cpg object go to cpg.hpp for more details*/
        cpg::CPG cpg_;
        /*!    imu buffer  to get imu data from a topic ros*/
        realtime_tools::RealtimeBuffer<std::vector<double>> imu_buffer;
        realtime_tools::RealtimeBuffer<std::vector<double>> joy_buffer;
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
        std::vector<tf::Transform> end_effectors_transform;
        /*leg_end_effector_static_transforms are the end transforms between the last servo and the tip of the leg. Those are constant*/
        tf::Transform leg_end_effector_static_transforms;
        /*_tracik_solvers inverse kinematic solvers from trac_ik. There are NlLegs solvers, from base_link to the tip of the leg */
        std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, NLegs> _tracik_solvers;
        /*_bounds for _tracik_solvers init */
        KDL::Twist _bounds;
        /*_q_init are the current angles of each servos, for each leg */
        std::array<KDL::JntArray, NLegs> _q_init;
        /*_q_out are the desired angle position of each servos, for each leg */
        std::array<KDL::JntArray, NLegs> _q_out;
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
        //For debug only for the imu feedback convention
        //tf::TransformBroadcaster br;

        //To use KDL instead of track_ik
        urdf::Model _robot_model;
        KDL::Tree _my_tree;
        // std::array<std::shared_ptr<KDL::ChainIkSolverPos_NR>, NLegs> _kdl_ik_solvers;

        bool _isRunning;
        std::vector<double> _ctrl;

        ros::Time _current_time;
        ros::Time _prev_time;
        double _t;

    private:
        SafetyConstraint _constraint;

        /**
         * \brief Put the servos at the position 0 at init
         */
        void initJointPosition();
        /*!  Subscriber to recover imu data*/
        ros::Subscriber _sub_imu;
        ros::Subscriber _sub_joy;

        /*!  Subscriber to recover tf data*/
        ros::Subscriber _sub_tf;
        /**
         * \brief callback to recover imu data
         */
        void imuCB(const sensor_msgs::ImuConstPtr& msg);
        void joyCB(const sensor_msgs::Joy::ConstPtr& msg);
        /**
         * \brief callback to recover end effector position
         */
        void tfCB(const tf2_msgs::TFMessageConstPtr& msg);

        void move(float duration, std::vector<double> ctrl);
        void move_back(float duration, std::vector<double> ctrl);
        void move_left(float duration, std::vector<double> ctrl);
        void turn_left(float duration, std::vector<double> ctrl);
        void turn_right(float duration, std::vector<double> ctrl);
    }; // class KBLControllerV

    /**
     * \brief Constructor
     */
    template <class SafetyConstraint, int NLegs>
    KBLControllerV<SafetyConstraint, NLegs>::KBLControllerV()
    {
      _isRunning = false;
        e = 0.05;
        has_init = false;
        end_effectors_transform.resize(NLegs, tf::Transform());
        leg_end_effector_static_transforms = tf::Transform(tf::Quaternion(0.707106781188, 0.707106781185, -7.31230107717e-14, -7.3123010772e-14), tf::Vector3(0.0, 0.03825, -0.115));
        _leg_map_to_paper = {0, 2, 4, 5, 3, 1};
        _ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
        // Init the IK solver
        std::string chain_start = "base_link";
        std::vector<std::string> chain_ends;
        float timeout = 0.005;
        float eps = 1e-5;
        std::string urdf = "robot_description"; // our urdf is loaded in the /robot_description topic

        // Construct the objects doing inverse kinematic computations
        for (size_t leg = 0; leg < NLegs; ++leg) {
            chain_ends.push_back("force_sensor_" + std::to_string(leg));
            std::cout << "force_sensor_" + std::to_string(leg) << std::endl;
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
        _kp = 3;

        //For KDL instead of track_ik
        _robot_model.initParam("/robot_description");
        kdl_parser::treeFromUrdfModel(_robot_model, _my_tree);
        std::vector<KDL::Chain> kdl_leg_chains;
        kdl_leg_chains.resize(NLegs, KDL::Chain());
        for (size_t leg = 0; leg < NLegs; ++leg) {
            bool error = _my_tree.getChain(chain_start, chain_ends[leg], kdl_leg_chains[leg]);
            // std::cout << error << " " << kdl_leg_chains[leg].getNrOfJoints() << kdl_leg_chains[leg].segments[0].getName() << " " << kdl_leg_chains[leg].segments[1].getName() << " " << kdl_leg_chains[leg].segments[2].getName() << std::endl;
            // KDL::ChainFkSolverPos_recursive fksolver1(kdl_leg_chains[leg]); //Forward position solver
            // KDL::ChainIkSolverVel_pinv iksolver1v(kdl_leg_chains[leg]); //Inverse velocity solver
            // // KDL::ChainIkSolverPos_NR iksolver1(kdl_leg_chains[leg], fksolver1, iksolver1v, 100, 1e-6);
            // _kdl_ik_solvers[leg] = std::make_shared<KDL::ChainIkSolverPos_NR>(kdl_leg_chains[leg], fksolver1, iksolver1v, 100, 1e-6);
        }
    } // namespace keep_body_level_velocity_controller

    /**
     * \brief Tries to recover n_joints JointHandles
     */
    template <class SafetyConstraint, int NLegs>
    bool KBLControllerV<SafetyConstraint, NLegs>::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
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
        // ros subscriber to recover imu data
        _sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &KBLControllerV<SafetyConstraint, NLegs>::imuCB, this);

        _sub_joy = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &KBLControllerV<SafetyConstraint, NLegs>::joyCB, this);
        // ros subscriber to recover end effector positions (tip of the legs positions)
        _sub_tf = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &KBLControllerV<SafetyConstraint, NLegs>::tfCB, this);

        _isRunning = true;

        return true;
    }

    /**
     * \brief This function is the control loop itself, called periodically by the controller_manager
     */
    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::update(const ros::Time& /*time*/, const ros::Duration& period)
    {
        //Read the imu buffer to recover imu data
        std::vector<double>& imu_quat = *imu_buffer.readFromRT();

        std::vector<double>& joy_cmd = *joy_buffer.readFromRT();
        std::cout << "Joy command : "<< std::endl;

        if (imu_quat.size() == 4) {
            //Recover current orientation
            quat = tf::Quaternion(imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]);
            P = tf::Matrix3x3(quat);
            P.getRPY(rpy.x, rpy.y, rpy.z);
            //Here we use the TF_EULER_DEFAULT_ZYX convention : the yaw is around z, the pitch is around y and the rooll is around x
            //Doing so we have a discrapency between the feedback from the imu and the TF_EULER_DEFAULT_ZYX convention
            //The roll is actually -pitch
            //The pitch is the roll
            //We create a new P which will follow the TF_EULER_DEFAULT_ZYX from ros convention
            tf::Quaternion tmp(rpy.z, rpy.x, -rpy.y);
            P = tf::Matrix3x3(tmp);
            //Create R, the rotation that will bring the hexapod body level
            //We only want to keep the yaw direction from P and bring the roll and pitch to 0
            R = tf::Matrix3x3(tf::Quaternion(tf::Vector3(0, 0, 1), rpy.z)) * P.transpose();
            //Other way to create R :
            // tf::Quaternion tmp0(0, -rpy.x, rpy.y);
            // R = tf::Matrix3x3(tmp0);

            //Uncomment to debug imu conventions
            //tf::Transform transformimu(P, tf::Vector3(0.0, 0.0, 0.0));
            //br.sendTransform(tf::StampedTransform(transformimu, ros::Time::now(), "base_link", "imu"));

            for (unsigned int i = 0; i < 3; i++) {
                for (unsigned int j = 0; j < 3; j++) {
                    _P_eigen(i, j) = P[i].m_floats[j];
                    _R_eigen(i, j) = R[i].m_floats[j];
                }
            }
        }

        //Set the servo positions to 0 at the beginning because the init and starting function are not called long enough to do that
        if (has_init == false) {
            initJointPosition();
        }
        //Then carry on the control
        else {

            /* Read the position values from the servos*/
            KDL::JntArray array(3); //Contains the 3 joint angles for one leg
            for (unsigned int i = 0; i < NLegs; i++) {
                array(0) = joints[i]->getPosition();
                array(1) = joints[6 + i]->getPosition();
                array(2) = joints[12 + i]->getPosition();
                _q_init.at(i) = array; // angles for the first leg
                _r(0, i) = end_effectors_transform[i].getOrigin().m_floats[0]; //x position of the tip of the first leg in the robot body frame
                _r(1, i) = end_effectors_transform[i].getOrigin().m_floats[1]; //y position of the tip of the first leg in the robot body frame
                _r(2, i) = end_effectors_transform[i].getOrigin().m_floats[2]; //z position of the tip of the first leg in the robot body frame
            }
            //desired end effector position, _r is expressed in the world frame then the rotation is applied and the we put everything in the body frame again.
            _r_tilde = _P_eigen.transpose() * _R_eigen * _P_eigen * _r;
            std::array<KDL::Frame, NLegs> _r_tilde_frame;

            for (size_t leg = 0; leg < 6; leg++) {
                // _r_tilde(2, leg) = -std::sqrt(std::abs(_r(0, leg) * _r(0, leg) + _r(1, leg) * _r(1, leg) + _r(2, leg) * _r(2, leg) - _r_tilde(0, leg) * _r_tilde(0, leg) - _r_tilde(1, leg) * _r_tilde(1, leg)));
                _r_tilde_frame.at(leg) = KDL::Frame(KDL::Vector(_r_tilde(0, leg), _r_tilde(1, leg), _r_tilde(2, leg)));
            }

            std::array<int, NLegs> status = KBLControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(_q_init, _r_tilde_frame, _q_out, false);
            // std::array<int, NLegs> status = KBLControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(_kdl_ik_solvers, _q_init, _r_tilde_frame, _q_out, false);

            /* Send velocity command */
            for (unsigned int i = 0; i < NLegs; i++) {
                //control the servos to go to the desired angle postions
                joints[i]->setCommand(-_kp * _q_out.at(i)(0));
                joints[6 + i]->setCommand(-_kp * 1 * _q_out.at(i)(1));
                joints[12 + i]->setCommand(-_kp * 1 * _q_out.at(i)(2));
            }

            if(joy_cmd.size()>2)
            {
              std::cout << joy_cmd[0] << " ; " << joy_cmd[1] << std::endl;
              if(joy_cmd[1] ==  1 )
              {
                move(2,_ctrl);
              }
              if(joy_cmd[1] ==  -1 )
              {
                move_back(2,_ctrl);
              }
              if(joy_cmd[0] ==  1 )
              {
                turn_left(2,_ctrl);
              }
              if(joy_cmd[0] ==  -1 )
              {
                turn_right(2,_ctrl);
              }
            }


        }


        _constraint.enforce(period);
    }
    /**
    * \brief Put the servos at the position 0 at init
    */
    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::initJointPosition()
    {

        unsigned int count = 0;
        for (unsigned int i = 0; i < n_joints; i++) {
            joints[i]->setCommand(-_kp * (joints[i]->getPosition()));
            if (std::abs(joints[i]->getPosition()) < e) {
                count++; //when the position 0 is reached do count++
            }
            if (count == n_joints) {
                has_init = true; //When every servo is around the 0 position stop the init phase and go on with cpg control
            }
        }
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::move(float duration, std::vector<double> ctrl)
    {
      _prev_time = _current_time;
      _current_time = ros::Time::now();
      //  std::cout << "time : " << _current_time - _prev_time << std::endl;

      if (_isRunning == true) {
          //  _t = current_tine
          _t += 0.01;
          //ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
          unsigned int count = 0;
          hexapod_controller::HexapodControllerSimple _controller(ctrl, std::vector<int>());
          //  _controller.computeErrors(rpy.x,rpy.y);
          std::vector<double> pos = _controller.pos(_t);
          //  std::cout << "time : " << _t << std::endl;
          if (_t > duration) {
              _isRunning = false;

              _t = 0;
              for (unsigned int i = 0; i < NLegs; i++) {

                  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                  for (unsigned int j = 0; j < NLegs; j++) {
                      if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                          count++; //when the position 0 is reached do count++
                          //  std::cout << count << std::endl;
                      }
                  }
              }
          }
          for (unsigned int i = 0; i < NLegs; i++) {
              joints[i]->setCommand(_kp * (pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
      }
      else {
          for (unsigned int i = 0; i < n_joints; i++) {
              joints[i]->setCommand(0);
          }
      }
      _isRunning = true;
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::move_back(float duration, std::vector<double> ctrl)
    {
      _prev_time = _current_time;
      _current_time = ros::Time::now();
      //  std::cout << "time : " << _current_time - _prev_time << std::endl;

      if (_isRunning == true) {
          //  _t = current_tine
          _t += 0.01;
          //ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
          unsigned int count = 0;
          hexapod_controller::HexapodControllerSimple _controller(ctrl, std::vector<int>());
          //  _controller.computeErrors(rpy.x,rpy.y);
          std::vector<double> pos = _controller.pos(_t);
          //  std::cout << "time : " << _t << std::endl;
          if (_t > duration) {
              _isRunning = false;

              _t = 0;
              for (unsigned int i = 0; i < NLegs; i++) {

                  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                  for (unsigned int j = 0; j < NLegs; j++) {
                      if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                          count++; //when the position 0 is reached do count++
                          //  std::cout << count << std::endl;
                      }
                  }
              }
          }
          for (unsigned int i = 0; i < NLegs; i++) {
              joints[i]->setCommand(_kp * (-pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
      }
      else {
          for (unsigned int i = 0; i < n_joints; i++) {
              joints[i]->setCommand(0);
          }
      }
      _isRunning = true;
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::move_left(float duration, std::vector<double> ctrl)
    {
      _prev_time = _current_time;
      _current_time = ros::Time::now();
      //  std::cout << "time : " << _current_time - _prev_time << std::endl;

      if (_isRunning == true) {
          //  _t = current_tine
          _t += 0.01;
          //ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
          unsigned int count = 0;
          hexapod_controller::HexapodControllerSimple _controller(ctrl, std::vector<int>());
          //  _controller.computeErrors(rpy.x,rpy.y);
          std::vector<double> pos = _controller.pos(_t);
          //  std::cout << "time : " << _t << std::endl;
          if (_t > duration) {
              _isRunning = false;

              _t = 0;
              for (unsigned int i = 0; i < NLegs; i++) {

                  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                  for (unsigned int j = 0; j < NLegs; j++) {
                      if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                          count++; //when the position 0 is reached do count++
                          //  std::cout << count << std::endl;
                      }
                  }
              }
          }
          for (unsigned int i = 0; i < NLegs-3; i++) {
              joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2]-0.1 - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
          for (unsigned int i = 3; i < NLegs; i++) {
              joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2]+0.1 - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
      }
      else {
          for (unsigned int i = 0; i < n_joints; i++) {
              joints[i]->setCommand(0);
          }
      }
      _isRunning = true;
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::turn_left(float duration, std::vector<double> ctrl)
    {
      _prev_time = _current_time;
      _current_time = ros::Time::now();
      //  std::cout << "time : " << _current_time - _prev_time << std::endl;

      if (_isRunning == true) {
          //  _t = current_tine
          _t += 0.01;
          //ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
          unsigned int count = 0;
          hexapod_controller::HexapodControllerSimple _controller(ctrl, std::vector<int>());
          //  _controller.computeErrors(rpy.x,rpy.y);
          std::vector<double> pos = _controller.pos(_t);
          //  std::cout << "time : " << _t << std::endl;
          if (_t > duration) {
              _isRunning = false;

              _t = 0;
              for (unsigned int i = 0; i < NLegs; i++) {

                  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                  for (unsigned int j = 0; j < NLegs; j++) {
                      if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                          count++; //when the position 0 is reached do count++
                          //  std::cout << count << std::endl;
                      }
                  }
              }
          }
          for (unsigned int i = 0; i < NLegs-3; i++) {
              joints[i]->setCommand(_kp * (-pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
          for (unsigned int i = 3; i < NLegs; i++) {
              joints[i]->setCommand(_kp * (pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
      }
      else {
          for (unsigned int i = 0; i < n_joints; i++) {
              joints[i]->setCommand(0);
          }
      }
      _isRunning = true;
    }


    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::turn_right(float duration, std::vector<double> ctrl)
    {
      _prev_time = _current_time;
      _current_time = ros::Time::now();
      //  std::cout << "time : " << _current_time - _prev_time << std::endl;

      if (_isRunning == true) {
          //  _t = current_tine
          _t += 0.01;
          //ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
          unsigned int count = 0;
          hexapod_controller::HexapodControllerSimple _controller(ctrl, std::vector<int>());
          //  _controller.computeErrors(rpy.x,rpy.y);
          std::vector<double> pos = _controller.pos(_t);
          //  std::cout << "time : " << _t << std::endl;
          if (_t > duration) {
              _isRunning = false;

              _t = 0;
              for (unsigned int i = 0; i < NLegs; i++) {

                  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
                  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
                  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
                  for (unsigned int j = 0; j < NLegs; j++) {
                      if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                          count++; //when the position 0 is reached do count++
                          //  std::cout << count << std::endl;
                      }
                  }
              }
          }
          for (unsigned int i = 0; i < NLegs-3; i++) {
              joints[i]->setCommand(_kp * (pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
          for (unsigned int i = 3; i < NLegs; i++) {
              joints[i]->setCommand(_kp * (-pos[3 * i] - joints[i]->getPosition()));
              joints[i + 6]->setCommand(_kp * (pos[3 * i + 1] - joints[i + 6]->getPosition()));
              joints[i + 12]->setCommand(_kp * (pos[3 * i + 2] - joints[i + 12]->getPosition()));
              //  joints[i]->setCommand(_kp * (0 - joints[i]->getPosition()));
              //  joints[i + 6]->setCommand(_kp * (0 - joints[i + 6]->getPosition()));
              //  joints[i + 12]->setCommand(_kp * (0 - joints[i + 12]->getPosition()));
              for (unsigned int j = 0; j < NLegs; j++) {
                  if ((std::abs(pos[3 * i] - joints[j]->getPosition()) < e) && (std::abs(pos[3 * i + 1] - joints[j + 6]->getPosition()) < e) && (std::abs(pos[3 * i + 2] - joints[j + 12]->getPosition()) < e)) {
                      count++; //when the position 0 is reached do count++
                      //  std::cout << count << std::endl;
                  }
              }
          }
      }
      else {
          for (unsigned int i = 0; i < n_joints; i++) {
              joints[i]->setCommand(0);
          }
      }
      _isRunning = true;
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::imuCB(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_buffer.writeFromNonRT({msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
    }

    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::joyCB(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_buffer.writeFromNonRT({msg->axes[0], msg->axes[1], msg->axes[2]});
    }


    template <class SafetyConstraint, int NLegs>
    void KBLControllerV<SafetyConstraint, NLegs>::tfCB(const tf2_msgs::TFMessageConstPtr& msg)
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
    std::array<int, NLegs> KBLControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(
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

    // /** Get joint angles from end-effector pose.
    //
    //       @param q_init joint angles used to initialise the inverse kinematics
    //       @param frame target poses
    //       @param q_out resulting joint angles as found by inverse kinematics
    //       @param stop_on_failure if true, the function will return as soon as
    //           the inverse kinematic computation fails for one leg (we walk
    //           through them in ascending leg number).
    //
    //       @return status of the computation of each leg; failed if < 0
    //   **/
    // template <class SafetyConstraint, int NLegs>
    // std::array<int, NLegs> KBLControllerV<SafetyConstraint, NLegs>::cartesian_to_joint(std::array<KDL::ChainIkSolverPos_NR, NLegs> kdl_ik_solvers,
    //     const std::array<KDL::JntArray, NLegs>& q_init,
    //     const std::array<KDL::Frame, NLegs>& frame,
    //     std::array<KDL::JntArray, NLegs>& q_out,
    //     bool stop_on_failure)
    // {
    //     std::array<int, NLegs> status;
    //
    //     for (size_t leg = 0; leg < NLegs; ++leg) {
    //         std::cout << "in\n";
    //         status[leg] = kdl_ik_solvers[leg].CartToJnt(q_init[leg], frame[leg], q_out[leg]);
    //         if (stop_on_failure && status[leg] < 0)
    //             break;
    //         std::cout << "out\n";
    //     }
    //
    //     return status;
    // }

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

} // namespace keep_body_level_velocity_controller

#endif
