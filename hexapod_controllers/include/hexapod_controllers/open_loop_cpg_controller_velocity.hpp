
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

#ifndef OPEN_LOOP_CPG_CONTROLLER_VELOCITY_H
#define OPEN_LOOP_CPG_CONTROLLER_VELOCITY_H

#include "cpg.hpp"
#include <chrono>
#include <cmath>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>
namespace open_loop_cpg_controller_velocity {

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
    class OlCpgControllerV : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        OlCpgControllerV();
        ~OlCpgControllerV(){};

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

        /*!  Name of the joints defined in urdf and .yaml config file. Make the correspondance with ros_control JointHandles  */
        std::vector<std::string> joint_names;
        /*!  JointHandle objects recovered thanks to joint_names*/
        std::vector<std::shared_ptr<hardware_interface::JointHandle>> joints;
        /*!  number of joints */
        unsigned int n_joints;
        /*!  has_init will be set to true when all joint positions are at 0 +- e*/
        bool has_init;
        float e;
        cpg::CPG cpg_;

        /*!    angles of the soulder joints in the axial plane at init*/
        std::vector<float> X, Xprev, Xcommand;
        /*!    angles of the soulder joints in the sagittal plane at init*/
        std::vector<float> Y, Yprev, Ycommand;

    private:
        SafetyConstraint _constraint;

        /**
         * \brief Put the servos at the position 0 at init
         */
        void initJointPosition();

    }; // class OlCpgControllerV

    /**
     * \brief Constructor
     */
    template <class SafetyConstraint, int NLegs>
    OlCpgControllerV<SafetyConstraint, NLegs>::OlCpgControllerV()
    {
        e = 0.05;
        has_init = false;
        X.resize(NLegs, 0.0);
        Xprev.resize(NLegs, 0.0);
        Xcommand.resize(NLegs, 0.0);
        Y.resize(NLegs, 0.0);
        Yprev.resize(NLegs, 0.0);
        Ycommand.resize(NLegs, 0.0);
    }

    /**
     * \brief Tries to recover n_joints JointHandles
     */
    template <class SafetyConstraint, int NLegs>
    bool OlCpgControllerV<SafetyConstraint, NLegs>::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
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

        return true;
    }

    /**
     * \brief This function is the control loop itself, called periodically by the controller_manager
     */
    template <class SafetyConstraint, int NLegs>
    void OlCpgControllerV<SafetyConstraint, NLegs>::update(const ros::Time& /*time*/, const ros::Duration& period)
    {

        //Set the servo positions to 0 at the beginning because the init and starting function are not called long enough to do that
        if (has_init == false) {
            initJointPosition();
            X[1] = -joints[5]->getPosition();
            Y[1] = joints[11]->getPosition();
            std::cout << Y[1] << std::endl;
        }
        //Then carry on the cpg control
        else {

            X[0] = joints[0]->getPosition();
            Y[0] = joints[6]->getPosition();

            X[4] = joints[2]->getPosition();
            Y[4] = joints[8]->getPosition();

            X[3] = -joints[4]->getPosition();
            Y[3] = joints[10]->getPosition();

            X[1] = -joints[5]->getPosition();
            Y[1] = joints[11]->getPosition();

            X[2] = joints[1]->getPosition();
            Y[2] = joints[7]->getPosition();

            X[5] = -joints[3]->getPosition();
            Y[5] = joints[9]->getPosition();

            /*compute X,Y derivatives*/
            std::vector<std::pair<float, float>> XYdot = cpg_.computeXYdot(X, Y);

            std::cout << "Pates" << 0 << " X: " << X[0] << " Y: " << Y[0] << std::endl;
            std::cout << "Xdot : " << XYdot[0].first << " Ydot : " << XYdot[0].second << std::endl;
            std::cout << "Pates" << 1 << " X: " << X[2] << " Y: " << Y[2] << std::endl;
            std::cout << "Xdot : " << XYdot[2].first << " Ydot : " << XYdot[2].second << std::endl;
            std::cout << "Pates" << 2 << " X: " << X[4] << " Y: " << Y[4] << std::endl;
            std::cout << "Xdot : " << XYdot[4].first << " Ydot : " << XYdot[4].second << std::endl;
            std::cout << "Pates" << 3 << " X: " << X[5] << " Y: " << Y[5] << std::endl;
            std::cout << "Xdot : " << XYdot[5].first << " Ydot : " << XYdot[5].second << std::endl;
            std::cout << "Pates" << 4 << " X: " << X[3] << " Y: " << Y[3] << std::endl;
            std::cout << "Xdot : " << XYdot[3].first << " Ydot : " << XYdot[3].second << std::endl;
            std::cout << "Pates" << 5 << " X: " << X[1] << " Y: " << Y[1] << std::endl;
            std::cout << "Xdot : " << XYdot[1].first << " Ydot : " << XYdot[1].second << std::endl;

            std::cout << "\n";

            joints[0]->setCommand(0.02 * XYdot[0].first);
            joints[6]->setCommand(0.02 * XYdot[0].second);
            joints[12]->setCommand(0.02 * XYdot[0].second);

            joints[3]->setCommand(0.02 * XYdot[5].first);
            joints[9]->setCommand(-0.02 * XYdot[5].second);
            joints[15]->setCommand(-0.02 * XYdot[5].second);

            joints[4]->setCommand(-0.02 * XYdot[3].first);
            joints[10]->setCommand(0.02 * XYdot[3].second);
            joints[16]->setCommand(0.02 * XYdot[3].second);

            joints[1]->setCommand(-0.02 * XYdot[2].first);
            joints[7]->setCommand(-0.02 * XYdot[2].second);
            joints[13]->setCommand(-0.02 * XYdot[2].second);

            joints[2]->setCommand(0.02 * XYdot[4].first);
            joints[8]->setCommand(0.02 * XYdot[4].second);
            joints[14]->setCommand(0.02 * XYdot[4].second);

            joints[5]->setCommand(0.02 * XYdot[1].first);
            joints[11]->setCommand(-0.02 * XYdot[1].second);
            joints[17]->setCommand(-0.02 * XYdot[1].second);
        }
        _constraint.enforce(period);
    }
    /**
    * \brief Put the servos at the position 0 at init
    */
    template <class SafetyConstraint, int NLegs>
    void OlCpgControllerV<SafetyConstraint, NLegs>::initJointPosition()
    {

        unsigned int count = 0;
        for (unsigned int i = 0; i < n_joints; i++) {
            joints[i]->setCommand(-0.3 * (joints[i]->getPosition() - 0.1));
            if (std::abs(joints[i]->getPosition() - 0.1) < e) {
                count++; //when the position 0 is reached do count++
            }
            if (count == n_joints) {
                has_init = true; //When every servo is around the 0 position stop the init phase and go on with cpg control
            }
        }
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

} // namespace open_loop_cpg_controller_velocity

#endif
