
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

#ifndef OPEN_LOOP_CPG_CONTROLLER_H
#define OPEN_LOOP_CPG_CONTROLLER_H

#include "cpg.hpp"
#include <chrono>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>

namespace open_loop_cpg_controller {

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
    class OlCpgController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
    public:
        OlCpgController();
        ~OlCpgController(){};

        /**
         * \brief Tries to recover n_joints JointHandles
         */
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh);
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

        std::chrono::steady_clock::time_point current_time;

        std::chrono::steady_clock::time_point last_time;

        bool integration_has_diverged;

    private:
        SafetyConstraint _constraint;

        /**
         * \brief Put the servos at the position 0 at init
         */
        void initJointPosition();

    }; // class OlCpgController

    /**
     * \brief Constructor
     */
    template <class SafetyConstraint, int NLegs>
    OlCpgController<SafetyConstraint, NLegs>::OlCpgController()
    {
        e = 0.001;
        has_init = false;
        X.resize(NLegs, 0.0);
        Xprev.resize(NLegs, 0.0);
        Xcommand.resize(NLegs, 0.0);
        Y.resize(NLegs, 0.0);
        Yprev.resize(NLegs, 0.0);
        Ycommand.resize(NLegs, 0.0);
        integration_has_diverged = false;
    }

    /**
     * \brief Tries to recover n_joints JointHandles
     */
    template <class SafetyConstraint, int NLegs>
    bool OlCpgController<SafetyConstraint, NLegs>::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
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
    void OlCpgController<SafetyConstraint, NLegs>::update(const ros::Time& /*time*/, const ros::Duration& period)
    {
        current_time = std::chrono::steady_clock::now();
        //cpg_.set_rk_dt(period.toSec());

        // std::cout << period.toSec() << std::endl;
        // std::cout << period << std::endl;

        //Set the servo positions to 0 at the beginning because the init and starting function are not called long enough to do that
        if (has_init == false) {
            initJointPosition();
            Xcommand = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
            Ycommand = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        }
        //Then carry on the cpg control
        else {
            //
            // for (unsigned int i = 0; i < n_joints; i++) {
            //     std::cout << i << " jointpos " << joints[i]->getPosition() << std::endl;
            // }

            X[0] = joints[0]->getPosition();
            Y[0] = joints[12]->getPosition();

            X[1] = joints[5]->getPosition();
            Y[1] = -joints[17]->getPosition();

            X[2] = -joints[1]->getPosition();
            Y[2] = -joints[13]->getPosition();

            X[3] = -joints[4]->getPosition();
            Y[3] = joints[16]->getPosition();

            X[4] = joints[2]->getPosition();
            Y[4] = joints[14]->getPosition();

            X[5] = joints[3]->getPosition();
            Y[5] = -joints[15]->getPosition();

            std::chrono::duration<double> time_span
                = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - last_time);
            std::cout << time_span.count() << std::endl;

            /*compute X,Y derivatives*/
            std::vector<std::pair<float, float>> XYdot = cpg_.computeXYdot(Xcommand, Ycommand);

            for (int i = 0; i < XYdot.size(); i++) {
                /*Integrate XYdot*/
                std::pair<float, float> xy = cpg_.RK4(Xcommand[i], Ycommand[i], XYdot[i]);
                Xcommand[i] = xy.first;
                Ycommand[i] = xy.second;
                // X[i] = Xcommand[i];
                // Y[i] = Ycommand[i];

                /*Check if integration hasn't diverged*/
                if (std::isnan(xy.first) || std::isnan(xy.second)) {
                    std::cout << "INTEGRATION HAS DIVERGED : reboot the node and use a bigger loop rate"
                              << std::endl;
                    integration_has_diverged = true;
                }
            }

            for (unsigned int i = 0; i < NLegs; i++) {
                std::cout << "X" << i << "  Sensor value : " << X[i] << " Command value : " << Xcommand[i] << " error : " << Xcommand[i] - X[i] << std::endl;
                std::cout << "Y" << i << "  Sensor value : " << Y[i] << " Command value : " << Ycommand[i] << " error : " << Ycommand[i] - Y[i] << std::endl;
            }

            if (integration_has_diverged == false) {

                joints[0]->setCommand(Xcommand[0]);
                joints[6]->setCommand(Ycommand[0]);
                joints[12]->setCommand(Ycommand[0]);

                joints[1]->setCommand(-Xcommand[2]);
                joints[7]->setCommand(-Ycommand[2]);
                joints[13]->setCommand(-Ycommand[2]);

                joints[2]->setCommand(Xcommand[4]);
                joints[8]->setCommand(Ycommand[4]);
                joints[14]->setCommand(Ycommand[4]);

                joints[3]->setCommand(Xcommand[5]);
                joints[9]->setCommand(-Ycommand[5]);
                joints[15]->setCommand(-Ycommand[5]);

                joints[4]->setCommand(-Xcommand[3]);
                joints[10]->setCommand(Ycommand[3]);
                joints[16]->setCommand(Ycommand[3]);

                joints[5]->setCommand(Xcommand[1]);
                joints[11]->setCommand(-Ycommand[1]);
                joints[17]->setCommand(-Ycommand[1]);
            }
        }
        _constraint.enforce(period);
        last_time = std::chrono::steady_clock::now();
    }
    /**
    * \brief Put the servos at the position 0 at init
    */
    template <class SafetyConstraint, int NLegs>
    void OlCpgController<SafetyConstraint, NLegs>::initJointPosition()
    {

        unsigned int count = 0;
        for (unsigned int i = 0; i < n_joints; i++) {
            joints[i]->setCommand(0); //send to the hardware inteface the position command 0
            if (abs(joints[i]->getPosition()) < e) {
                count++; //when the position 0 is reached do count++
            }
            if (count == n_joints) {
                has_init = true; //When every servo is around the 0 position stop the init phase and go on with cpg control
            }
        }
    }

    // bool OlCpgController<SafetyConstraint, NLegs>::goToJointPosition(float value)
    // {
    //
    //     unsigned int count = 0;
    //     for (unsigned int i = 0; i < n_joints; i++) {
    //         joints[i]->setCommand(0); //send to the hardware inteface the position command 0
    //         if (abs(joints[i]->getPosition()) < e) {
    //             count++; //when the position 0 is reached do count++
    //         }
    //         if (count == n_joints) {
    //             has_init = true; //When every servo is around the 0 position stop the init phase and go on with cpg control
    //         }
    //     }
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

} // namespace open_loop_cpg_controller

#endif
