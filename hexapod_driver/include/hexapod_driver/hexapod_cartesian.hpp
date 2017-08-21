#ifndef HEXAPOD_DRIVER_HEXAPOD_KINEMATIC_HPP
#define HEXAPOD_DRIVER_HEXAPOD_KINEMATIC_HPP

#include <ros/ros.h>
#include <hexapod_driver/hexapod.hpp>
#include <hexapod_ik/multipod_ik.hpp>

namespace hexapod_ros {

    class HexapodCartesian : public Hexapod {
    public:
        HexapodCartesian(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers");
        // ~HexapodCartesian();

        virtual void init();

        /** Make the hexapod move for some time, based on control parameters.

            We generate the joint trajectories with the controller
            https://github.com/resibots/hexapod_common/tree/master/hexapod_controller

            @param ctrl vector of parameters for the controller (see
                hexapod_common/hexapod_controller)
            @param duration how much time the robot will move (s)
            @param reset call reset_odom() at the beginning of the method
            @param step time step for trajectory
        **/
        virtual void move(std::vector<double> ctrl, double duration, bool reset = true, double step = 0.02);

    protected:
        std::shared_ptr<multipod_ik::MultipodInverseKinematics<6>> _hexapod_ik;
    };
} // namespace hexapod_ros

#endif
