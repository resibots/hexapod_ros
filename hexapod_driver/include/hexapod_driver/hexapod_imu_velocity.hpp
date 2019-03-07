#ifndef HEXAPOD_DRIVER_HEXAPOD_IMU_VELOCITY_HPP
#define HEXAPOD_DRIVER_HEXAPOD_IMU_VELOCITY_HPP

#include <ros/ros.h>
#include <tf/tf.h>

namespace hexapod_ros {

    class HexapodIMUVel {
    public:
        HexapodIMUVel(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers");
        ~HexapodIMUVel();

        bool init();

        /** Put the robot to rest.
            This method will have the robot gently rest its body on the floor.
        **/
        void relax();

        void reset();
        void wait();

        /** Put the hexapod in "neutral" pose.
            Meaning that all joints are at the middle of their range and the
            robot is standing straight.
        **/
        void zero();

        /** Make the hexapod move for some time, based on control parameters.

            We generate the joint trajectories with the controller
            https://github.com/resibots/hexapod_common/tree/master/hexapod_controller

            @param ctrl vector of parameters for the controller (see
                hexapod_common/hexapod_controller)
            @param duration how much time the robot will move (s)
            @param reset call reset_odom() at the beginning of the method
        **/
        virtual void move(std::vector<double> ctrl, double duration, bool reset = true);

        void reset_odom();
        tf::Transform transform();

    protected:
        void _pos_update();
        void _send_trajectories(double duration);
        void _send_trajectory(size_t i, double duration);

        // ROS node handle
        ros::NodeHandle _nh;
        // Store the values of parameters for this ROS node
        std::string _odom_frame, _base_link_frame, _namespace;
        bool _odom_enable, _mocap_odom_enable;

        // ROS Publisher for robot_localization Filter
        ros::Publisher _reset_filter_pub;

        // TF position
        tf::StampedTransform _pos, _init_pos;
        float _duration;
        std::string param_name;
        bool _isRunning;
        std::vector<double> _ctrl;
        int _mode;
    };
} // namespace hexapod_ros

#endif
