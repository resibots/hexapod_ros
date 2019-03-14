#ifndef HEXAPOD_DRIVER_HEXAPOD_CPG_HPP
#define HEXAPOD_DRIVER_HEXAPOD_CPG_HPP

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hexapod_controller/cpg_open_loop.hpp>
#include <ros/ros.h>
#include <tf/tf.h>

namespace hexapod_ros {

    class HexapodCPG {
    public:
        using trajectory_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        HexapodCPG(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers");
        ~HexapodCPG();

        void init();

        /** Put the robot to rest.
            This method will have the robot gently rest its body on the floor.
        **/
        void relax();

        void reset();

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
        // Trajectory Action Lib Client
        std::vector<std::shared_ptr<trajectory_client>> _traj_clients;
        std::vector<trajectory_msgs::JointTrajectory> _traj_msgs;
        // ROS Publisher for robot_localization Filter
        ros::Publisher _reset_filter_pub;
        // TF position
        tf::StampedTransform _pos, _init_pos;

        hexapod_controller::CpgOpenLoop controller;
    };
} // namespace hexapod_ros

#endif
