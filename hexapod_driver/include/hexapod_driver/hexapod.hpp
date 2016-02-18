#ifndef HEXAPOD_DRIVER_HEXAPOD_HPP
#define HEXAPOD_DRIVER_HEXAPOD_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/tf.h>

namespace hexapod_ros {

    class Hexapod {
    public:
        using trajectory_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        Hexapod(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers");
        ~Hexapod();

        void init();
        void relax();
        void reset();
        void zero();
        void move(std::vector<double> ctrl, double duration, bool reset = true);
        void reset_odom();
        tf::Vector3 position();
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
    };
}

#endif
