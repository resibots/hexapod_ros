#ifndef HEXAPOD_DRIVER_HEXAPOD_HPP
#define HEXAPOD_DRIVER_HEXAPOD_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace hexapod_ros {

    class Hexapod {
    public:
        using trajectory_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        Hexapod(ros::NodeHandle nh, std::string ns = "/dynamixel_controllers");
        ~Hexapod() {}

        void init();
        void reset();
        void move(std::vector<double> ctrl, double duration);

    protected:
        // ROS node handle
        ros::NodeHandle _nh;
        // Store the values of parameters for this ROS node
        std::string _odom_topic_name, _namespace;
        bool _odom_enable, _mocap_odom_enable;
        // Trajectory Action Lib Client
        std::vector<std::shared_ptr<trajectory_client>> _traj_clients;
        std::vector<trajectory_msgs::JointTrajectory> _traj_msgs;
    };
}

#endif
