#ifndef HEXAPOD_DRIVER_HEXAPOD_HPP
#define HEXAPOD_DRIVER_HEXAPOD_HPP

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
        void move_legs(std::vector<double> ctrl, double t_init, double duration, bool reset = false);
        void move_positions(std::vector<double> pos, double duration);
        void move_leg(int leg, std::vector<double> joint_angles);
        void reset_odom();

        void add_removed(int i)
        {
            if (i < 6 && i >= 0)
                _removed.push_back(i);
        }

        void clear_removed()
        {
            _removed.clear();
        }

        void add_offseted(int i, double offset)
        {
            if (i < 6 && i >= 0)
                _offseted[i] = offset;
        }

        void clear_offseted()
        {
            _offseted.clear();
        }

        tf::Transform transform();
        geometry_msgs::Twist twist();
        tf::Transform pos() { return _pos; }

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
        // TF listener
        tf::TransformListener _listener;
        // removed legs (in software)
        std::vector<int> _removed;
        // offset legs (in software)
        std::map<int, double> _offseted;
    };
}

#endif
