#include <hexapod_driver/hexapod.hpp>
#include <hexapod_controller/hexapod_controller_simple.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <thread>

using namespace hexapod_ros;

Hexapod::Hexapod(ros::NodeHandle nh, std::string ns) : _nh(nh), _namespace(ns)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();
}

void Hexapod::init()
{
    // Private node handle
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("Odom", _odom_frame, std::string("/odom"));
    n_p.param("BaseLink", _base_link_frame, std::string("/base_link"));
    n_p.param("OdomEnable", _odom_enable, false);
    n_p.param("MoCapOdomEnable", _mocap_odom_enable, false);

    if (_odom_enable && _mocap_odom_enable) {
        ROS_WARN_STREAM("You have enabled both the motion capture and the visual odometry! Using visual odometry for measuring..");
        _mocap_odom_enable = false;
    }

    // Init trajectory clients
    _traj_clients.clear();
    _traj_msgs.clear();
    for (size_t i = 0; i < 6; i++) {
        std::string traj_topic = _namespace + "/leg_" + std::to_string(i) + "_controller/follow_joint_trajectory";
        _traj_clients.push_back(std::make_shared<trajectory_client>(traj_topic, true));
        // TO-DO: blocking duration in params?
        if (!_traj_clients[i]->waitForServer(ros::Duration(1.0)))
            ROS_ERROR_STREAM("leg_" << i << " actionlib server could not be found at: " << traj_topic);

        trajectory_msgs::JointTrajectory msg;
        msg.joint_names.clear();

        msg.joint_names.push_back("body_leg_" + std::to_string(i));
        msg.joint_names.push_back("leg_" + std::to_string(i) + "_1_2");
        msg.joint_names.push_back("leg_" + std::to_string(i) + "_2_3");

        _traj_msgs.push_back(msg);
    }
    ROS_INFO_STREAM("Trajectory actionlib controllers initialized!");

    // Init ROS related

    if (_odom_enable) {
        // create publisher to reset UKF filter (robot_localization)
        _reset_filter_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1000);
    }
}

void Hexapod::reset()
{
    //TO-DO: Raise hexapod softly
}

void Hexapod::move(std::vector<double> ctrl, double duration, bool reset)
{
    if (reset && (_odom_enable || _mocap_odom_enable)) {
        reset_odom();
    }
    // time step for trajectory
    double step = 0.03;

    // Initialize controller
    hexapod_controller::HexapodControllerSimple controller(ctrl, std::vector<int>());

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    // Calculate points of trajectory
    for (double t = 0.0; t <= duration; t += step) {
        std::vector<double> pos = controller.pos(t);

        for (size_t i = 0; i < 6; i++) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.clear();

            // Should update HexapodControllerSimple to output proper angles
            point.positions.push_back(-pos[i * 3]);
            point.positions.push_back(pos[i * 3 + 1]);
            point.positions.push_back(-pos[i * 3 + 2]);

            point.time_from_start = ros::Duration(t + 0.5);

            _traj_msgs[i].points.push_back(point);
        }
    }

    std::vector<std::thread> threads;
    // Send messages/goals
    for (size_t i = 0; i < 6; i++) {
        if (!_traj_clients[i]->isServerConnected()) {
            ROS_WARN_STREAM("leg_" << i << " actionlib server is not connected!");
            continue;
        }

        threads.push_back(std::thread(&Hexapod::_test, *this, i, duration));

        // _traj_msgs[i].header.stamp = ros::Time::now();
        //
        // control_msgs::FollowJointTrajectoryGoal goal;
        // goal.trajectory = _traj_msgs[i];
        // _traj_clients[i]->sendGoal(goal);
    }
    for (size_t i = 0; i < threads.size(); i++) {
        threads[i].join();
    }

    // for (size_t i = 0; i < 6; i++) {
    //     if (!_traj_clients[i]->isServerConnected()) {
    //         ROS_WARN_STREAM("leg_" << i << " actionlib server is not connected!");
    //         continue;
    //     }
    //     // TO-DO: blocking duration in params
    //     _traj_clients[i]->waitForResult(ros::Duration(duration));
    //     if (_traj_clients[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    //         ROS_WARN_STREAM("Trajectory execution for leg_" << std::to_string(i) << " failed with status '" << _traj_clients[i]->getState().toString() << "'");
    // }
}

void Hexapod::_test(size_t i, double duration)
{
    _traj_msgs[i].header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = _traj_msgs[i];
    _traj_clients[i]->sendGoal(goal);

    _traj_clients[i]->waitForResult(ros::Duration(duration + 0.5));
    if (_traj_clients[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN_STREAM("Trajectory execution for leg_" << std::to_string(i) << " failed with status '" << _traj_clients[i]->getState().toString() << "'");
}

void Hexapod::reset_odom()
{
    // reset visual odometry
    if (_odom_enable) {
        ros::ServiceClient client = _nh.serviceClient<std_srvs::Empty>("/reset_odom");
        std_srvs::Empty srv;
        if (client.call(srv)) {
            ROS_INFO_STREAM("reset_odom sent");
        }
        else {
            ROS_INFO_STREAM("Failed to reset odometry");
        }
        // reset UKF filter (robot_localization)
        // by publishing a PoseWithCovarianceStamped message
        geometry_msgs::PoseWithCovarianceStamped pose_with_cov_st;
        // set message's header
        pose_with_cov_st.header.stamp = ros::Time::now();
        pose_with_cov_st.header.frame_id = _odom_frame;
        // set position
        pose_with_cov_st.pose.pose.position.x = 0;
        pose_with_cov_st.pose.pose.position.y = 0;
        pose_with_cov_st.pose.pose.position.z = 0;
        // set orientation
        pose_with_cov_st.pose.pose.orientation.x = 0;
        pose_with_cov_st.pose.pose.orientation.y = 0;
        pose_with_cov_st.pose.pose.orientation.z = 0;
        pose_with_cov_st.pose.pose.orientation.w = 1;
        // publish message to reset UKF filter
        _reset_filter_pub.publish(pose_with_cov_st);
        ROS_INFO_STREAM("Message to reset UKF filter sent");
    }

    // Reset mocap system
    if (_mocap_odom_enable) {
        // TO-DO: Check if we need this python node
        ros::ServiceClient odom_client = _nh.serviceClient<std_srvs::Empty>("/odom_transform_restart");
        std_srvs::Empty empty_srv;
        if (odom_client.call(empty_srv)) {
            ROS_INFO_STREAM("odom_transform_restart sent");
        }
        else {
            ROS_INFO_STREAM("Failed to call odom_transform_restart");
        }
    }

    // Get odom transformation
    if (_odom_enable || _mocap_odom_enable) {
        ros::Duration(0.1).sleep();
        pos_update();
        _init_pos = _pos;
    }
}

void Hexapod::pos_update()
{
    tf::TransformListener listener;
    while (_nh.ok()) {
        try {
            listener.lookupTransform(_base_link_frame, _odom_frame, ros::Time(0), _pos);
        }
        catch (tf::TransformException ex) {
            ROS_WARN_STREAM("Failed to get transfromation from '" << _base_link_frame << "' to '" << _odom_frame << "': " << ex.what());
            ros::Duration(0.1).sleep();
        }

        break;
    }
}

tf::Vector3 Hexapod::position()
{
    return _pos.getOrigin() - _init_pos.getOrigin();
}

tf::Transform Hexapod::transform()
{
    return tf::Transform(_pos.getRotation() - _init_pos.getRotation(), _pos.getOrigin() - _init_pos.getOrigin());
}
