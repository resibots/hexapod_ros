#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hexapod_controller/hexapod_controller_simple.hpp>
#include <hexapod_driver/hexapod_imu.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <thread>

#include <iostream>
using namespace hexapod_ros;

Hexapod::Hexapod(ros::NodeHandle nh, std::string ns) : _nh(nh), _namespace(ns)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();
}

Hexapod::~Hexapod()
{
    ROS_INFO_STREAM("Relaxing...");
    relax();
}

void Hexapod::init()
{
    // Private node handle
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("odom", _odom_frame, std::string("/odom"));
    n_p.param("base_link", _base_link_frame, std::string("/base_link"));
    n_p.param("odom_enable", _odom_enable, false);
    n_p.param("mocap_odom_enable", _mocap_odom_enable, true);

    if (_odom_enable && _mocap_odom_enable) {
        ROS_WARN_STREAM("You have enabled both the motion capture and the visual "
                        "odometry! Using visual odometry for measuring..");
        _mocap_odom_enable = false;
    }

    // Init trajectory clients
    _traj_clients.clear();
    _traj_msgs.clear();
    for (size_t i = 0; i < 6; i++) {
        std::string traj_topic = _namespace + "/leg_" + std::to_string(i) + "_controller/follow_joint_trajectory";
        _traj_clients.push_back(
            std::make_shared<trajectory_client>(traj_topic, true));
        // TO-DO: blocking duration in params?
        if (!_traj_clients[i]->waitForServer(ros::Duration(1.0)))
            ROS_ERROR_STREAM("leg_" << i
                                    << " actionlib server could not be found at: "
                                    << traj_topic);

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
        _reset_filter_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/set_pose", 1000);
    }
    //_mode_pub = _nh.advertise<std_msg::Float64>("/mode", 1000);
    //_time_pub = _nh.advertise<std_msg::Float64>("/time", 1000);
    //_ctrl_pub = _nh.advertise<std_msg::Float64MultiArray>("/ctrl", 1000);
    param_name = "mode";
    if (!nh.getParam(param_name, _mode)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        return false;
    }
    param_name = "duration";
    if (!nh.getParam(param_name, _duration)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        return false;
    }
    param_name = "isRunning";
    if (!nh.getParam(param_name, _isRunning)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        return false;
    }

    param_name = "ctrl";
    if (!nh.getParam(param_name, _ctrl)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        return false;
    }

    param_name = "isRunning";
    if (!nh.getParam(param_name, _isRunning)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
        return false;
    }
    _mode_running = nh.subscribe<std_msg::Bool>("/simu_running", 1, &HexapodIMU::mode_running_CB, this);
    // Reset
    ROS_INFO_STREAM("Reset...");

    reset();

    // Get odom transformation
    if (_odom_enable || _mocap_odom_enable) {
        reset_odom();
    }
}
void Hexapod::wait()
{
    while (_isRunning == 1) {
        nh.getParam("isRunning", _isRunning);
    }
}
void Hexapod::relax()
{
    wait();
    _mode = relax;
    _nh.setParam("mode", _mode);
    _duration = 2.0;
    _nh.setParam("duration", _duration);

    double duration = 2.0;
    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    for (size_t i = 0; i < 6; i++) {
        for (double t = 0; t < duration; t += 0.1) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.clear();

            point.positions.push_back(0.0);
            double a = (duration - t) / duration;
            double b = t / duration;
            point.positions.push_back(a * M_PI_4 / 6.0 + b * (M_PI_2 * 1.2));
            point.positions.push_back(0.0);

            point.time_from_start = ros::Duration(t);

            _traj_msgs[i].points.push_back(point);
        }
    }

    _send_trajectories(duration);

    ros::Duration(0.5).sleep();
}

void Hexapod::reset()
{
    wait();
    _mode = reset;
    _nh.setParam("mode", _mode);
    _duration = 0.1;
    _nh.setParam("duration", _duration);

    double duration = 0.1;
    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    for (size_t i = 0; i < 6; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.clear();

        point.positions.push_back(0.0);
        point.positions.push_back(M_PI_2);
        point.positions.push_back(256 * M_PI / 2048.0);

        point.time_from_start = ros::Duration(duration);

        _traj_msgs[i].points.push_back(point);
    }

    _send_trajectories(duration);

    ros::Duration(0.5).sleep();

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    for (size_t i = 0; i < 6; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.clear();

        point.positions.push_back(0.0);
        point.positions.push_back(0.0);
        point.positions.push_back(256 * M_PI / 2048.0);

        point.time_from_start = ros::Duration(duration);

        _traj_msgs[i].points.push_back(point);
    }

    _send_trajectories(duration);

    ros::Duration(0.5).sleep();

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    for (size_t i = 0; i < 6; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.clear();

        point.positions.push_back(0.0);
        point.positions.push_back(0.0);
        point.positions.push_back(0.0);

        point.time_from_start = ros::Duration(duration);

        _traj_msgs[i].points.push_back(point);
    }

    _send_trajectories(duration);
}

void Hexapod::zero()
{
    wait();
    _mode = zero;
    _nh.setParam("mode", _mode);
    _duration = 0.1;
    _nh.setParam("duration", _duration);

    double duration = 0.1;
    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    for (size_t i = 0; i < 6; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.clear();

        point.positions.push_back(0.0);
        point.positions.push_back(0.0);
        point.positions.push_back(0.0);

        point.time_from_start = ros::Duration(duration);

        _traj_msgs[i].points.push_back(point);
    }

    _send_trajectories(duration);
}

void Hexapod::move(std::vector<double> ctrl, double duration, bool reset)
{
    wait();
    _mode = move;
    _nh.setParam("mode", _mode);
    _duration = duration;
    _nh.setParam("duration", _duration);
    _ctrl = ctrl;
    _nh.setParam("controle", _ctrl);

    while (_mode_running == true) {};
    // Start from zero
    zero();

    if (reset && (_odom_enable || _mocap_odom_enable)) {
        reset_odom();
    }
    // time step for trajectory
    double step = 0.001;

    // Initialize controller
    hexapod_controller::HexapodControllerSimple controller(ctrl, std::vector<int>());

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    // Calculate points of trajectory
    for (double f = 0; f <= 5; f += 0.02) {

        for (double t = 0.0; t <= 0.02; t += step) {
            std::vector<double> pos = controller.pos(t + f);

            for (size_t i = 0; i < 6; i++) {
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions.clear();

                // Should update HexapodControllerSimple to output proper angles
                point.positions.push_back(-pos[i * 3]);
                point.positions.push_back(pos[i * 3 + 1]);
                point.positions.push_back(-pos[i * 3 + 2]);

                point.time_from_start = ros::Duration(t);

                _traj_msgs[i].points.push_back(point);
            }
        }
        _send_trajectories(0.02);
        for (size_t i = 0; i < 6; i++) {
            _traj_msgs[i].points.clear();
        }
    }
    //  _send_trajectories(duration);

    // Go back to zero
    zero();
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
            ROS_ERROR_STREAM("Failed to reset odometry");
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
            ROS_ERROR_STREAM("Failed to call odom_transform_restart");
        }
    }

    // Get odom transformation
    if (_odom_enable || _mocap_odom_enable) {
        ros::Duration(0.01).sleep();
        _pos_update();
        _init_pos = _pos;
    }
}

tf::Transform Hexapod::transform()
{
    if (!_odom_enable && !_mocap_odom_enable)
        ROS_ERROR_STREAM("Mocap and odom are disabled so Hexapod::transform should "
                         "not be called");

    _pos_update();
    return _init_pos.inverse() * _pos;
}

void Hexapod::_pos_update()
{
    tf::TransformListener listener;
    ros::Time start_t = ros::Time::now();
    while (_nh.ok()) {
        try {
            listener.lookupTransform(_odom_frame, _base_link_frame, ros::Time(0),
                _pos);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_DEBUG_STREAM("Failed to get transfromation from '"
                << _base_link_frame << "' to '" << _odom_frame
                << "': " << ex.what());
        }
        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(1.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get transfromation from '"
                << _base_link_frame << "' to '" << _odom_frame);
            break;
        }
    }
}

void Hexapod::_send_trajectories(double duration)
{
    std::vector<std::thread> threads;

    // Send messages/goals
    for (size_t i = 0; i < 6; i++) {
        if (!_traj_clients[i]->isServerConnected()) {
            ROS_WARN_STREAM("leg_" << i << " actionlib server is not connected!");
            continue;
        }

        if (_traj_msgs[i].points.size() == 0) {
            ROS_WARN_STREAM("Msg size for leg_" << i << " is zero!");
            continue;
        }

        threads.push_back(
            std::thread(&Hexapod::_send_trajectory, this, i, duration));
    }

    for (size_t i = 0; i < threads.size(); i++) {
        threads[i].join();
    }
}

void Hexapod::_send_trajectory(size_t i, double duration)
{
    _traj_msgs[i].header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = _traj_msgs[i];
    _traj_clients[i]->sendGoal(goal);
    _traj_clients[i]->waitForResult(ros::Duration(duration + 0.005));
    //  _traj_clients[i]->waitForResult(ros::Duration(duration + 0.5));
    if (_traj_clients[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN_STREAM("Trajectory execution for leg_"
            << std::to_string(i) << " failed with status '"
            << _traj_clients[i]->getState().toString() << "'");
}
