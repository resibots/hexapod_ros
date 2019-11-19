#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hexapod_controller/hexapod_controller_simple.hpp>
#include <hexapod_driver/hexapod_BD.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <thread>

using namespace hexapod_ros;

HexapodBlackDrops::HexapodBlackDrops(ros::NodeHandle nh, std::string ns) : _nh(nh), _namespace(ns)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();

    // ros::console::shutdown();
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

HexapodBlackDrops::~HexapodBlackDrops()
{
    ROS_INFO_STREAM("Relaxing...");
    relax();
}

void HexapodBlackDrops::init()
{
    // Private node handle
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("odom", _odom_frame, std::string("/odom"));
    n_p.param("base_link", _base_link_frame, std::string("/base_link"));
    n_p.param("odom_enable", _odom_enable, false);
    n_p.param("mocap_odom_enable", _mocap_odom_enable, true);

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

    // Reset
    ROS_INFO_STREAM("Reset...");
    reset();

    // Get odom transformation
    if (_odom_enable || _mocap_odom_enable) {
        reset_odom();
    }
}

void HexapodBlackDrops::relax()
{
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
            point.positions.push_back(a * M_PI_4 / 6.0 + b * (M_PI_2 * 1.3));
            point.positions.push_back(0.0);

            point.time_from_start = ros::Duration(t);

            _traj_msgs[i].points.push_back(point);
        }
    }

    _send_trajectories(duration);

    ros::Duration(0.5).sleep();
}

void HexapodBlackDrops::reset()
{
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

void HexapodBlackDrops::zero()
{
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

void HexapodBlackDrops::move(std::vector<double> ctrl, double duration, bool reset)
{
    // Start from zero
    zero();

    for (auto i : _removed) {
        move_leg(i, {0.0, M_PI * 1.1 / 2.0, M_PI * 1.1 / 2.0});
    }

    if (reset && (_odom_enable || _mocap_odom_enable)) {
        reset_odom();
    }
    // time step for trajectory
    double step = 0.1;

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
            std::vector<int>::iterator it = find(_removed.begin(), _removed.end(), (int)i);
            if (it != _removed.end())
                continue;
            double angle = pos[i * 3 + 1];
            std::map<int, double>::iterator it2 = _offseted.find(i);
            if (it2 != _offseted.end())
                angle = it2->second;
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.clear();

            // Should update HexapodControllerSimple to output proper angles
            point.positions.push_back(-pos[i * 3]);
            point.positions.push_back(angle);
            point.positions.push_back(-pos[i * 3 + 2]);

            point.time_from_start = ros::Duration(t);

            _traj_msgs[i].points.push_back(point);
        }
    }

    _send_trajectories(duration);

    // Go back to zero
    zero();
}

void HexapodBlackDrops::move_legs(std::vector<double> ctrl, double t_init, double duration, bool reset)
{
    // time step for trajectory
    double step = 0.03;

    // Initialize controller
    hexapod_controller::HexapodControllerSimple controller(ctrl, std::vector<int>());

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    // Calculate points of trajectory
    for (double t = t_init; t <= t_init + duration; t += step) {
        std::vector<double> pos = controller.pos(t);

        for (size_t i = 0; i < 6; i++) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.clear();
            // Should update HexapodControllerSimple to output proper angles
            point.positions.push_back(-pos[i * 3]);
            point.positions.push_back(pos[i * 3 + 1]);
            point.positions.push_back(-pos[i * 3 + 2]);

            point.time_from_start = ros::Duration(t - t_init);

            _traj_msgs[i].points.push_back(point);
        }
    }

    _send_trajectories(duration);
}

void HexapodBlackDrops::move_positions(std::vector<double> pos, double duration)
{
    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }
    for (size_t i = 0; i < 6; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.clear();
        // Should update HexapodControllerSimple to output proper angles
        point.positions.push_back(-pos[i * 3]);
        point.positions.push_back(pos[i * 3 + 1]);
        point.positions.push_back(-pos[i * 3 + 2]);

        point.time_from_start = ros::Duration(duration);

        _traj_msgs[i].points.push_back(point);
    }

    _send_trajectories(duration);
}

void HexapodBlackDrops::move_leg(int leg, std::vector<double> joint_angles)
{
    assert(leg < 6);
    double duration = 1.0;
    // Clear message points
    _traj_msgs[leg].points.clear();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.clear();

    point.positions.push_back(joint_angles[0]);
    point.positions.push_back(joint_angles[1]);
    point.positions.push_back(joint_angles[2]);

    point.time_from_start = ros::Duration(duration);

    _traj_msgs[leg].points.push_back(point);

    _send_trajectories(duration);

    ros::Duration(0.5).sleep();
}

void HexapodBlackDrops::reset_odom()
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

tf::Transform HexapodBlackDrops::transform()
{
    _pos_update();
    return _init_pos.inverse() * _pos;
}

geometry_msgs::Twist HexapodBlackDrops::twist()
{
    geometry_msgs::Twist _twist;
    // tf::TransformListener listener;
    ros::Time start_t = ros::Time::now();
    while (_nh.ok()) {
        try {
            _listener.lookupTwist(_odom_frame, _base_link_frame, ros::Time(0), ros::Duration(0.00001), _twist);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_DEBUG_STREAM("Failed to get twist from '" << _base_link_frame << "' to '" << _odom_frame << "': " << ex.what());
        }
        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(1.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get twist from '" << _base_link_frame << "' to '" << _odom_frame);
            break;
        }
    }
}

void HexapodBlackDrops::_pos_update()
{
    ros::Time start_t = ros::Time::now();
    while (_nh.ok()) {
        try {
            _listener.lookupTransform(_odom_frame, _base_link_frame, ros::Time(0), _pos);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR_STREAM("Failed to get transformation from '" << _base_link_frame << "' to '" << _odom_frame << "': " << ex.what());
        }
        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(1.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get transformation from '" << _base_link_frame << "' to '" << _odom_frame);
            break;
        }
    }
}

void HexapodBlackDrops::_send_trajectories(double duration)
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

        threads.push_back(std::thread(&HexapodBlackDrops::_send_trajectory, this, i, duration));
        // _send_trajectory(i, duration);
    }

    // for (size_t i = 0; i < 6; i++) {
    // size_t i = 0;
    // _traj_clients[i]->waitForResult(ros::Duration(duration));
    // if (_traj_clients[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_WARN_STREAM("Trajectory execution for leg_" << std::to_string(i) << " failed with status '" << _traj_clients[i]->getState().toString() << "'");

    for (size_t i = 0; i < threads.size(); i++) {
        threads[i].join();
    }
}

void HexapodBlackDrops::_send_trajectory(size_t i, double duration)
{
    _traj_msgs[i].header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = _traj_msgs[i];
    _traj_clients[i]->sendGoal(goal);

    _traj_clients[i]->waitForResult(ros::Duration(duration));
    if (_traj_clients[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN_STREAM("Trajectory execution for leg_" << std::to_string(i) << " failed with status '" << _traj_clients[i]->getState().toString() << "'");
}
