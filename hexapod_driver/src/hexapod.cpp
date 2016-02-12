#include <hexapod_driver/hexapod.hpp>
#include <hexapod_controller/hexapod_controller_simple.hpp>

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
    n_p.param("Odom", _odom_topic_name, std::string("/odom"));
    n_p.param("OdomEnable", _odom_enable, true);
    n_p.param("MoCapOdomTransformEnable", _mocap_odom_enable, false);

    if (!_odom_enable)
        _mocap_odom_enable = false;

    // Init trajectory clients
    _traj_clients.clear();
    _traj_msgs.clear();
    for (size_t i = 0; i < 6; i++) {
        std::string traj_topic = _namespace + "/leg_" + std::to_string(i) + "_controller/follow_joint_trajectory";
        _traj_clients.push_back(std::make_shared<trajectory_client>(traj_topic, true));
        // TO-DO: blocking duration in params
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
}

void Hexapod::reset()
{
    //TO-DO: Raise hexapod softly
}

void Hexapod::move(std::vector<double> ctrl, double duration)
{
    // time step for trajectory
    double step = 0.0005;

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

            point.time_from_start = ros::Duration(t);

            _traj_msgs[i].points.push_back(point);
        }
    }

    // Send messages/goals
    for (size_t i = 0; i < 6; i++) {
        if (!_traj_clients[i]->isServerConnected()) {
            ROS_WARN_STREAM("leg_" << i << " actionlib server is not connected!");
            continue;
        }

        _traj_msgs[i].header.stamp = ros::Time::now();

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = _traj_msgs[i];
        _traj_clients[i]->sendGoal(goal);
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
