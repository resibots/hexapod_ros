#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hexapod_controller/hexapod_controller_imu.hpp>
#include <hexapod_driver/hexapod_imu_velocity.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <thread>

#include <iostream>
using namespace hexapod_ros;

HexapodIMUVel::HexapodIMUVel(ros::NodeHandle nh, std::string ns) : _nh(nh), _namespace(ns)
{
    if (_namespace[0] != '/')
        _namespace = "/" + _namespace;
    init();
}

HexapodIMUVel::~HexapodIMUVel()
{
    ROS_INFO_STREAM("Relaxing...");
    relax();
}

bool HexapodIMUVel::init()
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

    // Init ROS related
    if (_odom_enable) {
        // create publisher to reset UKF filter (robot_localization)
        _reset_filter_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/set_pose", 1000);
    }
    //_mode_pub = _nh.advertise<std_msg::Float64>("/mode", 1000);
    //_time_pub = _nh.advertise<std_msg::Float64>("/time", 1000);
    //_ctrl_pub = _nh.advertise<std_msg::Float64MultiArray>("/ctrl", 1000);
    param_name = "/dynamixel_controllers/hexapod_controller/mode";
    if (!_nh.getParam(param_name, _duration)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        return false;
    }
    param_name = "/dynamixel_controllers/hexapod_controller/isRunning";
    if (!_nh.getParam(param_name, _isRunning)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        return false;
    }

    param_name = "/dynamixel_controllers/hexapod_controller/ctrl";
    if (!_nh.getParam(param_name, _ctrl)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        return false;
    }

    param_name = "/dynamixel_controllers/hexapod_controller/duration";
    if (!_nh.getParam(param_name, _duration)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << _nh.getNamespace() << ").");
        return false;
    }

    // Reset
    ROS_INFO_STREAM("Reset...");

    reset();

    // Get odom transformation
    if (_odom_enable || _mocap_odom_enable) {
        reset_odom();
    }
    return true;
}
void HexapodIMUVel::wait()
{
    while (_isRunning == 1) {
        _nh.getParam("/dynamixel_controllers/hexapod_controller/isRunning", _isRunning);
    }
}
void HexapodIMUVel::relax()
{
    wait();
    _mode = 3;
    _duration = 2.0;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/duration", _duration);
    _nh.setParam("/dynamixel_controllers/hexapod_controller/mode", _mode);

    ros::Duration(0.5).sleep();
}

void HexapodIMUVel::reset()
{
    wait();
    _mode = 1;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/mode", _mode);

    ros::Duration(0.5).sleep();
}

void HexapodIMUVel::zero()
{
    wait();
    _mode = 0;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/mode", _mode);
    _duration = 0.1;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/duration", _duration);
}

void HexapodIMUVel::move(std::vector<double> ctrl, double duration, bool reset)
{
    wait();
    _mode = 2;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/mode", _mode);
    _duration = duration;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/duration", _duration);
    _ctrl = ctrl;
    _nh.setParam("/dynamixel_controllers/hexapod_controller/controle", _ctrl);
    zero();
}

void HexapodIMUVel::reset_odom()
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

tf::Transform HexapodIMUVel::transform()
{
    if (!_odom_enable && !_mocap_odom_enable)
        ROS_ERROR_STREAM("Mocap and odom are disabled so Hexapod::transform should "
                         "not be called");

    _pos_update();
    return _init_pos.inverse() * _pos;
}

void HexapodIMUVel::_pos_update()
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
