#include <hexapod_driver/hexapod_cartesian.hpp>
#include <hexapod_controller/hexapod_controller_cartesian.hpp>

// For all KDL things
#include <trac_ik/trac_ik.hpp>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <thread>

using namespace hexapod_ros;

HexapodCartesian::HexapodCartesian(ros::NodeHandle nh, std::string ns)
    : Hexapod(nh, ns)
{
    HexapodCartesian::init();
}

void HexapodCartesian::init()
{
    ros::NodeHandle nh("~");

    // ROS parameters
    std::string chain_start, chain_end, urdf_param, urdf;
    nh.getParam("chain_start", chain_start);
    nh.getParam("chain_end", chain_end);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    nh.getParam(urdf_param, urdf);
    double timeout, eps;
    nh.param("timeout", timeout, 0.005);
    nh.param("eps", eps, 1e-5);

    // ROS_INFO_STREAM("Parameters: "
    //     << chain_start << "; "
    //     << chain_end << "; "
    //     << urdf_param << "; "
    //     << urdf << "; "
    //     << timeout << "; "
    //     << eps);

    ROS_INFO_STREAM("Building the inverse kinematics model...");

    // Initialization of hexapod inverse kinematics
    std::array<std::string, 6> chain_ends;
    for (size_t i = 0; i < 6; ++i) {
        std::stringstream chain_end_full;
        chain_end_full << chain_end << (int)i;
        chain_ends[i] = chain_end_full.str();
    }

    _hexapod_ik = std::make_shared<multipod_ik::MultipodInverseKinematics<6>>(
        chain_start, chain_ends, urdf, timeout, eps);

    ROS_INFO_STREAM("Cartesian initialisation finished");
}

void HexapodCartesian::move(std::vector<double> ctrl, double duration, bool reset, double step)
{
    // Start from zero
    zero();

    ROS_INFO_STREAM("Starting to move");

    if (reset && (_odom_enable || _mocap_odom_enable)) {
        reset_odom();
    }

    // Initialize controller
    std::array<std::array<double, 3>, 6> scaling = {{{{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}}}};
    // template parameters : 6 legs, 100 samples per second
    hexapod_controller::HexapodControllerCartesian<6, 100> controller(ctrl, {}, scaling);

    // Clear message points
    for (size_t i = 0; i < 6; i++) {
        _traj_msgs[i].points.clear();
    }

    // Calculate points of trajectory
    bool computation_success = true;
    for (double t = 0.0; t <= duration; t += step) {
        std::array<std::array<double, 3>, 6> pos = controller.pos(t);

        for (size_t leg = 0; leg < 6; leg++) {
            KDL::Frame frame(KDL::Vector(pos[leg][0], pos[leg][1], pos[leg][2]));
            KDL::JntArray result(3);

            KDL::JntArray joint_angles(3);
            joint_angles(1) = 0.2;
            joint_angles(2) = 0.4;
            KDL::Frame reference_pose;
            _hexapod_ik->joint_to_cartesian(leg, joint_angles, reference_pose);
            frame.p += reference_pose.p;

            // TODO: optimise by reusing previous result (variable is currently short-lived)
            if (_hexapod_ik->cartesian_to_joint(leg, result, frame, result) < 0) {
                ROS_WARN_STREAM("Failed to solve inverse kinematics problem for leg "
                    << (int)leg); // << ". Target frame was\n"
                // << frame << "\nand result is " << result);
                // computation_success = false;
            }
            else {
                trajectory_msgs::JointTrajectoryPoint point;
                // point.positions.clear();

                point.positions.push_back(result(0));
                point.positions.push_back(result(1));
                point.positions.push_back(result(2));

                point.time_from_start = ros::Duration(t);

                _traj_msgs[leg].points.push_back(point);
            }
        }
    }

    if (computation_success) {
        _send_trajectories(duration);

        // Go back to zero
        zero();
    }
    else
        ROS_WARN_STREAM("Could not send trajectory due to failures of inverse kinematics (see above)");

    ROS_INFO_STREAM("Finished to move");
}
