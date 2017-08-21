#include <hexapod_driver/hexapod.hpp>
#include <hexapod_driver/hexapod_cartesian.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_driver");
    ros::NodeHandle n;

    // auto hexa = std::make_shared<hexapod_ros::Hexapod>(n);
    // hexa->move({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, 5.0);
    // auto t = hexa->transform();
    // auto p = t.getOrigin();
    // ROS_INFO_STREAM("Pos: " << p[0] << " " << p[1] << " " << p[2]);

    auto hexa = std::make_shared<hexapod_ros::HexapodCartesian>(n);
    hexa->move(
        {1, 0, 0.5,
            0, 0.25, 0.5,
            0.7, 0.25, 0.5,
            1, 0.5, 0.5,
            0, 0.25, 0.5,
            0.7, 0.75, 0.5,
            1, 0, 0.5,
            0, 0.25, 0.5,
            0.7, 0.25, 0.5,
            1, 0.5, 0.5,
            0, 0.25, 0.5,
            0.7, 0.75, 0.5,
            1, 0, 0.5,
            0, 0.25, 0.5,
            0.7, 0.25, 0.5,
            1, 0.5, 0.5,
            0, 0.25, 0.5,
            0.7, 0.75, 0.5},
        5.0);
    // auto t = hexa->transform();
    // auto p = t.getOrigin();
    // ROS_INFO_STREAM("Pos: " << p[0] << " " << p[1] << " " << p[2]);
}
