#include <hexapod_driver/hexapod.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_driver");
    ros::NodeHandle n;

    auto hexa = std::make_shared<hexapod_ros::Hexapod>(n);
    // hexa->move({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, 5.0);
    hexa->move(std::vector<double>(36, 0.0), 5.0);
    while (ros::ok()) {
        auto t = hexa->transform();
    }
    // auto p = t.getOrigin();
    // ROS_INFO_STREAM("Pos: " << p[0] << " " << p[1] << " " << p[2]);
}
