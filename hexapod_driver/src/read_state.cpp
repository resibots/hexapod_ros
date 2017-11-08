#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace global {
    ros::Time prev_time;
}

void joint_getter(const sensor_msgs::JointState::ConstPtr& msg)
{
    ros::Duration dur = ros::Time::now() - global::prev_time;
    if (dur.toSec() >= 0.1) {
        global::prev_time = ros::Time::now();
        std::cout << "Positions: ";
        for (int i = 0; i < msg->position.size(); i++) {
            std::cout << msg->position[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "Velocities: ";
        for (int i = 0; i < msg->velocity.size(); i++) {
            std::cout << msg->velocity[i] << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_reader");
    ros::NodeHandle n;

    global::prev_time = ros::Time::now();

    ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_getter);

    ros::spin();
}
