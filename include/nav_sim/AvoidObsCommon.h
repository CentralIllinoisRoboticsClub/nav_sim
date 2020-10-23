#include <tf2/utils.h>
#include <geometry_msgs/msg/pose.hpp>

float get_yaw(geometry_msgs::msg::Pose pose)
{
    double roll, pitch, yaw;
    //tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    //tf::Matrix3x3 quat_matrix(q);
    //quat_matrix.getRPY(roll, pitch, yaw);
    tf2::getYaw(pose.orientation);
    return (float)yaw;
}
