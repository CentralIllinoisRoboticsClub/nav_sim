#include <tf2/transform_datatypes.h>

float get_yaw(geometry_msgs::Pose pose)
{
    double roll, pitch, yaw;
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 quat_matrix(q);
    quat_matrix.getRPY(roll, pitch, yaw);
    return (float)yaw;
}
