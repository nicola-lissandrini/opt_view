#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <xmlrpcpp/XmlRpc.h>

#define QUA ROS_INFO("\e[33mReached %d\e[0m:%s", __LINE__, __FILE__);

double paramDouble(XmlRpc::XmlRpcValue &param);
std::string paramString (XmlRpc::XmlRpcValue &param);
tf::Pose paramPose (XmlRpc::XmlRpcValue &param);
Eigen::Isometry3d paramPoseEigen(XmlRpc::XmlRpcValue &param);
Eigen::MatrixXd paramMatrix (XmlRpc::XmlRpcValue &param, int rows, int cols);

#endif // COMMON_H
