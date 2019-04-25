#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Geometry>
#include <xmlrpcpp/XmlRpc.h>

double paramDouble(XmlRpc::XmlRpcValue &param);
std::string paramString (XmlRpc::XmlRpcValue &param);
tf::Pose paramPose (XmlRpc::XmlRpcValue &param);
Eigen::Isometry3d paramPoseEigen (XmlRpc::XmlRpcValue &param);


#endif // COMMON_H
