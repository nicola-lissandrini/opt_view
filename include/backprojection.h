#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <opt_view/ProjectedView.h>

#include <Eigen/SVD>

#define NODE_NAME "backprojection"
#include "pd_rosnode.h"
#include "common.h"

#define POINTS_NO 5

#define pinv(A) ((A).completeOrthogonalDecomposition().pseudoInverse())

class BackProjection
{
	Eigen::Matrix<double, 3, 4>  projectionMatrix;
	int pxWidth, pxHeight;
	std::vector<Eigen::Vector2d> pointsToProject;
	Eigen::Isometry3d pose;

	bool initialized;
	void updatePointsToProject (double width, double height);
	Eigen::Vector3d backProject (Eigen::Vector2d imagePoint);

public:
	BackProjection ():
		initialized(false),
		pointsToProject(POINTS_NO)
	{}

	void updateParams (const sensor_msgs::CameraInfo &cameraInfo);
	void updatePose (const Eigen::Isometry3d &newPose);
	std::vector<Eigen::Vector3d> backProjectPoints ();
};

class BackProjectionNode : public PdRosNode
{
	ros::Subscriber cameraInfoSub;
	ros::Publisher projectedPointsPub;
	BackProjection backProjection;

	int actions ();
	void initParams ();
	void initROS ();

public:
	BackProjectionNode ();

	void cameraInfoCallback (const sensor_msgs::CameraInfo &cameraInfo);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};



#endif // BACKPROJECTION_H

