#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <opt_view/ProjectedView.h>

#include <Eigen/QR>

#define NODE_NAME "backprojection"
#include "pd_rosnode.h"
#include "common.h"

#define POINTS_NO 5

// Moore penrose inverse
/*#define lpinv(A) (((A).transpose() * (A)).inverse() * ((A).transpose()))
#define pinv(A) (lpinv(A))
#define rpinv(A) ((A).transpose() * ((A)  * (A).transpose()).inverse())*/


typedef Eigen::Matrix<double, 3, 4> ProjectionMatrix;

class BackProjection
{
	ProjectionMatrix  intrinsicMatrix;
	int pxWidth, pxHeight;
	std::vector<Eigen::Vector2d> pointsToProject;
	Eigen::Isometry3d pose;
	Eigen::Quaterniond toCamera;

	bool initialized;
	void updatePointsToProject (double width, double height);
	Eigen::Vector3d backProject (Eigen::Vector2d imagePoint);

	ProjectionMatrix getExtrinsicMatrix ();

public:
	BackProjection ();

	void updateParams (const sensor_msgs::CameraInfo &cameraInfo);
	void updatePose (const Eigen::Isometry3d &newPose);
	std::vector<Eigen::Vector3d> backProjectPoints ();

	inline bool isInitialized () {
		return initialized;
	}
};

class BackProjectionNode : public PdRosNode
{
	ros::Subscriber cameraInfoSub;
	ros::Subscriber cameraOdomSub;
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


