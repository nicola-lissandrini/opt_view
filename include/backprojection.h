#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/CameraInfo.h>
#include <opt_view/ProjectedView.h>

#define NODE_NAME "backprojection"
#include "pd_rosnode.h"
#include "common.h"

class BackProjection
{
public:
	BackProjection () {}
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
};



#endif // BACKPROJECTION_H
