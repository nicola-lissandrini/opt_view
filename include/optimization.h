#ifndef OPTIMIZATION_NODE_H
#define OPTIMIZATION_NODE_H

#define NODE_NAME "optimization"
#include "pd_rosnode.h"
#include "visibility_matrix_builder.h"

#include <opt_view/SparseMatrixInt.h>
#include <opt_view/MultiagentPose.h>
#include <nav_msgs/Odometry.h>

enum OptimizationType {
	OPTIMIZATION_CW,
	OPTIMIZATION_CCW
};

class Optimization
{
	Eigen::Isometry2d cameraPose;

	struct Params {
		OptimizationType type;
		double angleStep;
	} params;

public:
	Optimization () {}

	int initParams(XmlRpc::XmlRpcValue &rpcParams);
	void setPose (const Eigen::Isometry3d &cameraPose3D);
};

class OptimizationNode : public PdRosNode
{
	ros::Subscriber selfMatrixSub;
	ros::Subscriber neighborMatrixSub;
	ros::Subscriber cameraOdomSub;
	ros::Publisher rotationStepPub;
	Optimization optimization;
	int agentId, neighborId;

	int actions ();
	void initParams ();
	void initROS ();

public:
	OptimizationNode ();

	void selfMatrixCallback (const opt_view::SparseMatrixInt &matrix);
	void neighborMatrixCallback (const opt_view::SparseMatrixInt &matrix);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};

#endif // OPTIMIZATION_NODE_H
