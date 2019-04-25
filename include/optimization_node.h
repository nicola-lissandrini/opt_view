#ifndef OPTIMIZATION_NODE_H
#define OPTIMIZATION_NODE_H

#define NODE_NAME "optimization"
#include "pd_rosnode.h"
#include "visibility_matrix_builder.h"

#include <eigen_conversions/eigen_msg.h>
#include <opt_view/ProjectedView.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

class OptimizationNode : public PdRosNode
{
	ros::Subscriber projViewSub;
	ros::Subscriber cameraOdomSub;
	ros::Publisher matrixRvizPub;
	VisibilityMatrixBuilder builder;
	VisibilityMatrix visibilityMatrix;
	nav_msgs::MapMetaData mapData;

	int actions ();
	void initParams ();
	void initROS ();
	void publishRviz(const VisibilityMatrix &matrix);

public:
	OptimizationNode ();

	void projViewCallback (const opt_view::ProjectedView &newProjView);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};

#endif // OPTIMIZATION_NODE_H
