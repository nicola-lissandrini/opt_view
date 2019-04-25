#ifndef VISIBILITY_MATRIX_BUILDER_NODE_H
#define VISIBILITY_MATRIX_BUILDER_NODE_H

#define NODE_NAME "visibility_matrix_builder"
#include "pd_rosnode.h"
#include "visibility_matrix_builder.h"

class VisibilityMatrixBuilderNode : public PdRosNode
{
	ros::Subscriber projViewSub;
	ros::Subscriber cameraOdomSub;
	ros::Publisher matrixRvizPub;
	ros::Publisher sparseMatrixPub;
	VisibilityMatrixBuilder builder;
	VisibilityMatrix visibilityMatrix;
	nav_msgs::MapMetaData mapData;


	int actions ();
	void initParams ();
	void initROS ();
	void publishRviz(const VisibilityMatrix &matrix);
	void publishSparseMsg(const VisibilityMatrix &matrix);

public:
	VisibilityMatrixBuilderNode();

	void projViewCallback (const opt_view::ProjectedView &newProjView);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};

#endif // VISIBILITY_MATRIX_BUILDER_NODE_H
