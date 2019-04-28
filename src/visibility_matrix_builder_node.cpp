#include "visibility_matrix_builder_node.h"
#include <opt_view/SparseMatrixInt.h>

using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace Eigen;

VisibilityMatrixBuilderNode::VisibilityMatrixBuilderNode():
	PdRosNode(NODE_NAME),
	doPublishRviz(false)
{
	initParams ();
	initROS ();
}

void VisibilityMatrixBuilderNode::initParams ()
{
	builder.setParams (params);

	doPublishRviz = bool (params["publish_rviz"]);
	agentId = int (params["agent_id"]);
}

void VisibilityMatrixBuilderNode::initROS ()
{
	cameraOdomSub = nh.subscribe (paramString (params["camera_odom_topic"]), 1, &VisibilityMatrixBuilderNode::cameraOdomCallback, this);
	projViewSub = nh.subscribe (paramString (params["projected_view_topic"]), 1, &VisibilityMatrixBuilderNode::projViewCallback, this);
	matrixRvizPub = nh.advertise<nav_msgs::OccupancyGrid> (paramString (params["occupancy_grid_topic"]), 1);
	sparseMatrixPub = nh.advertise<opt_view::SparseMatrixInt> (paramString (params["visibility_matrix_topic"]), 1);
}

void VisibilityMatrixBuilderNode::cameraOdomCallback (const nav_msgs::Odometry &odom)
{
	Isometry3d eigenPose;

	tf::poseMsgToEigen (odom.pose.pose, eigenPose);

	builder.setPose (eigenPose);
}

void VisibilityMatrixBuilderNode::projViewCallback (const opt_view::ProjectedView &newProjView)
{
	vector<Vector3d> points;

	if (isnan (newProjView.points[0].x)) {
		NODE_INFO ("Received nan in message. Skipping.");
		return;
	}

	points.resize (newProjView.points.size ());

	for (int i = 0; i < newProjView.points.size (); i++) {
		tf::pointMsgToEigen (newProjView.points[i], points[i]);
	}

	if (!builder.setPoints (points)) {
		NODE_INFO ("Invalid points number. Skipping.");
		return;
	}
}

void VisibilityMatrixBuilderNode::publishRviz (const VisibilityMatrix &matrix)
{
	nav_msgs::OccupancyGrid rvizMatrix;

	visibilityToOccupancyMsg (matrix, rvizMatrix);

	matrixRvizPub.publish (rvizMatrix);
}

void VisibilityMatrixBuilderNode::publishSparseMsg (const VisibilityMatrix &matrix)
{
	opt_view::SparseMatrixInt sparseMsg;

	sparseMsg.rows = matrix.rows ();
	sparseMsg.cols = matrix.cols ();
	sparseMsg.cellSize = matrix.getRegion ().cellSize;
	sparseMsg.origin.x = matrix.getRegion ().rangeMin(0);
	sparseMsg.origin.y = matrix.getRegion ().rangeMin(1);
	sparseMsg.origin.z = 0;
	sparseMsg.agentId = agentId;

	sparseMsg.elements.resize (matrix.count ());
	for (int i = 0; i < matrix.count (); i++) {
		Tripleti curr = matrix.getElement (i);
		sparseMsg.elements[i].i = curr.row ();
		sparseMsg.elements[i].j = curr.col ();
		sparseMsg.elements[i].val = curr.value ();
	}

	sparseMatrixPub.publish (sparseMsg);
}

int VisibilityMatrixBuilderNode::actions ()
{
	if (builder.isReady () && !builder.hasComputed ()) {
		visibilityMatrix.clear ();
		builder.compute (visibilityMatrix);
		publishSparseMsg (visibilityMatrix);

		if (doPublishRviz)
			publishRviz (visibilityMatrix);
	}

	return 0;
}


int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	VisibilityMatrixBuilderNode vmb;

	return vmb.spin ();
}
