#include "visibility_matrix_builder.h"

using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace Eigen;

VisibilityMatrixBuilderNode::VisibilityMatrixBuilderNode():
	PdRosNode(NODE_NAME)
{
	initParams ();
	initROS ();
}

void VisibilityMatrixBuilderNode::initParams ()
{
	builder.setParams (params);
	Region globalRegion = builder.getGlobalRegion ();

	mapData.map_load_time = Time::now ();
	mapData.resolution = globalRegion.cellSize;
	mapData.width = globalRegion.maxK ();
	mapData.height = globalRegion.maxH ();

}

void VisibilityMatrixBuilderNode::initROS ()
{
	cameraOdomSub = nh.subscribe (paramString (params["camera_odom_topic"]), 1, &VisibilityMatrixBuilderNode::cameraOdomCallback, this);
	projViewSub = nh.subscribe (paramString (params["projected_view_topic"]), 1, &VisibilityMatrixBuilderNode::projViewCallback, this);
	matrixRvizPub = nh.advertise<nav_msgs::OccupancyGrid> (paramString (params["occupancy_grid_topic"]), 1);
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
	mapData.map_load_time = Time::now ();
	mapData.origin.position.x = builder.getGlobalRegion ().rangeMin(0);
	mapData.origin.position.y = builder.getGlobalRegion ().rangeMin(1);

	rvizMatrix.header.seq = 0;
	rvizMatrix.header.stamp = Time::now ();
	rvizMatrix.header.frame_id = "map";
	rvizMatrix.info = mapData;
	rvizMatrix.data.resize (matrix.rows () * matrix.cols());

	for (int i = 0; i < matrix.rows (); i++) {
		for (int j = 0; j < matrix.cols (); j++) {
			rvizMatrix.data[i * matrix.cols () + j] = (matrix(i,j) > 0 ? 50:0);
		}
	}

	matrixRvizPub.publish (rvizMatrix);
}

int VisibilityMatrixBuilderNode::actions ()
{
	if (builder.isReady () && !builder.hasComputed ()) {
		visibilityMatrix.resize (0,0);
		builder.compute (visibilityMatrix);
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
