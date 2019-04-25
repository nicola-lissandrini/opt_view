#include "visibility_matrix_builder_node.h"
#include <opt_view/SparseMatrixInt.h>

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
	sparseMatrixPub = nh.advertise<opt_view::SparseMatrixInt> (paramString (params["visibility_matrix_topic"]), 1);
}

void VisibilityMatrixBuilderNode::cameraOdomCallback (const nav_msgs::Odometry &odom)
{
	Isometry3d eigenPose;

	tf::poseMsgToEigen (odom.pose.pose, eigenPose);

	builder.setPose (eigenPose);


	class VisibilityMatrixBuilder
	{
		Eigen::Isometry3d cameraPose;
		std::vector<Eigen::Vector3d> points;
		Eigen::Vector2d center;

		Region global;
		Region local;
		bool poseSet, pointsSet, paramsSet, computed;

		void getLines (std::vector<Line> &lines);
		Line getLine (const Eigen::Vector3d &a, const Eigen::Vector3d &b);
		void buildMatrix (VisibilityMatrix &visibilityMatrix, const std::vector<Line> &lines);
		bool checkInside (const Eigen::Vector2d &point, const std::vector<Line> &lines);
		Eigen::Vector2d indicesMap (int h, int k);
		int sideSign (const Eigen::Vector2d &point, const Line &line);
		void resetFlags ();

	public:
		VisibilityMatrixBuilder ():
			poseSet(false),
			pointsSet(false),
			paramsSet(false)
		{}

		void setPose (const Eigen::Isometry3d &pose);
		bool setPoints (const std::vector<Eigen::Vector3d> &newPoints);
		void setParams (XmlRpc::XmlRpcValue &rpcParams);
		void compute (VisibilityMatrix &visibilityMatrix);

		bool isReady ();
		bool hasComputed ();


		inline Region getGlobalRegion () {
			return global;
		}
		inline Eigen::Isometry3d getPose () {
			return cameraPose;
		}
	};

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
	rvizMatrix.data = vector<int8_t> (matrix.rows () * matrix.cols(), 0);

	for (int i = 0; i < matrix.count (); i++) {
		Tripletui triplet = matrix.getElement (i);
		rvizMatrix.data[triplet.row () * matrix.cols () + triplet.col ()] = 50 * triplet.value ();
	}

	matrixRvizPub.publish (rvizMatrix);
}

void VisibilityMatrixBuilderNode::publishSparseMsg (const VisibilityMatrix &matrix)
{
	opt_view::SparseMatrixInt sparseMsg;

	sparseMsg.rows = matrix.rows ();
	sparseMsg.cols = matrix.cols ();

	sparseMsg.elements.resize (matrix.count ());
	for (int i = 0; i < matrix.count (); i++) {
		Tripletui curr = matrix.getElement (i);
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
		publishRviz (visibilityMatrix);
		publishSparseMsg (visibilityMatrix);
	}

	return 0;
}


int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	VisibilityMatrixBuilderNode vmb;

	return vmb.spin ();
}
