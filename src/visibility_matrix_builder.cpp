#include "visibility_matrix_builder.h"

#include <eigen_conversions/eigen_msg.h>
using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace Eigen;

Region paramRegion (XmlRpcValue &param)
{
	Region ret;

	ret.rangeMin(0) = paramDouble (param["x_min"]);
	ret.rangeMax(0) = paramDouble (param["x_max"]);
	ret.rangeMin(1) = paramDouble (param["y_min"]);
	ret.rangeMax(1) = paramDouble (param["y_max"]);

	return ret;
}

int Region::maxH () const {
	return (rangeMax(0) - rangeMin(0)) / cellSize;
}

int Region::maxK () const {
	return (rangeMax(1) - rangeMin(1)) / cellSize;
}

Vector2d Region::operator () (int h, int k) const
{
	assert (h < maxH() && k < maxK());
	return pose.inverse () * Vector2d (rangeMin(0) + h * cellSize,
							rangeMin(1) + k * cellSize);
}

Vector2i Region::worldIndices (Vector2d pt) const
{
	Vector2d ptLocalFrame = pose * pt;

	return Vector2i ((ptLocalFrame(0) - rangeMin(0)) / cellSize,
					 (ptLocalFrame(1) - rangeMin(1)) / cellSize);
}

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
	mapData.width = globalRegion.maxH ();
	mapData.height = globalRegion.maxK ();

	agentId = (int) params["agent_id"];
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
	Isometry3d pose = builder.getPose ();
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
			rvizMatrix.data[j * matrix.rows () + i] = (matrix(i,j) > 0 ? 50:0);
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

void VisibilityMatrixBuilder::setPose (const Isometry3d &pose)
{
	Vector3d pos3d = pose.translation ();
	Vector3d euler = pose.rotation ().eulerAngles (0, 1, 2);
	local.pose = Translation2d (pos3d(0), pos3d(1)) * Rotation2Dd (euler(2));
	poseSet = true;
	computed = false;
}

bool VisibilityMatrixBuilder::setPoints (const std::vector<Vector3d> &newPoints)
{
	if (newPoints.size () != POINTS_NO)
		return false;
	points.clear ();
	points = newPoints;
	center = points[CENTER_PT_INDEX].head<2> ();

	pointsSet = true;
	computed = false;

	return true;
}

void VisibilityMatrixBuilder::setParams (XmlRpcValue &rpcParams)
{
	double cellSize = paramDouble (rpcParams["cell_size"]);
	global = paramRegion (rpcParams["global_region"]);
	local = paramRegion (rpcParams["local_region"]);
	global.cellSize = cellSize;
	local.cellSize = cellSize;

	paramsSet = true;
	computed = false;
}

Line VisibilityMatrixBuilder::getLine (const Vector3d &a, const Vector3d &b)
{
	Line line;
	Matrix<double, 2, 3> phi;

	// Build matrix:
	// phi = [ a(1:2)' 1 ]
	//		 [ b(1:2)' 1 ]

	phi.block<1, 2> (0, 0) = a.head<2> ().transpose ();
	phi.block<1, 2> (1, 0) = b.head<2> ().transpose ();
	phi(0, 2) = 1;
	phi(1, 2) = 1;

	// Solve problem phi * line = 0
	FullPivLU<Matrix<double, 2, 3>> luDecomp(phi);
	line = luDecomp.kernel ();

	return line;
}

Vector2d VisibilityMatrixBuilder::indicesMap (int h, int k) {

}

void VisibilityMatrixBuilder::getLines (vector<Line> &lines)
{
	int i;
	lines.resize (LINES_NO);

	for (i = 0; i < LINES_NO - 1; i++) {
		lines[i] = getLine (points[i], points[i+1]);
	}
	lines[i] = getLine(points[i], points[0]);
}

int VisibilityMatrixBuilder::sideSign (const Vector2d &point, const Line &line)
{
	Vector3d pointHomog(point(0), point(1), 1);

	if (pointHomog.dot(line) > 0)
		return 1;
	else
		return -1;
}

void VisibilityMatrixBuilder::resetFlags()
{
	poseSet = false;
	pointsSet = false;
	computed = false;
}

bool VisibilityMatrixBuilder::checkInside (const Vector2d &point, const std::vector<Line> &lines)
{
	int centerSign, currPtSign;

	for (int i = 0; i < CORNERS_NO; i++) {
		centerSign = sideSign (center, lines[i]);
		currPtSign = sideSign (point, lines[i]);

		if (centerSign != currPtSign)
			return false;
	}

	return true;
}

void VisibilityMatrixBuilder::buildMatrix (VisibilityMatrix &visibilityMatrix, const vector<Line> &lines)
{
	for (int h = 0; h < local.maxH (); h++) {
		for (int k = 0; k < local.maxK (); k++) {
			Vector2d curr = local(h, k);

			cout << curr << endl << endl;

			if (checkInside (curr, lines)) {
				Vector2i globalIndices = local.worldIndices (curr);
				visibilityMatrix(globalIndices(0), globalIndices(1)) = 1;
			}
		}
	}
}

void VisibilityMatrixBuilder::compute (VisibilityMatrix &visibilityMatrix)
{
	vector<Line> lines;

	visibilityMatrix = MatrixXd::Zero (global.maxH (), global.maxK ());
	getLines (lines);
	buildMatrix (visibilityMatrix, lines);

	resetFlags ();
}

bool VisibilityMatrixBuilder::isReady() {
	return poseSet && pointsSet && paramsSet;
}

bool VisibilityMatrixBuilder::hasComputed() {
	return computed;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	VisibilityMatrixBuilderNode vmb;

	return vmb.spin ();
}
