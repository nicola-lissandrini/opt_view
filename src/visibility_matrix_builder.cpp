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

void VisibilityMatrixBuilderNode::initParams () {
	builder.setParams (params);
}

void VisibilityMatrixBuilderNode::initROS ()
{
	cameraOdomSub = nh.subscribe (paramString (params["camera_odom_topic"]), 1, &VisibilityMatrixBuilderNode::cameraOdomCallback, this);
	projViewSub = nh.subscribe (paramString (params["projected_view_topic"]), 1, &VisibilityMatrixBuilderNode::projViewCallback, this);
}

void VisibilityMatrixBuilderNode::cameraOdomCallback (const nav_msgs::Odometry &odom)
{
	Isometry3d eigenPose;

	tf::poseMsgToEigen (odom.pose.pose, eigenPose);

	VisibilityMatrixBuilder.setPose (eigenPose);
}

void VisibilityMatrixBuilderNode::projViewCallback (const opt_view::ProjectedView &newProjView)
{
	vector<Vector3d> points;

	points.resize (newProjView.points.size ());

	for (int i = 0; i < newProjView.points.size (); i++) {
		tf::pointMsgToEigen (newProjView.points[i], points[i]);
	}

	if (!builder.setPoints (points)) {
		NODE_INFO ("Invalid points number. Skipping.");
		return;
	}
}

int VisibilityMatrixBuilderNode::actions ()
{

}

void VisibilityMatrixBuilder::setPose (const Isometry3d &pose) {
	cameraPose = pose;
}

bool VisibilityMatrixBuilder::setPoints (const std::vector<Vector3d> &newPoints)
{
	if (newPoints.size () != POINTS_NO)
		return false;
	points.clear ();
	points = newPoints;
	return true;
}

void VisibilityMatrixBuilder::setParams (const XmlRpcValue &_params)
{
	params.cellSize = paramDouble (_params["cell_size"]);
	params.xMinRelative = paramDouble (_params["x_min_relative"]);
	params.yMinRelative = paramDouble (_params["y_min_relative"]);

	double xMaxRelative = paramDouble (_params["x_max_relative"]);
	double yMaxRelative = paramDouble (_params["y_max_relative"]);

	params.countH = (xMaxRelative - params.xMinRelative) / params.cellSize;
	params.countK = (yMaxRelative - params.yMinRelative) / params.cellSize;

	initializeMatrix (params.countH, params.countK);
	initialized = true;
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
	line = phi.fullPivHouseholderQr ().solve (Vector3d::Zero ());
	return line;
}

Vector2d VisibilityMatrixBuilder::indicesMap (int h, int k) {
	return Vector2d (params.xMinRelative + double(h) * params.cellSize,
					 params.yMinRelative + double(k) * params.cellSize);
}

void VisibilityMatrixBuilder::getLines (vector<Line> &lines)
{
	lines.resize (LINES_NO);

	for (int i = 0; i < LINES_NO - 1; i++)
		lines[i] = getLine (points[i], points[i-1]);
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

bool VisibilityMatrixBuilder::checkInside (const Vector2d &point, const std::vector<Line> &lines)
{
	int centerSign, currPtSign;

	for (int i = 0; i < CENTER_PT_INDEX; i++) {
		centerSign = sideSign (point[CENTER_PT_INDEX], line[i]);
		currPtSign = sideSign (point[i], line[i]);

		if (centerSign != currPtSign)
			return false;
	}

	return true;
}

void VisibilityMatrixBuilder::buildMatrix (MatrixXd &visibilityMatrix, const vector<Line> &lines)
{
	for (int h = 0; h < params.countH; h++) {
		for (int k = 0; k < params.countK; k++) {
			Vector2d curr = indicesMap (h, k);

			if (checkInside (curr, lines))
				visibilityMatrix(h,k) = 1;
		}
	}
}

void VisibilityMatrixBuilder::compute (MatrixXd &visibilityMatrix)
{
	vector<Line> lines;

	visibilityMatrix = MatrixXd::Zero (params.countH, params.countK);
	getLines (lines);

	buildMatrix (visibilityMatrix, lines);
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	VisibilityMatrixBuilderNode vmb;

	return vmb.spin ();
}
