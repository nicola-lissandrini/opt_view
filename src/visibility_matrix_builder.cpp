#include "visibility_matrix_builder.h"

#include <eigen_conversions/eigen_msg.h>

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

void VisibilityMatrix::toSparse (Sparsei &sparse) const  {
	sparse.resize (rows (), cols ());
	sparse.setFromTriplets (elements.begin (), elements.end ());
}

void VisibilityMatrix::fromMsg(const opt_view::SparseMatrixInt &matrixMsg)
{
	region.cellSize = matrixMsg.cellSize;
	region.rangeMin = Vector2d (matrixMsg.origin.x,
								matrixMsg.origin.y);
	region.rangeMax = region.rangeMin + Vector2d (matrixMsg.cellSize * matrixMsg.cols,
												  matrixMsg.cellSize * matrixMsg.rows);
	region.pose = Translation2d (0,0) * Rotation2Dd(0);

	clear ();
	setRegion (region);

	for (int i = 0; i < matrixMsg.elements.size (); i++) {
		opt_view::TripletInt curr = matrixMsg.elements[i];
		set (curr.i, curr.j, curr.val);
	}
}


void VisibilityMatrix::setRegion (const Region &newRegion) {
	region = newRegion;
}

VisibilityMatrix VisibilityMatrix::operator + (const VisibilityMatrix &other)
{
	Sparsei selfSparse, otherSparse, resultSparse;

	this->toSparse (selfSparse);
	other.toSparse (otherSparse);

	resultSparse = selfSparse + otherSparse;

	VisibilityMatrix ret(resultSparse);
	ret.setRegion (region);
	return ret;
}

VisibilityMatrix::VisibilityMatrix (const Sparsei &sparse)
{
	for (int k = 0; k <	sparse.outerSize (); k++) {
		for (Sparsei::InnerIterator it(sparse, k); it; ++it)
			set (it.row (), it.col(), it.value ());
	}
}

void VisibilityMatrix::clear() {
	elements.clear ();
}

void VisibilityMatrix::set (int i, int j, u_int8_t val) {
	elements.push_back (Tripleti (i, j, val));
}

const Tripleti &VisibilityMatrix::getElement(int i) const {
	return elements[i];
}

int Region::maxH () const {
	return round ((rangeMax(0) - rangeMin(0)) / cellSize);
}

int Region::maxK () const {
	return round ((rangeMax(1) - rangeMin(1)) / cellSize);
}

Vector2d Region::operator () (int h, int k) const
{
	assert (h < maxH() && k < maxK());

	return pose * Vector2d (rangeMin(0) + h * cellSize,
							rangeMin(1) + k * cellSize);
}

Vector2i Region::indicesFromWorld (Vector2d pt) const
{
	Vector2d ptLocalFrame = pose.inverse () * pt;


	return Vector2i (round ((ptLocalFrame(0) - rangeMin(0)) / cellSize),
					 round ((ptLocalFrame(1) - rangeMin(1)) / cellSize));
}

Vector2d Region::worldStart () const {
	return pose * rangeMin;
}

Vector2d Region::worldEnd () const {
	return pose * rangeMax;
}

void VisibilityMatrixBuilder::setPose (const Isometry3d &pose)
{
	Vector3d pos3d = pose.translation ();
	Quaterniond rot3d(pose.rotation ());
	local.pose = Translation2d (pos3d(0), pos3d(1)) * Rotation2Dd (0);

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
	global.pose = Translation2d (0,0) * Rotation2Dd (0);
	local.pose = global.pose;

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

	if (pointHomog.dot (line) > 0)
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


			if (checkInside (curr, lines)) {
				Vector2i globalIndices = global.indicesFromWorld (curr);

				if (globalIndices(0) >= global.maxH () ||
						globalIndices(1) >= global.maxK ())
					ROS_ERROR ("Local region is out of global. Trimming view. Possible Errors");
				else
					visibilityMatrix.set (globalIndices(0),
										  globalIndices(1));
			}
		}
	}
}

void VisibilityMatrixBuilder::compute (VisibilityMatrix &visibilityMatrix)
{
	vector<Line> lines;

	visibilityMatrix.clear ();
	visibilityMatrix.setRegion (global);
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


void visibilityToOccupancyMsg(const VisibilityMatrix &matrix, nav_msgs::OccupancyGrid &rvizMatrix)
{
	nav_msgs::MapMetaData mapData;

	mapData.map_load_time = ros::Time::now ();
	mapData.resolution = matrix.getRegion ().cellSize;
	mapData.width = matrix.cols ();
	mapData.height = matrix.rows ();
	mapData.origin.position.x = matrix.getRegion ().rangeMin(0);
	mapData.origin.position.y = matrix.getRegion ().rangeMin(1);

	rvizMatrix.header.seq = 0;
	rvizMatrix.header.stamp = ros::Time::now ();
	rvizMatrix.header.frame_id = "map";
	rvizMatrix.info = mapData;
	rvizMatrix.data = vector<int8_t> (matrix.rows () * matrix.cols(), 0);

	for (int i = 0; i < matrix.count (); i++) {
		Tripleti triplet = matrix.getElement (i);
		rvizMatrix.data[triplet.row () * matrix.cols () + triplet.col ()] = 33 * triplet.value ();
	}
}
