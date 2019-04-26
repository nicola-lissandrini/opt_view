#include "optimization.h"
#include <unsupported/Eigen/EulerAngles>
#include <mav_msgs/common.h>

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace Eigen;


Optimization::Optimization()
{
	flags.addFlag ("pose_set");
	flags.addFlag ("self_set");
	flags.addFlag ("neighbor_set");
}

int Optimization::initParams (XmlRpcValue &rpcParams)
{
	string typeStr = paramString (rpcParams["optimization_type"]);

	if (typeStr == "CCW")
		params.type = OPTIMIZATION_CCW;
	else {
		if (typeStr == "CW") {
			params.type = OPTIMIZATION_CW;
		} else {
			return -1;
		}
	}

	params.angleStep = paramDouble (rpcParams["angle_step"]);
	params.gain = paramDouble (rpcParams["proportional_gain"]);

	return 0;
}

void Optimization::setPose (const Isometry3d &cameraPose3D)
{
	Vector3d pos3D = cameraPose3D.translation ();
	Quaterniond rot3D(cameraPose3D.rotation ());
	double yaw = mav_msgs::yawFromQuaternion (rot3D);

	cameraPose = Translation2d (pos3D(0), pos3D(1)) * Rotation2Dd (yaw);
	flags.set ("pose_set");
}

void Optimization::setSelfFromMsg (const opt_view::SparseMatrixInt &matrixMsg) {
	setFromMsg (selfVisibility, matrixMsg);
	flags.set ("self_set");
}

void Optimization::setNeighborFromMsg (const opt_view::SparseMatrixInt &matrixMsg) {
	setFromMsg (neighborVisibility, matrixMsg);
	flags.set ("neighbor_set");
}

void Optimization::setFromMsg (VisibilityMatrix &visibility, const opt_view::SparseMatrixInt &matrixMsg)
{
	Region region;

	region.cellSize = matrixMsg.cellSize;
	region.rangeMin = Vector2d (matrixMsg.origin.x,
								matrixMsg.origin.y);
	region.rangeMax = region.rangeMin + Vector2d (matrixMsg.cellSize * matrixMsg.cols,
												  matrixMsg.cellSize * matrixMsg.rows);
	region.pose = Translation2d (0,0) * Rotation2Dd(0);

	visibility.clear ();
	visibility.setRegion (region);

	for (int i = 0; i < matrixMsg.elements.size (); i++) {
		opt_view::TripletInt curr = matrixMsg.elements[i];
		visibility.set (curr.i, curr.j, curr.val);
	}
}

void Optimization::computeTotalVisibility () {
	totalVisibility = selfVisibility + neighborVisibility;
}

int Optimization::getSpacialValue (int i, int j)
{
	Region global = selfVisibility.getRegion ();
	Vector2d pointWorld = global(i, j);
	Vector2d pointCamera = cameraPose.inverse () * pointWorld;

	if (pointCamera(1) > 0)
		return 1;
	else
		return -1;
}

void Optimization::computeOverlappingWithSpacial ()
{
	int spacialValue;

	overlappingVisibility.clear ();
	overlappingVisibility.setRegion (totalVisibility.getRegion ());

	for (int i = 0; i < totalVisibility.count (); i++) {
		const Tripleti &curr = totalVisibility.getElement (i);

		if (curr.value () > 1) {
			spacialValue = getSpacialValue (curr.row (), curr.col ());
			overlappingVisibility.set (curr.row (), curr.col (), spacialValue);
		}
	}
}

int Optimization::computeImbalanceFactor ()
{
	int imbalanceFactor = 0;

	computeOverlappingWithSpacial ();

	for (int i = 0; i < overlappingVisibility.count (); i++) {
		imbalanceFactor += overlappingVisibility.getElement (i).value ();
	}

	return imbalanceFactor;
}

double sign (double a) {
	return (a >= 0)? 1: -1;
}

double Optimization::computeStep () {
	return results.imbalanceFactor * params.gain * params.angleStep;
}

int Optimization::optimize ()
{
	computeTotalVisibility ();
	results.imbalanceFactor = computeImbalanceFactor ();
	results.optimizationStep = computeStep ();

	flags.setProcessed ();
	return 0;
}







































