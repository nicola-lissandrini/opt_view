#include "self_positioning.h"

using namespace std;
using namespace Eigen;

SelfPositioning::SelfPositioning ():
	started(false)
{
}

void SelfPositioning::setParams (XmlRpc::XmlRpcValue &_params)
{
	agentsNo = int (_params["agents_no"]);

	computedFormation.resize (agentsNo);
	currentFormation.resize (agentsNo);
	formationGeometry.resize (agentsNo);

	targetPose = Translation3d (Vector3d::Zero());

	for (int i = 0; i < agentsNo; i++) {
		formationGeometry[i] = paramPoseEigen (_params["formation_geometry"][i]);
	}
}

void SelfPositioning::setTargetPose (const Pose &pose) {
	targetPose = pose;
}

void SelfPositioning::updateFormation (const Isometry3d &transformation, int i)
{
	formationGeometry[i] = formationGeometry[i] * transformation;
}

int SelfPositioning::setFormationState (const Formation &formation)
{
	if (formation.size () != currentFormation.size ())
		return -1;

	if (!started) {
		started = true;
	}

	currentFormation = formation;

	return 0;
}

void SelfPositioning::compute ()
{
	for (int i = 0; i < agentsNo; i++)
		computedFormation[i] = targetPose * formationGeometry[i];
}


Formation SelfPositioning::getComputedFormation () {
	return computedFormation;
}


























