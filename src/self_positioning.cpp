#include "self_positioning.h"

using namespace std;
using namespace Eigen;

SelfPositioning::SelfPositioning ()
{
}

void SelfPositioning::setParams (XmlRpc::XmlRpcValue &_params)
{
	params.agentsNo = int (_params["agents_no"]);
	params.kg = paramDouble (_params["const"]["kg"]);
	params.dg = paramDouble (_params["const"]["dg"]);
	params.dk = paramDouble (_params["const"]["dk"]);

	targetFormation.resize (params.agentsNo);
	computedFormation.resize (params.agentsNo);
	currentFormation.resize (params.agentsNo);

	initFormation (_params);
}

void SelfPositioning::initFormation (XmlRpc::XmlRpcValue &_params)
{
	for (int i = 0; i < params.agentsNo; i++) {
		targetFormation[i] = paramPoseEigen (_params["initial_formation"][i]);
	}
}

int SelfPositioning::setTargetFormation (const Formation &formation)
{
	if (formation.size () != targetFormation.size ())
		return -1;

	targetFormation = formation;
	return 0;
}

int SelfPositioning::setFormationState (const Formation &formation)
{
	if (formation.size () != currentFormation.size ())
		return -1;

	currentFormation = formation;
	return 0;
}

void SelfPositioning::compute ()
{
	computeTrajectory ();
}

Force SelfPositioning::computeGoalForce (int i)
{
	Pose dist = currentFormation[i] * targetFormation[i].inverse ();

	return params.kg * dist.translation () / dist.translation ().norm ();
}


void SelfPositioning::computeTrajectory ()
{
	for (int i = 0; i < targetFormation.size (); i++) {
		Force currFg = computeGoalForce (i);
		Pose oldPose = currentFormation[i];
		Pose newPose;

		newPose = oldPose * Translation3d (currFg * params.dk);

		cout << oldPose.translation ()<< "\n -- \n" << newPose.translation ()<< "\n\n +++++" << endl;
		computedFormation[i] = newPose;
	}
}

Formation SelfPositioning::getComputedFormation () {
	return computedFormation;
}


























