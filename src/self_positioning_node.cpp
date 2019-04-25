#include <ros/ros.h>
#include "self_positioning.h"
#include <eigen_conversions/eigen_msg.h>

using namespace ros;
using namespace Eigen;

SelfPositioningNode::SelfPositioningNode ():
	PdRosNode (NODE_NAME),
	started(false)
{
	initParams ();
	initROS ();
}

void SelfPositioningNode::initParams ()
{
	selfPositioning.setParams (params);
}

void SelfPositioningNode::initROS ()
{
	targetPoseSub = nh.subscribe (paramString (params["target_sub"]), 1, &SelfPositioningNode::targetCallback, this);
	formationStateSub = nh.subscribe (paramString (params["formation_sub"]), 1, &SelfPositioningNode::formationStateCallback, this);
	angleSub = nh.subscribe (paramString (params["angle_sub"]), 1, &SelfPositioningNode::updateFormation, this);

	goalPub = nh.advertise<opt_view::MultiagentPose> (paramString (params["trajectory_pub"]), 100);
}

void SelfPositioningNode::updateFormation (const opt_view::MultiagentPose &transformation)
{
	Isometry3d transformEigen;
	int agentId = transformation.agent_id;
	tf::poseMsgToEigen (transformation.target_pose.pose, transformEigen);

	selfPositioning.updateFormation (transformEigen, agentId);
}

int SelfPositioningNode::actions ()
{
	if (!started) {
		return 0;
	}

	selfPositioning.compute ();
	publishTrajectory ();
	return 0;
}

void SelfPositioningNode::publishTrajectory ()
{
	Formation computedFormation = selfPositioning.getComputedFormation ();

	for (int i = 0; i < computedFormation.size (); i++) {
		opt_view::MultiagentPose targetPose;

		targetPose.agent_id = i;
		tf::poseEigenToMsg (computedFormation[i], targetPose.target_pose.pose);

		goalPub.publish (targetPose);
	}
}

void formationMsgToEigen (const opt_view::Formation &msg, Formation &eigenFormation)
{
	for (int i = 0; i < msg.formation.size (); i++) {
		Pose currPose;
		tf::poseMsgToEigen (msg.formation[i].pose, currPose);

		eigenFormation[i] = currPose;
	}
}

void SelfPositioningNode::targetCallback (const geometry_msgs::Pose &pose)
{
	Pose eigenPose;

	tf::poseMsgToEigen (pose, eigenPose);
	selfPositioning.setTargetPose (eigenPose);
}

void SelfPositioningNode::formationStateCallback(const opt_view::Formation &newFormation)
{
	Formation eigenFormation;

	eigenFormation.resize (newFormation.formation.size ());

	formationMsgToEigen (newFormation, eigenFormation);

	if (selfPositioning.setFormationState (eigenFormation)) {
		ROS_ERROR ("Invalid formation length in msg: %d. Skipping.", eigenFormation.size ());
		return;
	}

	if (!started) {
		NODE_INFO ("Received first odom.");
		started = true;
	}
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	SelfPositioningNode spn;

	return spn.spin ();
}
