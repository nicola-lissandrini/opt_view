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
	targetFormationSub = nh.subscribe (paramString (params["target_formation_sub"]), 1, &SelfPositioningNode::formationCallback, this);
	targetFormationSub = nh.subscribe (paramString (params["formation_sub"]), 1, &SelfPositioningNode::formationStateCallback, this);

	goalPub = nh.advertise<opt_view::MultiagentPose> (paramString (params["trajectory_pub"]), 100);
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

void SelfPositioningNode::formationCallback (const opt_view::Formation &newFormation)
{
	Formation eigenFormation;

	eigenFormation.resize (newFormation.formation.size ());
	formationMsgToEigen (newFormation, eigenFormation);

	if (selfPositioning.setTargetFormation (eigenFormation)){
		ROS_ERROR ("Invalid formation length in msg. Skipping.");
		return;
	}

}

void SelfPositioningNode::formationStateCallback(const opt_view::Formation &newFormation)
{
	Formation eigenFormation;

	eigenFormation.resize (newFormation.formation.size ());

	formationMsgToEigen (newFormation, eigenFormation);

	if (selfPositioning.setFormationState (eigenFormation)){
		ROS_ERROR ("Invalid formation length in msg: %d. Skipping.", 	eigenFormation.size ());
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
