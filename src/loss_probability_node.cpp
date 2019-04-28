#include "loss_probability.h"

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace Eigen;

LossProbabilityNode::LossProbabilityNode():
	PdRosNode(NODE_NAME)
{
	initParams ();
	initROS ();
}

void LossProbabilityNode::initParams ()
{
	lossProbability.initParams (params);
}

void LossProbabilityNode::initROS ()
{
	string topicPre = paramString (params["multiagent_topics_pre"]);
	string matrixTopicPost = paramString (params["visibility_matrix_topic_post"]);
	string targetPoseTopic = paramString (params["target_pose_topic"]);
	string probabilityTopic = paramString (params["probability_topic"]);

	visibilityMatrixSubs.resize (lossProbability.getAgentsNo ());

	for (int i = 0; i < visibilityMatrixSubs.size (); i++)
		visibilityMatrixSubs[i] = nh.subscribe (topicPre + to_string (i) + matrixTopicPost, 1, &LossProbabilityNode::visibilityCallback, this);
	targetPoseSub = nh.subscribe (targetPoseTopic, 1, &LossProbabilityNode::targetPoseCallback, this);
	probabilityPub = nh.advertise<std_msgs::Float64> (probabilityTopic, 1);
}

void LossProbabilityNode::visibilityCallback (const opt_view::SparseMatrixInt &visibility)
{
	lossProbability.updateVisibility (visibility);
}

void LossProbabilityNode::targetPoseCallback (const nav_msgs::Odometry &newPose) {
	Isometry3d eigenPose;
	Vector3d eigenVel;

	tf::poseMsgToEigen (newPose.pose.pose, eigenPose);
	tf::vectorMsgToEigen (newPose.twist.twist.linear, eigenVel);

	lossProbability.updatePose (eigenPose, eigenVel.head<2> ());
}

int LossProbabilityNode::actions ()
{
	if (lossProbability.isReady ())
		lossProbability.compute ();

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	LossProbabilityNode lpn;

	return lpn.spin ();
}
