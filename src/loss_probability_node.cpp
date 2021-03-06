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
	string totalAreaTopic = paramString (params["total_area_topic"]);

	visibilityMatrixSubs.resize (lossProbability.getAgentsNo ());

	for (int i = 0; i < visibilityMatrixSubs.size (); i++)
		visibilityMatrixSubs[i] = nh.subscribe (topicPre + to_string (i) + matrixTopicPost, 1, &LossProbabilityNode::visibilityCallback, this);
	targetPoseSub = nh.subscribe (targetPoseTopic, 1, &LossProbabilityNode::targetPoseCallback, this);
	probabilityPub = nh.advertise<std_msgs::Float64> (probabilityTopic, 1);
	matrixPub = nh.advertise<nav_msgs::OccupancyGrid> ("total_view", 1);
	totalAreaPub = nh.advertise<std_msgs::Float64> (totalAreaTopic, 1);
}

void LossProbabilityNode::publishRviz (const VisibilityMatrix &matrix)
{
	nav_msgs::OccupancyGrid rvizMatrix;

	visibilityToOccupancyMsg (matrix, rvizMatrix);

	matrixPub.publish (rvizMatrix);
}

void LossProbabilityNode::visibilityCallback (const opt_view::SparseMatrixInt &visibility) {
	lossProbability.updateVisibility (visibility);
}

void LossProbabilityNode::targetPoseCallback (const nav_msgs::Odometry &newPose)
{
	Isometry3d eigenPose;
	Vector3d eigenVel;

	tf::poseMsgToEigen (newPose.pose.pose, eigenPose);
	tf::vectorMsgToEigen (newPose.twist.twist.linear, eigenVel);

	lossProbability.updateTargetPose (eigenPose, eigenVel.head<2> ());
}

int LossProbabilityNode::actions ()
{
	if (lossProbability.isReady ()) {
		std_msgs::Float64 probMsg, totalMsg;

		probMsg.data = lossProbability.compute ();
		totalMsg.data = lossProbability.computeTotalArea ();


		probabilityPub.publish (probMsg);
		totalAreaPub.publish (totalMsg);
		publishRviz (lossProbability.getTotalView ());
	}

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	LossProbabilityNode lpn;

	return lpn.spin ();
}
