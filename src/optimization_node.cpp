#include "optimization.h"


using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace Eigen;

OptimizationNode::OptimizationNode ():
	PdRosNode (NODE_NAME)
{
	initParams ();
	initROS ();
}

void OptimizationNode::initParams ()
{
	if (optimization.initParams (params)) {
		NODE_ERROR ("Invalid optimization_type. Quitting.");
		shutdown ();
		return;
	}
}

void OptimizationNode::initROS ()
{
	agentId = int (params["agent_id"]);
	neighborId = int (params["neighbor_id"]);
	doPublishRviz = bool (params["rviz_publish"]);

	string topicPre = paramString (params["multiagent_topics_pre"]);
	string matrixTopicPost = paramString(params["visibility_matrix_topic_post"]);
	string cameraTopicPost = paramString(params["camera_pose_topic_post"]);
	string agentIdStr = to_string (agentId);
	string neighborIdStr = to_string (neighborId);
	string rotationTopic = paramString (params["rotation_topic"]);
	string rvizTopic = paramString (params["rviz_topic"]);
	string imbalanceTopic = paramString (params["imbalance_topic"]);
	//string lossProbabilityTopic = paramString (params["loss_probability_topic"]);

	selfMatrixSub = nh.subscribe (topicPre + agentIdStr + matrixTopicPost, 1, &OptimizationNode::selfMatrixCallback, this);
	neighborMatrixSub = nh.subscribe (topicPre + neighborIdStr + matrixTopicPost, 1, &OptimizationNode::neighborMatrixCallback, this);
	cameraOdomSub = nh.subscribe (topicPre + agentIdStr + cameraTopicPost, 1, &OptimizationNode::cameraOdomCallback, this);
//	lossProbabilitySub = nh.subscribe (lossProbabilityTopic, 1, &OptimizationNode::lossProbabilityCallback, this);
	rotationStepPub = nh.advertise<opt_view::MultiagentPose> (rotationTopic, 1);
	imbalancePub = nh.advertise<std_msgs::Int64> (imbalanceTopic, 1);
	rvizPub = nh.advertise<nav_msgs::OccupancyGrid> (rvizTopic, 1);
}


void OptimizationNode::selfMatrixCallback (const opt_view::SparseMatrixInt &matrix) {
	optimization.setSelfFromMsg (matrix);
}

void OptimizationNode::neighborMatrixCallback (const opt_view::SparseMatrixInt &matrix) {
	optimization.setNeighborFromMsg (matrix);
}

void OptimizationNode::lossProbabilityCallback (const std_msgs::Float64 &lossMsg) {
	optimization.setLossProbability (lossMsg.data);
}

void OptimizationNode::cameraOdomCallback (const nav_msgs::Odometry &odom)
{
	Isometry3d pose;

	tf::poseMsgToEigen (odom.pose.pose, pose);

	optimization.setPose (pose);
}

void OptimizationNode::publishRviz (const VisibilityMatrix &matrix)
{
	nav_msgs::OccupancyGrid rvizMatrix;

	visibilityToOccupancyMsg (matrix, rvizMatrix);

	rvizPub.publish (rvizMatrix);
}

void OptimizationNode::publishImbalanceFactor ()
{
	std_msgs::Int64 imbalanceMsg;
	imbalanceMsg.data = optimization.getResults ().imbalanceFactor;
	imbalancePub.publish (imbalanceMsg);
}

void OptimizationNode::publishOptimization ()
{
	double angleStep;
	Isometry3d poseStep;
	opt_view::MultiagentPose poseStepMsg;

	angleStep = optimization.getResults ().optimizationStep;
	poseStep = Translation3d (0, 0, 0) * AngleAxisd (angleStep, Vector3d::UnitZ ());

	poseStepMsg.agent_id = agentId;
	tf::poseEigenToMsg (poseStep, poseStepMsg.target_pose.pose);

	rotationStepPub.publish (poseStepMsg);
}

int OptimizationNode::actions ()
{
	if (!optimization.isReady ())
		return 0;

	optimization.optimize ();

	publishImbalanceFactor ();
	publishOptimization ();

	if (doPublishRviz)
		publishRviz (optimization.getOverlappingVisibility ());

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	OptimizationNode on;

	return on.spin ();
}






















































