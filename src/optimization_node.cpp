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
	string topicPre = paramString (params["multiagent_topics_pre"]);
	string matrixTopicPost = paramString(params["visibility_matrix_topic_post"]);
	string agentIdStr = to_string (agentId);
	string neighborIdStr = to_string (neighborId);
	string rotationTopic = paramString (params["rotation_topic"]);

	selfMatrixSub = nh.subscribe (topicPre + agentIdStr + matrixTopicPost, 1, &OptimizationNode::selfMatrixCallback, this);
	neighborMatrixSub = nh.subscribe (topicPre + neighborIdStr + matrixTopicPost, 1, &OptimizationNode::selfMatrixCallback, this);
	rotationStepPub = nh.advertise<opt_view::MultiagentPose> (rotationTopic, 1);
}


void OptimizationNode::selfMatrixCallback (const opt_view::SparseMatrixInt &matrix)
{

}

void OptimizationNode::neighborMatrixCallback (const opt_view::SparseMatrixInt &matrix)
{

}

void OptimizationNode::cameraOdomCallback (const nav_msgs::Odometry &odom)
{
	Isometry3d pose;

	tf::poseMsgToEigen (odom.pose.pose, pose);

	optimization.setPose (pose);
}

int OptimizationNode::actions ()
{
	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	OptimizationNode on;

	return on.spin ();
}






















































