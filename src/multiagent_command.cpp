#include <tf/tf.h>
#include <iostream>

#include "multiagent_command.h"
#define NODE_NAME "multiagent_command"
#include "pd_rosnode.h"

using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace tf;

MultiagentCommand::MultiagentCommand()
{
	initParams ();
	initROS ();
}

void MultiagentCommand::initParams ()
{
	try {
		nh.getParam (NODE_NAME, params);
		agentsCount = int (params["agents_no"]);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s",e.getMessage ().c_str ());
	}
}

void MultiagentCommand::initROS ()
{
	rate = new Rate (paramDouble (params["rate"]));

	agentsPub.resize (agentsCount);
	formation.formation.resize (agentsCount);
	agentsOdomSub.resize (agentsCount);

	commandSub = nh.subscribe (paramString (params["input_topic"]), 100, &MultiagentCommand::commandCallback, this);
	formationPub = nh.advertise<opt_view::Formation> (paramString (params["formation_topic"]), 1);


	for (int i = 0; i < agentsCount; i++) {
		stringstream pubTopic, odomTopic;

		pubTopic << paramString (params["topic_pre"]) << i << paramString (params["pose_topic_post"]);
		odomTopic << paramString (params["topic_pre"]) << i << paramString (params["odom_topic_post"]);
		agentsPub[i] = nh.advertise<geometry_msgs::PoseStamped> (pubTopic.str(), 1);
		agentsOdomSub[i] = nh.subscribe (odomTopic.str (), 1, &MultiagentCommand::odomCallback, this);
	}
}

int MultiagentCommand::spin ()
{
	while (ok ()) {

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

void MultiagentCommand::sendCommand (int agent, const geometry_msgs::PoseStamped &pose)
{
	agentsPub[agent].publish (pose);
}

void MultiagentCommand::commandCallback (const opt_view::MultiagentPose &command) {
	sendCommand (command.agent_id, command.target_pose);
}

void MultiagentCommand::odomCallback (const nav_msgs::Odometry &odom)
{
	int agentId = stoi (odom.child_frame_id);

	formation.formation[agentId].header = odom.header;
	formation.formation[agentId].pose = odom.pose.pose;
	formationPub.publish (formation);
}

int main (int argc, char *argv[])
{
	init (argc, argv,NODE_NAME);
	MultiagentCommand multiagentCommand;

	return multiagentCommand.spin ();
}
