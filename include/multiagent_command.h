#ifndef MULTIAGENT_COMMAND_H
#define MULTIAGENT_COMMAND_H

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpc.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <opt_view/MultiagentPose.h>
#include <opt_view/Formation.h>

class MultiagentCommand
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	ros::Subscriber commandSub;
	ros::Publisher formationPub;
	std::vector<ros::Publisher> agentsPub;
	std::vector<ros::Subscriber> agentsOdomSub;
	XmlRpc::XmlRpcValue params;

	opt_view::Formation formation;

	void initROS ();
	void initParams ();
	void sendCommand (int agent, const geometry_msgs::PoseStamped &pose);

	int agentsCount;

public:
	MultiagentCommand();

	int spin ();

	void commandCallback (const opt_view::MultiagentPose &command);
	void odomCallback (const nav_msgs::Odometry &odom);

};

#endif // MULTIAGENT_COMMAND_H
