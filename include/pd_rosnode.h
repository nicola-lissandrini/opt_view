#ifndef PD_ROSNODE_H
#define PD_ROSNODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Geometry>

#include "common.h"

#define NODE_INFO(...) ROS_INFO("\e[38;5;82m\e[1m"  NODE_NAME ":\e[0m " __VA_ARGS__)
#define NODE_ERROR(...) ROS_ERROR("\e[38;5;82m\e[1m"  NODE_NAME  ":\e[0m " __VA_ARGS__)
#define QUA ROS_INFO("\e[33mReached %d\e[0m:%s", __LINE__, __FILE__);

class PdRosNode
{
	std::string name;

protected:
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;

	virtual void initParams ();
	virtual void initROS ();

	virtual int actions () = 0;

public:
	PdRosNode (std::string _name);

	int spin ();
	std::string getName () {
		return name;
	}
};

#endif // PD_ROSNODE_H
