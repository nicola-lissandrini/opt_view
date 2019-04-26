#ifndef PD_ROSNODE_H
#define PD_ROSNODE_H

#include "common.h"

#define NODE_INFO(...) ROS_INFO("\e[38;5;82m\e[1m"  NODE_NAME ":\e[0m " __VA_ARGS__)
#define NODE_ERROR(...) ROS_ERROR("\e[38;5;82m\e[1m"  NODE_NAME  ":\e[0m " __VA_ARGS__)

template<typename T>
class ReadyFlags
{
	std::map<T, bool> flags;
	bool updated;

public:
	ReadyFlags ():
		updated(false)
	{}

	void addFlag (T id, bool initialValue = false) {
		flags.insert (std::make_pair (id, initialValue));
	}
	void resetFlags () {
		for (typename std::map<T, bool>::iterator it = flags.begin (); it != flags.end (); it++)
			it->second = false;
	}
	void set (T id) {
		flags[id] = true;
		updated = true;
	}
	void reset (T id) {
		flags[id] = false;
		updated = true;
	}
	bool get (T id) const {
		return flags[id];
	}
	bool setProcessed () {
		updated = false;
		resetFlags ();
	}
	bool isProcessed () const {
		return !updated;
	}
	bool isReady () const {
		for (typename std::map<T, bool>::const_iterator it = flags.begin (); it != flags.end (); it++)
			if (!it->second)
				return false;
		return true;
	}
};

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
