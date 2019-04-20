#ifndef VISIBILITY_GRID_H
#define VISIBILITY_GRID_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>

namespace gazebo {

namespace rendering {

class VisibilityGrid : public ModelPlugin
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber rosSub;
	std::thread rosQueueThread;
	msgs::Visual plane, edge1, edge2;
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;

	void buildPlane (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);
	void buildEdge1 (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);
	void buildEdge2 (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);

protected:
	virtual void UpdateChild ();

public:
	VisibilityGrid ():
		ModelPlugin ()
	{}

	void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);
};

}

}
#endif // VISIBILITY_GRID_H
