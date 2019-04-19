#ifndef VISIBILITY_GRID_H
#define VISIBILITY_GRID_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Scene.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>

namespace gazebo {

namespace rendering {

class VisibilityGrid : public ModelPlugin
{
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber rosSub;
	std::thread rosQueueThread;
	rendering::VisualPtr visual;
	ScenePtr scene;

	event::ConnectionPtr updateConnection;

protected:
	virtual void UpdateChild ();

public:
	VisibilityGrid ():
		ModelPlugin ()
	{}

	void Load (VisualPtr _visual, sdf::ElementPtr _sdf);
};

}

}
#endif // VISIBILITY_GRID_H
