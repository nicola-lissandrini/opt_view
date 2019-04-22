#ifndef VISIBILITY_GRID_H
#define VISIBILITY_GRID_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <opt_view/ProjectedView.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>

namespace gazebo {

class VisibilityGrid : public VisualPlugin
{

	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber matrixSub;
	ros::Subscriber poseSub;
	std::thread rosQueueThread;
	rendering::VisualPtr visual;
	event::ConnectionPtr updateConnection;

	ignition::math::Pose3d pose;

	void buildVisual ();

	void queueThread ();

public:
	VisibilityGrid ():
		VisualPlugin ()
	{}

	void Load (rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

	void updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView);
	void odometryCallback (const nav_msgs::OdometryConstPtr &odom);
};

}
#endif // VISIBILITY_GRID_H
