#ifndef VISIBILITY_GRID_H
#define VISIBILITY_GRID_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <opt_view/ProjectedView.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>

namespace gazebo {

class ProjectedViewVisual
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	std::string parentName;
	msgs::Visual visualMsg;
	ignition::math::Pose3d projectedPose;
	opt_view::ProjectedView viewPoints;

	void build ();

public:
	ProjectedViewVisual (const std::string &_parentName,
						 transport::NodePtr _node, transport::PublisherPtr _visPub):
		parentName(_parentName),
		node(_node),
		visPub(_visPub)
	{
		build ();
	}

	void updatePoints (const opt_view::ProjectedView &view);
	void updatePose (const ignition::math::Pose3d &newPose);
	void redraw ();
};

class VisibilityGrid : public ModelPlugin
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber matrixSub;
	ros::Subscriber poseSub;
	std::thread rosQueueThread;
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;

	ProjectedViewVisual *projectedViewVisual;

	void publishAll ();

	void queueThread ();

protected:
	virtual void UpdateChild ();

public:
	VisibilityGrid ():
		ModelPlugin ()
	{}

	void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);

	void updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView);
	void odometryCallback (const nav_msgs::OdometryConstPtr &odom);
};

}
#endif // VISIBILITY_GRID_H
