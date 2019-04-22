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
	transport::PublisherPtr visPub, reqPub;
	std::string parentName, id;
	ignition::math::Pose3d cameraPose;
	opt_view::ProjectedView viewPoints;
	msgs::Visual *oldMsg;

	double a,b;
	int c;

	void build ();

public:
	ProjectedViewVisual (const std::string &_parentName, const std::string &_id,
						 transport::NodePtr _node, transport::PublisherPtr _visPub, transport::PublisherPtr _reqPub):
		parentName(_parentName),id(_id),
		node(_node),
		visPub(_visPub), reqPub(_reqPub),
		a(-0.1),b(0),c(0)
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
	transport::PublisherPtr reqPub;

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
