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
#include <semaphore.h>
#include <errno.h>

namespace gazebo {

class VisibilityGrid : public VisualPlugin
{

	ros::NodeHandle *rosNode;
	ros::Subscriber odomSub;
	ros::Subscriber projViewSub;
	ros::CallbackQueue rosQueue;
	event::ConnectionPtr updateConnection;

	std::thread rosQueueThread;
	rendering::VisualPtr visualRoot;
	rendering::VisualPtr visualNew;
	uint visualOldId;
	rendering::ScenePtr scene;

	ignition::math::Pose3d cameraPose;
	std::vector<ignition::math::Vector3d> projPoints;

	std::string id;
	std::string projectedViewTopic, odometryTopic;
	bool first, newVisualAvailable;
	int unique;
	int count;

	void initROS ();
	inline std::string getVisualName ();

	void redraw ();
	msgs::Visual visualMsgFromPoints (const std::vector<ignition::math::Vector3d> &points);
	std::vector<ignition::math::Vector3d> convertPoints (const std::vector<geometry_msgs::Point> &points);
	bool processSDF(sdf::ElementPtr element);
	void UpdateChild ();

	void queueThread ();

	// Gazebo bug related
	sem_t *sem;

public:
	VisibilityGrid ():
		VisualPlugin (),
		first(true), count(0),
		newVisualAvailable(false)
	{}

	void Load (rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

	void updateMatrix (const opt_view::ProjectedViewConstPtr &newProjView);
	void odometryCallback (const nav_msgs::OdometryConstPtr &odom);
};

}
#endif // VISIBILITY_GRID_H
