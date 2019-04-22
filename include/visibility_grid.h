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
	ros::Subscriber odomSub;
	ros::Subscriber projViewSub;
	ros::CallbackQueue rosQueue;

	std::thread rosQueueThread;
	rendering::VisualPtr visualRoot, visualOld;
	rendering::ScenePtr scene;

	ignition::math::Pose3d cameraPose;
	opt_view::ProjectedView projView;

	std::string id;
	std::string projectedViewTopic, odometryTopic;
	bool first;

	void initROS ();
	void initVisual (rendering::VisualPtr &visual);
	void addVisual (rendering::VisualPtr visual);
	void removeVisual (rendering::VisualPtr visual);
	inline std::string getVisualName () {
		return "visual_proj_" + id;
	}

	void redraw ();
	msgs::Visual *visualMsgFromPoints (const std::vector<geometry_msgs::Point> &points);
	bool processSDF(sdf::ElementPtr element);

	void queueThread ();

public:
	VisibilityGrid ():
		VisualPlugin (),
		first(true)
	{}

	void Load (rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

	void updateMatrix (const opt_view::ProjectedViewConstPtr &newProjView);
	void odometryCallback (const nav_msgs::OdometryConstPtr &odom);
};

}
#endif // VISIBILITY_GRID_H
