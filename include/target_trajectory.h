#ifndef TARGET_TRAJECTORY_H
#define TARGET_TRAJECTORY_H

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>
#include <Eigen/Dense>

#define NODE_NAME "target_trajectory"
#include "pd_rosnode.h"

namespace gazebo {

class Trajectory
{
	std::vector<ignition::math::Vector2d> points;
	double sampleTime;

public:
	Trajectory ():
		sampleTime(1)
	{}

	void poulateFromCsv (const std::string &filename);
	void setSampleTime (double _sampleTime);
	ignition::math::Vector2d get (double t);
	double maxT ();
};

class TargetTrajectory : public ModelPlugin
{
	physics::ModelPtr model;
	ros::CallbackQueue rosQueue;
	physics::LinkPtr link;
	event::ConnectionPtr updateConnection;

	Trajectory traj;

public:
	TargetTrajectory ():
		ModelPlugin ()
	{}

	void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void OnUpdate ();

private:
	void queueThread ();
};


}

#endif // TARGET_TRAJECTORY_H
