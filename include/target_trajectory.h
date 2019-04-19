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

#define NODE_NAME "target_trajectory"
#include "pd_rosnode.h"

namespace gazebo {


class TargetTrajectory : public ModelPlugin
{
	physics::ModelPtr model;
	ros::CallbackQueue rosQueue;
	physics::LinkPtr link;
	event::ConnectionPtr updateConnection;

	struct Params {
		double speed;
		double amplitude;
		double frequency;
		double delay;
		ignition::math::Vector3d initialPosition;

		Params ():
			speed(1),
			amplitude(1),
			frequency(1),
			delay(0)
		{}
	} params;

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
