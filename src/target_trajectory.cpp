#include "target_trajectory.h"

#include <std_msgs/Empty.h>

using namespace ros;
using namespace std;
using namespace gazebo;
using namespace physics;
using namespace ignition::math;

void TargetTrajectory::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	string linkName;
	model = _model;

	if (!_sdf->HasElement ("link")) {
		gzerr << "Missing <link> element.";
		return;
	}

	linkName = _sdf->GetElement ("link")->Get<string> ();
	link  = boost::dynamic_pointer_cast<Link> (_model->GetByName (linkName));

	if (_sdf->HasElement ("speed"))
		params.speed = _sdf->GetElement ("speed")->Get<double> ();
	if (_sdf->HasElement ("amplitude"))
		params.amplitude = _sdf->GetElement ("amplitude")->Get<double> ();
	if (_sdf->HasElement ("frequency"))
		params.frequency = _sdf->GetElement ("frequency")->Get<double> ();
	if (_sdf->HasElement ("delay"))
		params.delay = _sdf->GetElement ("delay")->Get<double> ();
	if (_sdf->HasElement ("initial_position")) {
		sdf::ElementPtr ip = _sdf->GetElement ("initial_position");
		double x(0), y(0), z(0);

		if (ip->HasAttribute ("x"))
			ip->GetAttribute ("x")->Get<double> (x);
		if (ip->HasAttribute ("y"))
			ip->GetAttribute ("y")->Get<double> (y);
		if (ip->HasAttribute ("z"))
			ip->GetAttribute ("z")->Get<double> (z);

		params.initialPosition.Set (x,y,z);
	}

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  std::bind(&TargetTrajectory::OnUpdate, this));
}

void TargetTrajectory::OnUpdate ()
{
	Pose3d currPose;
	Quaterniond currOrientation = Quaterniond::Identity;
	Vector3d currPosition;
	common::Time time = model->GetWorld ()->SimTime ();
	double t = time.Double ();

	if (t < params.delay)
		return;

	t -= params.delay;

	currPosition.Set (
				params.speed*t,
				params.amplitude * sin (params.frequency * 2 * M_PI * t),
				0);

	currPose.Set (params.initialPosition + currPosition, currOrientation);

	model->SetRelativePose (currPose);
}



GZ_REGISTER_MODEL_PLUGIN(TargetTrajectory)
