#include "target_trajectory.h"

#include <std_msgs/Empty.h>
#include <fstream>

using namespace ros;
using namespace std;
using namespace gazebo;
using namespace physics;
using namespace ignition::math;

void Trajectory::poulateFromCsv (const string &filename)
{
	ifstream csdData;
	string line;
	int rows(0);

	try {
		csdData.open (filename);
	} catch (ios_base::failure &e) {
		throw e.what ();
	}


	while (getline (csdData, line)) {
		stringstream lineStream(line);
		string cell;
		Vector2d v;

		for (int i = 0; i < 2; i++) {
			if (!getline (lineStream, cell, ',')) {
				throw "Invalid input file";
			}
			v[i] = stod (cell);
		}

		points.push_back (v);
	}

}

void Trajectory::setSampleTime(double _sampleTime) {
	sampleTime = _sampleTime;
}

Vector2d Trajectory::get (double t) {
	return points[(int) round (t / sampleTime)];
}

double Trajectory::maxT() {
	return points.size () * sampleTime;
}

void TargetTrajectory::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	string linkName;
	model = _model;

	if (!_sdf->HasElement ("link")) {
		gzerr << "Missing <link> element." << endl;
		return;
	}
	linkName = _sdf->GetElement ("link")->Get<string> ();
	link = boost::dynamic_pointer_cast<Link> (_model->GetByName (linkName));

	if (!_sdf->HasElement ("trajectory")) {
		gzerr << "Missing <trajectory> element." << endl;
		return;
	}
	sdf::ElementPtr trajSdf = _sdf->GetElement ("trajectory");
	if (!trajSdf->HasAttribute ("sample_time")) {
		gzerr << "Sample time not specified. Add sample_time attribute to element <trajectory>" << endl;
		return;
	}
	double sampleTime;
	trajSdf->GetAttribute ("sample_time")->Get<double> (sampleTime);
	traj.setSampleTime (sampleTime);
	if (!trajSdf->HasElement ("filename")) {
		gzerr << "Missing <filename> element." << endl;
		return;
	}
	string filename = trajSdf->GetElement ("filename")->Get<string> ();
	try {
		traj.poulateFromCsv (filename);
	} catch (const string &err) {
		gzerr << err << endl;
		return;
	}

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  std::bind(&TargetTrajectory::OnUpdate, this));
}

void TargetTrajectory::OnUpdate ()
{
	Pose3d newPose;
	Vector2d newPoint;

	common::Time time = model->GetWorld ()->SimTime ();
	double t = time.Double ();

	if (t >= traj.maxT ())
		return;
	newPoint = traj.get (t);

	cout << t << " " << newPoint << endl;
	newPose.Set (Vector3d (newPoint.X (),
						   newPoint.Y (),
						   0),
				 Quaterniond::Identity);
	model->SetRelativePose (newPose);
}



GZ_REGISTER_MODEL_PLUGIN(TargetTrajectory)
