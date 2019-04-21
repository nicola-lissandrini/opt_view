#include "visibility_grid.h"

#include <tf/tf.h>

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;
using namespace ros;

#define POINTS_NO 4



void VisibilityGrid::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	string matrixTopic("update"), odometryTopic;
	model = _model;	

	if (_sdf->HasElement ("matrix_topic"))
		matrixTopic = _sdf->GetElement ("matrix_topic")->Get<string> ();
	if (!_sdf->HasElement ("camera_odom_topic")) {
		gzerr << "VisiblityGrid: must specify camera_odom_topic" << endl;
		return;
	}
	odometryTopic = _sdf->GetElement ("camera_odom_topic")->Get<string> ();

	node = transport::NodePtr (new transport::Node());
	node->Init ();
	visPub = node->Advertise<msgs::Visual> ("~/visual",1000);

	projectedViewVisual = new ProjectedViewVisual (_model->GetScopedName (),
												   node, visPub);


	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions matrixSubSo =
			SubscribeOptions::create<opt_view::ProjectedView> (matrixTopic, 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);
	SubscribeOptions poseSubSo =
			SubscribeOptions::create<nav_msgs::Odometry> (odometryTopic, 1,
																  boost::bind(&VisibilityGrid::odometryCallback, this, _1),
																  ros::VoidPtr(), &rosQueue);

	matrixSub = rosNode->subscribe (matrixSubSo);
	poseSub = rosNode->subscribe (poseSubSo);
	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::odometryCallback (const nav_msgs::OdometryConstPtr &odom)
{
	Pose3d pose;

	pose.Set (Vector3d (odom->pose.pose.position.x,
						odom->pose.pose.position.y,
						odom->pose.pose.position.z),
			  Quaterniond (odom->pose.pose.orientation.w,
						   odom->pose.pose.orientation.x,
						   odom->pose.pose.orientation.y,
						   odom->pose.pose.orientation.z));
	projectedViewVisual->updatePose (pose);
}

void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView)
{
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();

	projectedViewVisual->updatePoints (*projectedView);

	projectedViewVisual->redraw ();
}


void VisibilityGrid::UpdateChild ()
{
}

void VisibilityGrid::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

void ProjectedViewVisual::build ()
{
	visualMsg.set_parent_name (parentName);
	visualMsg.set_name ((parentName + "/visual").c_str ());

	msgs::Geometry *viewGeom = visualMsg.mutable_geometry ();
	viewGeom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly = viewGeom->add_polyline ();

	for (int i = 0; i < POINTS_NO; i++)
		msgs::Set (poly->add_point (), Vector2d ());
	poly->set_height (0.001);

	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	msgs::Set (visualMsg.mutable_material ()->mutable_ambient (), colorInner);
	msgs::Set (visualMsg.mutable_material ()->mutable_diffuse (), colorInner);
	msgs::Set (visualMsg.mutable_material ()->mutable_emissive (), colorZero);
}

void ProjectedViewVisual::updatePoints (const opt_view::ProjectedView &view) {
	viewPoints = view;
}

void ProjectedViewVisual::updatePose (const Pose3d &newPose) {
	cameraFrame = newPose;
}

void ProjectedViewVisual::redraw ()
{
	Vector3d pointCameraFrame;

	for (int i = 0; i < POINTS_NO; i++) {
		msgs::Polyline *poly = visualMsg.mutable_geometry ()->mutable_polyline (0);
		pointCameraFrame = Vector3d (viewPoints.points[i].x, viewPoints.points[i].y,0);

		msgs::Set (poly->mutable_point (i), Vector2d (pointCameraFrame.X (), pointCameraFrame.Y ()));
		msgs::Set (visualMsg.mutable_pose (), cameraFrame);
	}

	visPub->Publish (visualMsg);
}

GZ_REGISTER_MODEL_PLUGIN(VisibilityGrid)









































