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
	model = _model;

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
	SubscribeOptions so =
			SubscribeOptions::create<opt_view::ProjectedView> ("update_visibility_matrix", 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);

	rosSub = rosNode->subscribe (so);
	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView)
{
	opt_view::ProjectedView fakeView;
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();
	fakeView.points.resize (POINTS_NO);


	fakeView.points[0].x = 0;
	fakeView.points[0].y = 0;
	fakeView.points[0].z = 0;
	fakeView.points[1].x = 1;
	fakeView.points[1].y = 0;
	fakeView.points[1].z = 0;
	fakeView.points[2].x = 1;
	fakeView.points[2].y = 1;
	fakeView.points[2].z = 0;
	fakeView.points[3].x = 0;
	fakeView.points[3].y = 1;
	fakeView.points[3].z = 0;

	projectedViewVisual->updatePoints (fakeView);
	projectedViewVisual->updatePose (Pose3d (t*0.1, 0,0,0,0,0));

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









































