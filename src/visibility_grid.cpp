#include "visibility_grid.h"

#include <tf/tf.h>
#include <gazebo/transport/transport.hh>
#include "pd_rosnode.h"
#include <Ogre.h>

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;
using namespace ros;

#define POINTS_NO 4

void VisibilityGrid::Load (VisualPtr _visual, sdf::ElementPtr _sdf)
{
	visual = _visual;

	buildVisual ();

	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions matrixSubSo =
			SubscribeOptions::create<opt_view::ProjectedView> ("projectedViewTopic", 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);
	SubscribeOptions poseSubSo =
			SubscribeOptions::create<nav_msgs::Odometry> ("odometryTopic", 1,
																  boost::bind(&VisibilityGrid::odometryCallback, this, _1),
																  ros::VoidPtr(), &rosQueue);

	matrixSub = rosNode->subscribe (matrixSubSo);
	poseSub = rosNode->subscribe (poseSubSo);
	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));

}

void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView)
{}

void VisibilityGrid::buildVisual ()
{
	VisualPtr nuova, nuova2;
	nuova.reset (new Visual ("visual_bella", visual->GetParent ()));
	nuova2.reset (new Visual ("visual_bella2", visual->GetParent ()));
	msgs::Visual visualMsg, visualMsg2;

	visualMsg.set_name ("visualCiao");
	visualMsg.set_parent_name (visual->GetScene ()->Name ());
	visualMsg2.set_name ("visualCiao");
	visualMsg2.set_parent_name (visual->GetScene ()->Name ());

	visualMsg.mutable_geometry ()->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly = visualMsg.mutable_geometry ()->add_polyline ();

	visualMsg2.mutable_geometry ()->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly2 = visualMsg2.mutable_geometry ()->add_polyline ();

	msgs::Set (poly->add_point (), Vector2d (0,0));
	msgs::Set (poly->add_point (), Vector2d (0,1));
	msgs::Set (poly->add_point (), Vector2d (1,1));
	poly->set_height (1);

	msgs::Set (poly2->add_point (), Vector2d (0,0));
	msgs::Set (poly2->add_point (), Vector2d (0,1));
	msgs::Set (poly2->add_point (), Vector2d (-1,-1));
	poly2->set_height (1);

	auto msgPtr = new ConstVisualPtr(&visualMsg);
	auto msgPtr2 = new ConstVisualPtr(&visualMsg2);

	nuova->LoadFromMsg (*msgPtr);
	nuova->SetPose (Pose3d ());
	nuova->SetVisible (true);
	visual->GetScene ()->AddVisual (nuova);

	sleep (5);

	visual->GetScene ()->RemoveVisual (nuova);

	nuova2->LoadFromMsg (*msgPtr2);
	nuova2->SetPose (Pose3d ());
	nuova2->SetVisible (true);
	visual->GetScene ()->AddVisual (nuova2);



	/*

	msgs::Visual *visualMsg = new msgs::Visual;

	visualMsg->set_name (("visualMatrix" + id).c_str ());
	visualMsg->set_parent_name (parentName);

	visualMsg->clear_geometry ();
	visualMsg->mutable_geometry ()->set_type (msgs::Geometry::BOX);
	msgs::Set (visualMsg->mutable_geometry ()->mutable_box ()->mutable_size (), Vector3d (1,1,1));
	visPub->Publish (*visualMsg);

	visualMsg->set_delete_me (true);
	visPub->WaitForConnection ();
	visPub->Publish (*visualMsg);
	visualMsg->set_delete_me (false);

	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	msgs::Set (visualMsg->mutable_material ()->mutable_ambient (), colorInner);
	msgs::Set (visualMsg->mutable_material ()->mutable_diffuse (), colorInner);
	msgs::Set (visualMsg->mutable_material ()->mutable_emissive (), colorZero);

	Vector3d pointCameraFrame, pointWorldFrame;
	msgs::Geometry *geom = visualMsg->mutable_geometry ();
	geom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly = geom->add_polyline ();
	poly->set_height (0.001);

	for (int i = 0; i < POINTS_NO; i++) {
		pointCameraFrame = Vector3d (viewPoints.points[i].x, viewPoints.points[i].y, viewPoints.points[i].z);
		pointWorldFrame = cameraPose.Rot () * pointCameraFrame + cameraPose.Pos ();

		if (abs (pointWorldFrame.Z()) > 0.1) {
	//		ROS_WARN ("Received point in world frame is not zero.");
		}

		msgs::Set (poly->add_point (), Vector2d (pointWorldFrame.X (), pointWorldFrame.Y ()));
		//poly->mutable_point (i)->set_x (pointWorldFrame.X ());
		//poly->mutable_point (i)->set_y (pointWorldFrame.Y ());
	}
	b += 0.01;
	a *= -1;
	if (c > 0) {
		oldMsg->mutable_geometry ()->set_type (msgs::Geometry::BOX);
		oldMsg->set_delete_me (true);
		visPub->WaitForConnection ();
		visPub->Publish (*oldMsg);
		usleep (100000);
		gzerr << "Deleting " << oldMsg->name () << endl;
	}
	visPub->WaitForConnection ();
	visPub->Publish (*visualMsg);
	gzerr << "Creating " << visualMsg->name () << endl;
	if (c > 0) {
		delete oldMsg;
	}
	oldMsg = visualMsg;
	c++;*/
}


void VisibilityGrid::odometryCallback (const nav_msgs::OdometryConstPtr &odom)
{
	Pose3d newPose;

	newPose.Set (Vector3d (odom->pose.pose.position.x,
						odom->pose.pose.position.y,
						odom->pose.pose.position.z),
			  Quaterniond (odom->pose.pose.orientation.w,
						   odom->pose.pose.orientation.x,
						   odom->pose.pose.orientation.y,
						   odom->pose.pose.orientation.z));

	pose = newPose;
}

void VisibilityGrid::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

/*
void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView)
{
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();

	projectedViewVisual->updatePoints (*projectedView);

	projectedViewVisual->redraw ();
}
*/
/*
void VisibilityGrid::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	string projectedViewTopic("update"), odometryTopic, id;
	model = _model;

	Visual vis;



	if (_sdf->HasElement ("projected_view_topic"))
		projectedViewTopic = _sdf->GetElement ("projected_view_topic")->Get<string> ();
	if (!_sdf->HasElement ("camera_odom_topic")) {
		gzerr << "VisiblityGrid: must specify camera_odom_topic" << endl;
		return;
	}
	if (!_sdf->HasElement ("id")) {
		gzerr << "VisiblityGrid: must specify id" << endl;
		return;
	}
	odometryTopic = _sdf->GetElement ("camera_odom_topic")->Get<string> ();
	id = _sdf->GetElement ("id")->Get<string> ();

	node = transport::NodePtr (new transport::Node());
	node->Init ();
	visPub = node->Advertise<msgs::Visual> ("~/visual",1000);
	reqPub = node->Advertise<msgs::Request>("~/request");

	projectedViewVisual = new ProjectedViewVisual (_model->GetScopedName (),id,
												   node, visPub, reqPub);



	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions matrixSubSo =
			SubscribeOptions::create<opt_view::ProjectedView> (projectedViewTopic, 1,
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



void VisibilityGrid::UpdateChild ()
{
}


void ProjectedViewVisual::build ()
{

}

void ProjectedViewVisual::updatePoints (const opt_view::ProjectedView &view) {
	viewPoints = view;
}

void ProjectedViewVisual::updatePose (const Pose3d &newPose)
{
	cameraPose = newPose.Inverse ();
}

void ProjectedViewVisual::redraw ()
{
	msgs::Visual *visualMsg = new msgs::Visual;

	visualMsg->set_name (("visualMatrix" + id).c_str ());
	visualMsg->set_parent_name (parentName);

	visualMsg->clear_geometry ();
	visualMsg->mutable_geometry ()->set_type (msgs::Geometry::BOX);
	msgs::Set (visualMsg->mutable_geometry ()->mutable_box ()->mutable_size (), Vector3d (1,1,1));
	visPub->Publish (*visualMsg);

	visualMsg->set_delete_me (true);
	visPub->WaitForConnection ();
	visPub->Publish (*visualMsg);
	visualMsg->set_delete_me (false);

	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	msgs::Set (visualMsg->mutable_material ()->mutable_ambient (), colorInner);
	msgs::Set (visualMsg->mutable_material ()->mutable_diffuse (), colorInner);
	msgs::Set (visualMsg->mutable_material ()->mutable_emissive (), colorZero);

	Vector3d pointCameraFrame, pointWorldFrame;
	msgs::Geometry *geom = visualMsg->mutable_geometry ();
	geom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly = geom->add_polyline ();
	poly->set_height (0.001);

	for (int i = 0; i < POINTS_NO; i++) {
		pointCameraFrame = Vector3d (viewPoints.points[i].x, viewPoints.points[i].y, viewPoints.points[i].z);
		pointWorldFrame = cameraPose.Rot () * pointCameraFrame + cameraPose.Pos ();

		if (abs (pointWorldFrame.Z()) > 0.1) {
	//		ROS_WARN ("Received point in world frame is not zero.");
		}

		msgs::Set (poly->add_point (), Vector2d (pointWorldFrame.X (), pointWorldFrame.Y ()));
		//poly->mutable_point (i)->set_x (pointWorldFrame.X ());
		//poly->mutable_point (i)->set_y (pointWorldFrame.Y ());
	}
	b += 0.01;
	a *= -1;
	if (c > 0) {
		oldMsg->mutable_geometry ()->set_type (msgs::Geometry::BOX);
		oldMsg->set_delete_me (true);
		visPub->WaitForConnection ();
		visPub->Publish (*oldMsg);
		usleep (100000);
		gzerr << "Deleting " << oldMsg->name () << endl;
	}
	visPub->WaitForConnection ();
	visPub->Publish (*visualMsg);
	gzerr << "Creating " << visualMsg->name () << endl;
	if (c > 0) {
		delete oldMsg;
	}
	oldMsg = visualMsg;
	c++;
}
*/
GZ_REGISTER_VISUAL_PLUGIN(VisibilityGrid)









































