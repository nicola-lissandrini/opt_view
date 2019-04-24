#include "visibility_grid.h"

#include <tf/tf.h>
#include <gazebo/transport/transport.hh>
#include "pd_rosnode.h"
#include <Ogre.h>

#include <fcntl.h>       /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */



using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;
using namespace ros;

#define POINTS_NO 4

void VisibilityGrid::Load (VisualPtr _visual, sdf::ElementPtr _sdf)
{
	visualRoot = _visual;
	scene = _visual->GetScene ();
	unique = ::getppid ();

	/*// Solve Gazebo bug
	sem = sem_open ("visiblity_plugin_mutex", O_CREAT|O_EXCL, 0, 1);

	if (sem == SEM_FAILED) {
		switch (errno)
		{
		case EEXIST:
			ROS_INFO ("EEXIST");
			break;
		case EACCES:
			ROS_INFO ("EACCES");
			break;
		case EINVAL:
			ROS_INFO ("EINVAL");
			break;
		}
		ROS_INFO ("IO %d SONO IL SECONDO. ", unique);
	} else {
		ROS_INFO ("IO %d SONO IL PRIMO\n\n\n\nUSCITO\n\n\n", unique);
		sleep (1);
		sem_unlink ("visiblity_plugin_mutex");
		return;
	}
*/
	if (!processSDF (_sdf))
		return;

	initROS ();

}

bool VisibilityGrid::processSDF (sdf::ElementPtr element)
{
	id = element->GetElement ("id")->Get<string> ();

	if (!element->HasElement ("camera_odom_topic")) {
		gzerr << "visibility_grid_plugin: camera_odom_topic must be set" << endl;
		return false;
	}
	if (!element->HasElement ("projected_view_topic")) {
		gzerr << "visibility_grid_plugin: projected_view_topic must be set" << endl;
		return false;
	}

	odometryTopic = element->GetElement ("camera_odom_topic")->Get<string> ();
	projectedViewTopic = element->GetElement ("projected_view_topic")->Get<string> ();

	return true;
}

void VisibilityGrid::initROS ()
{
	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions projViewSubSo =
			SubscribeOptions::create<opt_view::ProjectedView> (projectedViewTopic, 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);
	SubscribeOptions odomSubSo =
			SubscribeOptions::create<nav_msgs::Odometry> (odometryTopic, 1,
																  boost::bind(&VisibilityGrid::odometryCallback, this, _1),
																  ros::VoidPtr(), &rosQueue);

	odomSub = rosNode->subscribe (odomSubSo);
	projViewSub = rosNode->subscribe (projViewSubSo);

	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));
	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::UpdateChild ()
{
	if (!newVisualAvailable)
		return;

	newVisualAvailable = false;
	redraw ();
}

void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &newProjView)
{
	projPoints = convertPoints (newProjView->points);

	newVisualAvailable = true;
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

	cameraPose = newPose;
}

vector<Vector3d> VisibilityGrid::convertPoints (const std::vector<geometry_msgs::Point> &points)
{
	vector<Vector3d> converted;
	Pose3d cameraToWorld = cameraPose.Inverse ();
	Vector3d cameraFramePoint, worldFramePoint;

	converted.resize (points.size());

	for (int i = 0; i < POINTS_NO; i++) {
		cameraFramePoint = Vector3d (points[i].x,
									 points[i].y,
									 points[i].z);
		// No conversion
		converted[i] = cameraFramePoint;
	}
	return converted;
}

string VisibilityGrid::getVisualName() {
	return "visual_proj_" + id + "_" +std::to_string (count) + std::to_string (unique);
}

void VisibilityGrid::redraw ()
{
	msgs::Visual visualMsg = visualMsgFromPoints (projPoints);
	string visualName = getVisualName ();

	if (!first) {
		scene->RemoveVisual (visualOldId);

	} else {
		first = false;
	}
	visualNew.reset (new Visual (visualName, scene));

	auto msgPtr = new ConstVisualPtr(&visualMsg);

	visualNew->LoadFromMsg (*msgPtr);
	visualNew->SetVisible (true);
	scene->AddVisual (visualNew);

	visualOldId = visualNew->GetId ();
	count++;
}

msgs::Visual VisibilityGrid::visualMsgFromPoints (const std::vector<Vector3d> &points)
{
	common::Time tt = scene->SimTime ();
	double t = tt.Double ();
	msgs::Visual visualMsg;
	msgs::Polyline *poly;
	Vector3d point;

	visualMsg.set_name (getVisualName ());
	visualMsg.set_parent_name (scene->Name ());


	visualMsg.mutable_geometry ()->set_type (msgs::Geometry::POLYLINE);
	poly = visualMsg.mutable_geometry ()->add_polyline ();

	/*msgs::Set (poly->add_point (), Vector2d (0,0));
	msgs::Set (poly->add_point (), Vector2d (0,0.1*t));
	msgs::Set (poly->add_point (), Vector2d (1,1));
	poly->set_height (1);
	msgs::Set (visualMsg.mutable_pose (), Pose3d (0.1*t, 0,0, 0,0,0));*/

	for (int i = 0; i < POINTS_NO; i++) {
		point = points[i];

		msgs::Set (poly->add_point (), Vector2d (point.X (), point.Y ()));
	}
	poly->set_height (0.001);
	msgs::Set (visualMsg.mutable_pose (), Pose3d (0, 0,0, 0,0,0));

	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	msgs::Set (visualMsg.mutable_material ()->mutable_ambient (), colorInner);
	msgs::Set (visualMsg.mutable_material ()->mutable_diffuse (), colorInner);
	msgs::Set (visualMsg.mutable_material ()->mutable_emissive (), colorZero);

	return visualMsg;
}

/*
void VisibilityGrid::initVisual ()
{
	VisualPtr nuova, nuova2;
	nuova.reset (new Visual ("visual_bella", visual->GetParent ()));
	nuova2.reset (new Visual ("visual_bella2", visual->GetParent ()));
	msgs::Visual visualMsg, visualMsg2;

	visualMsg.set_name ("visualCiao");sleep (1);
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
	c++;
}*/



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









































