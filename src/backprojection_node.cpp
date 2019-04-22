#include "backprojection.h"

using namespace ros;
using namespace XmlRpc;
using namespace std;
using namespace Eigen;

BackProjectionNode::BackProjectionNode():
	PdRosNode(NODE_NAME)
{
	initParams ();
	initROS ();
}

void BackProjectionNode::initParams ()
{
}

void BackProjectionNode::initROS ()
{
	string cameraTopic = paramString (params["camera_info_topic"]);
	string odomTopic = paramString (params["camera_odom_topic"]);
	string projectedTopic =  paramString (params["projected_view_topic"]);


	cameraInfoSub = nh.subscribe (cameraTopic, 1, &BackProjectionNode::cameraInfoCallback, this);
	cameraInfoSub = nh.subscribe (odomTopic, 1, &BackProjectionNode::cameraOdomCallback, this);
	projectedPointsPub = nh.advertise<opt_view::ProjectedView> (projectedTopic, 1);
}

void BackProjection::updatePose(const Isometry3d &newPose) {
	pose = newPose;
}

void BackProjectionNode::cameraInfoCallback (const sensor_msgs::CameraInfo &cameraInfo) {
	backProjection.updateParams (cameraInfo);
}

void BackProjectionNode::cameraOdomCallback (const nav_msgs::Odometry &odom) {
	Isometry3d eigenPose;

	tf::poseMsgToEigen (odom.pose.pose, eigenPose);
	backProjection.updatePose (eigenPose);
}

int BackProjectionNode::actions ()
{
	opt_view::ProjectedView projView;
	vector<Vector3d> points;
	projView.points.resize (points.size ());

	points = backProjection.backProjectPoints ();

	for (int i = 0; i < points.size (); i++) {
		projView.points[i].x = points[i](0);
		projView.points[i].y = points[i](1);
		projView.points[i].z = points[i](2);
	}

	projectedPointsPub.publish (projView);

	return 0;
}

Vector3d BackProjection::backProject (Vector2d imagePoint)
{
	Vector3d generic, director;
	Vector3d homogeneousImagePoint = Vector3d (imagePoint(0), imagePoint(1), 1);

	generic = pinv (projectionMatrix) * homogeneousImagePoint;
	director = generic * 1 / generic.norm ();
}


std::vector<Vector3d> BackProjection::backProjectPoints()
{
	std::vector<Vector3d> points;

	points.resize (POINTS_NO);
	for (int i = 0; i < POINTS_NO; i++)
		points[i] = backProject (pointsToProject[i]);

	return points;
}

void BackProjection::updateParams (const sensor_msgs::CameraInfo &cameraInfo)
{
	initialized = true;
	for (int i = 0; i < projectionMatrix.rows (); i++)
		for (int j = 0; j < projectionMatrix.cols (); i++)
			projectionMatrix(i,j) = cameraInfo.P[i * projectionMatrix.cols () + j];
	pxWidth = cameraInfo.width;
	pxHeight = cameraInfo.height;

	updatePointsToProject (pxWidth, pxHeight);
}

void BackProjection::updatePointsToProject (double width, double height)
{
	pointsToProject[0] = Vector2d (0, 0);
	pointsToProject[1] = Vector2d (width, 0);
	pointsToProject[2] = Vector2d (0, height);
	pointsToProject[3] = Vector2d (width, height);
	pointsToProject[4] = Vector2d (width/2, height/2);
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	BackProjectionNode bpn;

	return bpn.spin ();
}

























