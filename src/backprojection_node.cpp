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
	cameraOdomSub = nh.subscribe (odomTopic, 1, &BackProjectionNode::cameraOdomCallback, this);
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
	if (!backProjection.isInitialized ())
		return 0;

	opt_view::ProjectedView projView;
	vector<Vector3d> points;

	points = backProjection.backProjectPoints ();
	projView.points.resize (points.size ());

	for (int i = 0; i < points.size (); i++) {
		Vector3d curr = points[i];

		projView.points[i].x = curr(0);
		projView.points[i].y = curr(1);
		projView.points[i].z = curr(2);
	}


	projectedPointsPub.publish (projView);

	return 0;
}

BackProjection::BackProjection():
	initialized(false),
	pointsToProject(POINTS_NO)
{
	Matrix3d toCameraMx;

	toCameraMx << 0, 0, 1, 0, -1, 0, 1, 0, 0;
	toCamera = toCameraMx;
}

ProjectionMatrix BackProjection::getExtrinsicMatrix () {
	return intrinsicMatrix * (pose * toCamera).matrix ().inverse ();
}

Vector3d BackProjection::backProject (Vector2d imagePoint)
{
	Vector4d genericHomogeneous;
	Vector3d generic, director;
	Vector3d cameraOrigin = pose.translation ();
	Vector3d projected;
	Vector3d homogeneousImagePoint = Vector3d (imagePoint(0), imagePoint(1), 1);
	ProjectionMatrix extrinsicMatrix = getExtrinsicMatrix ();
	double lambda;

	genericHomogeneous = extrinsicMatrix.fullPivHouseholderQr().solve (homogeneousImagePoint);
	cout << "Originale\n"<< homogeneousImagePoint << "\nRicostruito\n" << extrinsicMatrix * genericHomogeneous << endl;
	generic = genericHomogeneous.head<3> () * 1 / genericHomogeneous(3);

	cout << generic << endl;

	director = generic - cameraOrigin;
	director = director * (1 / director.norm ());

	cout << director << endl;

	lambda = - cameraOrigin(2) / director(2);

	cout << "lambda\n" << lambda << "\n" << endl;

	projected = cameraOrigin + lambda * director;

	cout << "back\n" << projected << "\n" << endl;

	return projected;
}


std::vector<Vector3d> BackProjection::backProjectPoints()
{
	std::vector<Vector3d> points;

	points.resize (POINTS_NO);
	for (int i = 0; i < POINTS_NO; i++){
		points[i] = backProject (pointsToProject[i]);
	}

	return points;
}

void BackProjection::updateParams (const sensor_msgs::CameraInfo &cameraInfo)
{
	initialized = true;

	for (int i = 0; i < intrinsicMatrix.rows (); i++){
		for (int j = 0; j < intrinsicMatrix.cols () ; j++) {
			intrinsicMatrix(i,j) = cameraInfo.P[i * intrinsicMatrix.cols () + j];
		}
		// Ignore garbage in the 4-th column
		//intrinsicMatrix(i, 3) = 0;
	}

	//intrinsicMatrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
	pxWidth =cameraInfo.width;
	pxHeight = cameraInfo.height;

	cout << pxWidth << " " << pxHeight << endl;
	updatePointsToProject (pxWidth, pxHeight);
}

void BackProjection::updatePointsToProject (double width, double height)
{
	pointsToProject[0] = Vector2d (0, 0);
	pointsToProject[1] = Vector2d (width, 0);
	pointsToProject[2] = Vector2d (width, height);
	pointsToProject[3] = Vector2d (0, height);
	pointsToProject[4] = Vector2d (width/2, height/2);
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	BackProjectionNode bpn;

	return bpn.spin ();
}

























