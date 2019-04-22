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
	string cameraTopic = paramString (params["topic_pre"])
			+ paramString (params["agent_id"])
			+ paramString (params["camera_info_post"]);
	string projectedTopic =  paramString (params["topic_pre"])
			+ paramString (params["agent_id"])
			+ paramString (params["projected_view_post"]);


	cameraInfoSub = nh.subscribe (cameraTopic, 1, &BackProjectionNode::cameraInfoCallback, this);
	projectedPointsPub = nh.advertise<opt_view::ProjectedView> (projectedTopic, 1);
}

void BackProjectionNode::cameraInfoCallback (const sensor_msgs::CameraInfo &cameraInfo)
{
}

int BackProjectionNode::actions ()
{
	opt_view::ProjectedView projView;

	temp.points.resize (4);

	temp.points[0].x = 0;
	temp.points[0].y = 0;
	temp.points[0].z = -2;
	temp.points[1].x = 1;
	temp.points[1].y = 0;
	temp.points[1].z = -2;
	temp.points[2].x = 1;
	temp.points[2].y = 1;
	temp.points[2].z = -2;
	temp.points[3].x = 0;
	temp.points[3].y = 1;
	temp.points[3].z = -2;

	projectedPointsPub.publish (temp);

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);
	BackProjectionNode bpn;

	return bpn.spin ();
}
