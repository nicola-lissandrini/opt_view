#include "pd_rosnode.h"

#include <Eigen/Geometry>

using namespace ros;
using namespace std;
using namespace tf;
using namespace XmlRpc;

double paramDouble (XmlRpcValue &param)
{
	switch (param.getType ())
	{
	case XmlRpcValue::TypeInt:
		return (double) int (param);
	case XmlRpcValue::TypeDouble:
		return double (param);
	default:
		ROS_ERROR ("Invaild datatype %d, expecting %d",param.getType (), XmlRpcValue::TypeDouble);
		throw XmlRpcException ("Invalid datatype, expecting double");
	}
}

string paramString (XmlRpcValue &param)
{
	if (param.getType () != XmlRpcValue::TypeString)
		throw XmlRpcException ("Invalid datatype, expecting string");
	return string (param);
}

Pose paramPose (XmlRpcValue &param)
{
	Pose ret;

	ret.setOrigin (Vector3 (
					paramDouble (param["pos"]["x"]),
					paramDouble (param["pos"]["y"]),
					paramDouble (param["pos"]["z"])));
	ret.setRotation (Quaternion (
					paramDouble (param["rot"]["x"]),
					paramDouble (param["rot"]["y"]),
					paramDouble (param["rot"]["z"]),
					paramDouble (param["rot"]["w"])));
	return ret;
}

Eigen::Isometry3d paramPoseEigen(XmlRpcValue &param)
{
	Eigen::Translation3d tr(
					paramDouble (param["pos"]["x"]),
					paramDouble (param["pos"]["y"]),
					paramDouble (param["pos"]["z"]));
	Eigen::Quaterniond rot =  Eigen::Quaterniond (
					paramDouble (param["rot"]["w"]),
					paramDouble (param["rot"]["x"]),
					paramDouble (param["rot"]["y"]),
					paramDouble (param["rot"]["z"]));
	return tr * rot;
}

PdRosNode::PdRosNode (string _name):
	name(_name)
{
	initParams ();
	initROS ();
}

void PdRosNode::initParams ()
{
	try {
		nh.getParam (name, params);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s",e.getMessage ().c_str ());
	}
}

void PdRosNode::initROS ()
{
	rate = new Rate (paramDouble (params["rate"]));
}

int PdRosNode::spin ()
{
	int ret;

	while (ok ()) {
		ret = actions ();

		if (ret)
			return ret;

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}






































