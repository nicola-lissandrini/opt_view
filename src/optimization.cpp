#include "optimization.h"

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace Eigen;

int Optimization::initParams (XmlRpcValue &rpcParams)
{
	string typeStr = paramString (rpcParams["optimization_type"]);

	if (typeStr == "CCW")
		params.type = OPTIMIZATION_CCW;
	else {
		if (typeStr == "CW") {
			params.type = OPTIMIZATION_CW;
		} else {
			return -1;
		}
	}

	params.angleStep = paramDouble (rpcParams["angle_step"]);

	return 0;
}

void Optimization::setPose (const Isometry3d &cameraPose3D)
{
	Vector3d pos3D = cameraPose3D.translation ();
	Quaterniond rot3D = cameraPose3D.rotation ();


}
