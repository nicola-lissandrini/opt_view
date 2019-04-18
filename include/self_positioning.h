#ifndef SELF_POSITIONING_H
#define SELF_POSITIONING_H

#include <tf/tf.h>
#include <opt_view/Formation.h>
#include <opt_view/MultiagentPose.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#define NODE_NAME "self_positioning"
#include "pd_rosnode.h"

typedef Eigen::Affine3d Pose;
typedef std::vector<Pose> Formation;
typedef Eigen::Vector3d Force;
typedef std::vector<Force> Forces;

class SelfPositioning
{
	Formation targetFormation;
	Formation computedFormation;
	Formation currentFormation;
	Formation oldFormation;

	struct Params {
		int agentsNo;
		double sampleTime;
		double kg, dg, dk;
	} params;

	Force computeGoalForce (int i);
	Force computeRepulsiveForce (int i, int j);
	void computeTrajectory ();
	void initFormation (XmlRpc::XmlRpcValue &_params);

	bool started;

public:
	SelfPositioning ();

	void setParams (XmlRpc::XmlRpcValue &_params);
	int setTargetFormation (const Formation &formation);
	int setFormationState (const Formation &formation);

	void compute ();
	Formation getComputedFormation ();
};

class SelfPositioningNode : public PdRosNode
{
	ros::Subscriber targetFormationSub;
	ros::Subscriber formationStateSub;
	ros::Publisher goalPub;
	SelfPositioning selfPositioning;

	int actions ();
	void initParams ();
	void initROS ();

	void publishTrajectory ();

	bool started;

public:
	SelfPositioningNode ();

	void formationCallback (const opt_view::Formation &newFormation);
	void formationStateCallback (const opt_view::Formation &newFormation);
};

#endif // SELF_POSITIONING_H
