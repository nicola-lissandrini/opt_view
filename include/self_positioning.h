#ifndef SELF_POSITIONING_H
#define SELF_POSITIONING_H

#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <opt_view/Formation.h>
#include <opt_view/MultiagentPose.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen_conversions/eigen_msg.h>

#define NODE_NAME "self_positioning"
#include "pd_rosnode.h"
#include "common.h"

typedef Eigen::Isometry3d Pose;
typedef std::vector<Pose> Formation;
typedef Eigen::Vector3d Force;
typedef std::vector<Eigen::Vector3d> Vectors;

class SelfPositioning
{
	Pose targetPose;
	Formation computedFormation;
	Formation currentFormation;
	Formation formationGeometry;

	int agentsNo;
	bool started;

public:
	SelfPositioning ();

	void setParams (XmlRpc::XmlRpcValue &_params);
	void setTargetPose(const Pose &pose);
	void updateFormation (const Eigen::Isometry3d &transform, int i);
	int setFormationState (const Formation &formation);

	void compute ();
	Formation getComputedFormation ();
};

class SelfPositioningNode : public PdRosNode
{
	ros::Subscriber targetPoseSub;
	ros::Subscriber formationStateSub;
	ros::Subscriber angleSub;
	ros::Publisher goalPub;
	SelfPositioning selfPositioning;

	int actions ();
	void initParams ();
	void initROS ();

	void publishTrajectory ();

	bool started;

public:
	SelfPositioningNode ();

	void targetCallback (const geometry_msgs::Pose &target);
	void formationStateCallback (const opt_view::Formation &newFormation);
	void updateFormation (const opt_view::MultiagentPose &transformation);
};

#endif // SELF_POSITIONING_H
