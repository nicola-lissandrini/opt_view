#ifndef LOSS_PROBABILITY_NODE_H
#define LOSS_PROBABILITY_NODE_H

#define NODE_NAME "loss_probability"
#include "pd_rosnode.h"
#include "visibility_matrix_builder.h"
#include "kalman_filter.h"
#include <opt_view/SparseMatrixInt.h>

#define MODEL_STATES_N 4
#define MODEL_INPUTS_N 4
#define MODEL_OUTPUTS_N 2

class LossProbability
{
	ReadyFlags<int> flags;
	KalmanModel model;
	KalmanFilter kalman;
	std::vector<VisibilityMatrix> visibilityMatrices;
	VisibilityMatrix totalView, debugView;
	Eigen::Vector2d poseGroundTruth;
	Eigen::Vector2d velGroundTruth;
	int agentsNo;
	double predictionInterval;

	KalmanModel buildModel (XmlRpc::XmlRpcValue &params);
	void sumVisibilityMatrices (VisibilityMatrix &totalView);
	double computeProbability (const VisibilityMatrix &totalView, const Measure &prediction);

public:
	LossProbability ();

	void initParams (XmlRpc::XmlRpcValue &params);
	void updateVisibility (const opt_view::SparseMatrixInt &matrixMsg);
	int getAgentsNo ();
	double compute();
	
	bool isReady ();
	void updateTargetPose(const Eigen::Isometry3d &newPose, const Eigen::Vector2d &vel);
	void updateLeaderPose(const Eigen::Isometry3d &newPose);
	Measure getFiltered () const {
		return kalman.getFiltered ();
	}
	const VisibilityMatrix &getTotalView () const {
		return debugView;
	}
};

class LossProbabilityNode: public PdRosNode
{
	std::vector<ros::Subscriber> visibilityMatrixSubs;
	ros::Subscriber targetPoseSub;
	ros::Publisher probabilityPub;
	ros::Publisher matrixPub;

	LossProbability lossProbability;

	int actions ();
	void initParams ();
	void initROS ();

	void publishRviz(const VisibilityMatrix &matrix);

public:
	LossProbabilityNode ();

	void visibilityCallback (const opt_view::SparseMatrixInt &visibility);
	void targetPoseCallback (const nav_msgs::Odometry &newPose);
};

#endif // LOSS_PROBABILITY_NODE_H
