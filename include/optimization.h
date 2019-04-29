#ifndef OPTIMIZATION_NODE_H
#define OPTIMIZATION_NODE_H

#define NODE_NAME "optimization"
#include "pd_rosnode.h"
#include "visibility_matrix_builder.h"

#include <std_msgs/Int64.h>
#include <opt_view/SparseMatrixInt.h>
#include <opt_view/MultiagentPose.h>
#include <nav_msgs/Odometry.h>
#include <map>


struct OptimizationResults {
	int imbalanceFactor;
	double optimizationStep;
};

class Optimization
{
	Eigen::Isometry2d cameraPose;
	VisibilityMatrix selfVisibility, neighborVisibility;
	VisibilityMatrix totalVisibility, overlappingVisibility;
	double lossProbability;

	ReadyFlags<std::string> flags;
	struct Params {
		int optimizationSign;
		double angleStep;
		double gain;
		double probabilityGain;
	} params;

	OptimizationResults results;

	void setFromMsg (VisibilityMatrix &visibility, const opt_view::SparseMatrixInt &matrixMsg);

	int getSpacialValue (int i, int j);
	void computeTotalVisibility ();
	double computeImbalanceFactor();
	void computeOverlapping ();
	double computeStep ();
	double getCostValue(int i, int j);
	double getProbabilityValue();

public:
	Optimization ();

	int initParams(XmlRpc::XmlRpcValue &rpcParams);
	void setPose (const Eigen::Isometry3d &cameraPose3D);
	void setSelfFromMsg (const opt_view::SparseMatrixInt &matrixMsg);
	void setNeighborFromMsg (const opt_view::SparseMatrixInt &matrixMsg);
	void setLossProbability (double newLossProbability);
	int optimize ();

	bool isReady () const {
		return flags.isReady ();
	}
	OptimizationResults getResults () const {
		return results;
	}
	const VisibilityMatrix &getSelfVisibility () const {
		return selfVisibility;
	}
	const VisibilityMatrix &getNeighborVisibility () const {
		return neighborVisibility;
	}
	const VisibilityMatrix &getTotalVisibility () const {
		return totalVisibility;
	}
	const VisibilityMatrix &getOverlappingVisibility () const {
		return overlappingVisibility;
	}
};

class OptimizationNode : public PdRosNode
{
	ros::Subscriber selfMatrixSub;
	ros::Subscriber neighborMatrixSub;
	ros::Subscriber cameraOdomSub;
	ros::Subscriber lossProbabilitySub;
	ros::Publisher rotationStepPub;
	ros::Publisher rvizPub;
	ros::Publisher imbalancePub;
	Optimization optimization;
	int agentId, neighborId;
	bool doPublishRviz;

	int actions ();
	void initParams ();
	void initROS ();
	void publishRviz (const VisibilityMatrix &matrix);
	void publishImbalanceFactor();
	void publishOptimization ();

public:
	OptimizationNode ();

	void selfMatrixCallback (const opt_view::SparseMatrixInt &matrix);
	void neighborMatrixCallback (const opt_view::SparseMatrixInt &matrix);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
	void lossProbabilityCallback (const std_msgs::Float64 &lossMsg);
};

#endif // OPTIMIZATION_NODE_H
