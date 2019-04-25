#ifndef VISIBILITY_MATRIX_BUILDER_H
#define VISIBILITY_MATRIX_BUILDER_H

#define NODE_NAME "visibility_matrix_builder"
#include "pd_rosnode.h"
#include <eigen_conversions/eigen_msg.h>
#include <opt_view/ProjectedView.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#define POINTS_NO 5
#define CORNERS_NO (POINTS_NO - 1)
#define LINES_NO 4
#define CENTER_PT_INDEX (POINTS_NO - 1)

typedef Eigen::Vector3d Line;
typedef Eigen::MatrixXd VisibilityMatrix;

struct Region
{
	double cellSize;
	Eigen::Vector2d rangeMin;
	Eigen::Vector2d rangeMax;
	Eigen::Isometry2d pose;

	int maxH () const;
	int maxK () const;
	Eigen::Vector2d operator() (int h, int j) const;
	Eigen::Vector2i worldIndices (Eigen::Vector2d pt) const;
};

Region paramRegion (XmlRpc::XmlRpcValue &param);

int agentId;

class VisibilityMatrixBuilder
{
	Eigen::Isometry3d cameraPose;
	std::vector<Eigen::Vector3d> points;
	Eigen::Vector2d center;

	Region global;
	Region local;
	bool poseSet, pointsSet, paramsSet, computed;

	void getLines (std::vector<Line> &lines);
	Line getLine (const Eigen::Vector3d &a, const Eigen::Vector3d &b);
	void buildMatrix (VisibilityMatrix &visibilityMatrix, const std::vector<Line> &lines);
	bool checkInside (const Eigen::Vector2d &point, const std::vector<Line> &lines);
	Eigen::Vector2d indicesMap (int h, int k);
	int sideSign (const Eigen::Vector2d &point, const Line &line);
	void resetFlags ();

public:
	VisibilityMatrixBuilder ():
		poseSet(false),
		pointsSet(false),
		paramsSet(false)
	{}

	void setPose (const Eigen::Isometry3d &pose);
	bool setPoints (const std::vector<Eigen::Vector3d> &newPoints);
	void setParams (XmlRpc::XmlRpcValue &rpcParams);
	void compute (VisibilityMatrix &visibilityMatrix);

	inline bool isReady ();
	inline bool hasComputed ();


	inline Region getGlobalRegion () {
		return global;
	}
	inline Eigen::Isometry3d getPose () {
		return cameraPose;
	}
};

class VisibilityMatrixBuilderNode : public PdRosNode
{
	ros::Subscriber projViewSub;
	ros::Subscriber cameraOdomSub;
	ros::Publisher matrixRvizPub;
	VisibilityMatrixBuilder builder;
	VisibilityMatrix visibilityMatrix;
	nav_msgs::MapMetaData mapData;


	int actions ();
	void initParams ();
	void initROS ();
	void publishRviz(const VisibilityMatrix &matrix);

public:
	VisibilityMatrixBuilderNode();

	void projViewCallback (const opt_view::ProjectedView &newProjView);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};

#endif // VISIBILITY_MATRIX_BUILDER_H
