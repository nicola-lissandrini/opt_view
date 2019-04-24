#ifndef VISIBILITY_MATRIX_BUILDER_H
#define VISIBILITY_MATRIX_BUILDER_H

#define NODE_NAME "visibility_matrix_builder"
#include "pd_rosnode.h"
#include <eigen_conversions/eigen_msg.h>
#include <opt_view/ProjectedView.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#define POINTS_NO 5
#define LINES_NO 4
#define CENTER_PT_INDEX (LINES_NO - 1)

typedef Eigen::Vector3d Line;


class VisibilityMatrixBuilder
{
	Eigen::Isometry3d cameraPose;
	std::vector<Eigen::Vector3d> points;

	struct Params {
		double cellSize;
		double xMinRelative;
		double yMinRelative;
		int countH, countK;
	} params;

	void getLines (std::vector<Line> &lines);
	Line getLine (const Eigen::Vector3d &a, const Eigen::Vector3d &b);
	void buildMatrix (Eigen::MatrixXd &visibilityMatrix, const std::vector<Line> &lines);
	bool checkInside (const Eigen::Vector2d &point, const std::vector<Line> &lines);
	int sideSign (const Eigen::Vector2d &point, const Line &line);

public:
	VisibilityMatrixBuilder () {}

	Eigen::Vector2d indicesMap (int h, int k);
	void setPose (const Eigen::Isometry3d &pose);
	bool setPoints (const std::vector<Eigen::Vector3d> &newPoints);
	void setParams (const XmlRpc::XmlRpcValue &params);
	void compute (Eigen::MatrixXd &visibilityMatrix);
};

class VisibilityMatrixBuilderNode : public PdRosNode
{
	ros::Subscriber projViewSub;
	ros::Subscriber cameraOdomSub;
	ros::Publisher matrixPub;
	VisibilityMatrixBuilder builder;

	int actions ();
	void initParams ();
	void initROS ();

public:
	VisibilityMatrixBuilderNode();

	void projViewCallback (const opt_view::ProjectedView &newProjView);
	void cameraOdomCallback (const nav_msgs::Odometry &odom);
};

#endif // VISIBILITY_MATRIX_BUILDER_H
