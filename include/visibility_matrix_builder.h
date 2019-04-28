#ifndef VISIBILITY_MATRIX_BUILDER_H
#define VISIBILITY_MATRIX_BUILDER_H

#include "common.h"

#include <eigen_conversions/eigen_msg.h>
#include <opt_view/ProjectedView.h>
#include <opt_view/SparseMatrixInt.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <boost/mpl/clear.hpp>


#define POINTS_NO 5
#define CORNERS_NO (POINTS_NO - 1)
#define LINES_NO 4
#define CENTER_PT_INDEX (POINTS_NO - 1)

typedef Eigen::Vector3d Line;
typedef Eigen::Triplet<int8_t> Tripleti;
typedef Eigen::SparseMatrix<int8_t> Sparsei;


struct Region
{
	double cellSize;
	Eigen::Vector2d rangeMin;
	Eigen::Vector2d rangeMax;
	Eigen::Isometry2d pose;

	int maxH () const;
	int maxK () const;
	Eigen::Vector2d operator() (int h, int j) const;
	Eigen::Vector2i indicesFromWorld (Eigen::Vector2d pt) const;
	Eigen::Vector2d worldStart() const;
	Eigen::Vector2d worldEnd() const;

	Region () {
		pose = Eigen::Translation2d(0,0) * Eigen::Rotation2Dd(0);
	}
};

class VisibilityMatrix
{
	std::vector<Tripleti> elements;
	Region region;

public:
	VisibilityMatrix ()	{}

	VisibilityMatrix (const Sparsei &sparse);

	void clear ();
	void set (int i, int j, u_int8_t val = 1);
	const Tripleti &getElement (int i) const;
	void toSparse (Sparsei &sparse) const;
	void fromMsg (const opt_view::SparseMatrixInt &matrixMsg);
	void setRegion (const Region &newRegion);
	const Region &getRegion () const {
		return region;
	}
	int rows () const {
		return region.maxH ();
	}
	int cols () const {
		return region.maxK ();
	}
	int count () const {
		return elements.size ();
	}


	VisibilityMatrix operator +(const VisibilityMatrix &other);
};

Region paramRegion (XmlRpc::XmlRpcValue &param);
void visibilityToOccupancyMsg (const VisibilityMatrix &matrix, nav_msgs::OccupancyGrid &rvizMatrix);

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

	bool isReady ();
	bool hasComputed ();


	inline Region getGlobalRegion () {
		return global;
	}
	inline Eigen::Isometry3d getPose () {
		return cameraPose;
	}
};

#endif // VISIBILITY_MATRIX_BUILDER_H
