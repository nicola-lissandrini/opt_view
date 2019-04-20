#ifndef VISIBILITY_GRID_H
#define VISIBILITY_GRID_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>

namespace gazebo {


class CellVisual
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	msgs::Visual plane, edge1, edge2;
	std::string parentName;

	double cellSize;
	double edgeSize;
	std::string id;
	void setId (int _id);

	void buildPlane (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);
	void buildEdge1 (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);
	void buildEdge2 (const ignition::math::Color &ambientColor, const ignition::math::Color &emitColor);

public:
	CellVisual (const std::string &parentName, transport::NodePtr _node, transport::PublisherPtr _visPub);

	void build (int _id, double _cellSize, double _edgeSize);
	void updatePose (const ignition::math::Pose3d &pose);
	void redraw ();
};

class CellMatrix
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	std::vector<CellVisual*> cells;
	std::string parentName;
	ignition::math::Pose3d basePose;
	int rows, cols;
	double xM, yM, cellSize;

	ignition::math::Vector3d cellMap (int i, int j);

public:
	CellMatrix (int _rows, int _cols, double _xM, double _yM,
				const std::string &_parentName, transport::NodePtr _node, transport::PublisherPtr _visPub):
		parentName(_parentName),
		node(_node),
		visPub(_visPub),
		cells(_rows*_cols),
		rows(_rows),
		cols(_cols),
		xM(_xM), yM(_yM)
	{}

	void build (double cellSize);
	void updatePose(const ignition::math::Pose3d &pose);

	CellVisual &operator() (int i, int j) {
		assert (cols*i + j < cells.size ());
		return *cells[cols*i + j];
	}
};

class VisibilityGrid : public ModelPlugin
{
	transport::NodePtr node;
	transport::PublisherPtr visPub;
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber rosSub;
	std::thread rosQueueThread;
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;

	CellMatrix *cellMatrix;

	void publishAll ();

	void queueThread ();

protected:
	virtual void UpdateChild ();

public:
	VisibilityGrid ():
		ModelPlugin ()
	{}

	void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);

	void updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView);
};

}
#endif // VISIBILITY_GRID_H
