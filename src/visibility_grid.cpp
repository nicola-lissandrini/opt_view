#include "visibility_grid.h"

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;
using namespace ros;

#define CELL_SIZE 0.1
#define EDGE_SIZE 0.0008
#define CSH (cellSize/2)
#define ESH (edgeSize/2)
#define HEIGHT ESH

void VisibilityGrid::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	model = _model;

	node = transport::NodePtr (new transport::Node());
	node->Init ();
	visPub = node->Advertise<msgs::Visual> ("~/visual",1000);

	cellMatrix = new CellMatrix (100, 100, 0, 0,_model->GetScopedName (), node, visPub);
	cellMatrix->build (CELL_SIZE);

	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions so =
			SubscribeOptions::create<std_msgs::Empty> ("update_visibility_matrix", 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);

	rosSub = rosNode->subscribe (so);
	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::updateMatrix (const std_msgs::EmptyConstPtr &empty)
{
	std_msgs::Float64MultiArray visibilityMatrix;

	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();

	cellMatrix->updatePose (Pose3d (t*0.1, 0, 0, 0,0,0));
}


void VisibilityGrid::UpdateChild ()
{

}

void VisibilityGrid::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

void CellVisual::build (int _id, double _cellSize, double _edgeSize)
{
	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	Color colorEdge (0,0,0,0.45);

	cellSize = _cellSize;
	edgeSize = _edgeSize;

	setId (_id);

	buildPlane (colorInner, colorZero);
	buildEdge1 (colorEdge, colorZero);
	buildEdge2 (colorEdge, colorZero);
}

void CellVisual::setId(int _id)
{
	id = to_string (_id);

	plane.set_name (string ("cell" + id + "_plane").c_str());
	edge1.set_name (string ("cell" + id + "_edge1").c_str());
	edge2.set_name (string ("cell" + id + "_edge2").c_str());
}


void CellVisual::buildPlane (const Color &ambientColor, const Color &emitColor)
{
	plane.set_parent_name (parentName);

	msgs::Geometry *geomMsg = plane.mutable_geometry ();
	geomMsg->set_type (msgs::Geometry::PLANE);
	geomMsg->mutable_plane ()->mutable_size ()->set_x (CELL_SIZE);
	geomMsg->mutable_plane ()->mutable_size ()->set_y (CELL_SIZE);
	geomMsg->mutable_plane ()->mutable_normal ()->set_x (0);
	geomMsg->mutable_plane ()->mutable_normal ()->set_y (0);
	geomMsg->mutable_plane ()->mutable_normal ()->set_z (1);

	msgs::Set (plane.mutable_material ()->mutable_ambient (), ambientColor);
	msgs::Set (plane.mutable_material ()->mutable_diffuse (), ambientColor);
	msgs::Set (plane.mutable_material ()->mutable_emissive (), emitColor);

	msgs::Set (plane.mutable_pose (), Pose3d (0,0,HEIGHT, 0,0,0));
}

void CellVisual::buildEdge1 (const Color &ambientColor, const Color &emitColor)
{
	edge1.set_parent_name (parentName);

	msgs::Geometry *edgesGeom = edge1.mutable_geometry ();
	edgesGeom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly  = edgesGeom->add_polyline ();

	msgs::Set (poly->add_point (), Vector2d (-CSH-ESH,-CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH+ESH,-CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH+ESH,+CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH+ESH,+CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH-ESH,+CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH-ESH,-CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (-CSH+ESH,-CSH+ESH));

	poly->set_height (ESH);


	msgs::Set (edge1.mutable_material ()->mutable_ambient (), ambientColor);
	msgs::Set (edge1.mutable_material ()->mutable_diffuse (), ambientColor);
	msgs::Set (edge1.mutable_material ()->mutable_emissive (), emitColor);
}

void CellVisual::buildEdge2 (const Color &ambientColor, const Color &emitColor)
{
	edge2.set_parent_name (parentName);

	msgs::Geometry *edgesGeom = edge2.mutable_geometry ();

	edgesGeom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly = edgesGeom->add_polyline ();

	msgs::Set (poly->add_point (), Vector2d (-CSH-ESH,-CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (-CSH-ESH,+CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH+ESH,+CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (+CSH-ESH,+CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (-CSH+ESH,+CSH-ESH));
	msgs::Set (poly->add_point (), Vector2d (-CSH+ESH,-CSH+ESH));
	msgs::Set (poly->add_point (), Vector2d (-CSH-ESH,-CSH-ESH));

	poly->set_height (ESH);

	msgs::Set (edge2.mutable_material ()->mutable_ambient (), ambientColor);
	msgs::Set (edge2.mutable_material ()->mutable_diffuse (), ambientColor);
	msgs::Set (edge2.mutable_material ()->mutable_emissive (), emitColor);
}

CellVisual::CellVisual(const string &_parentName, transport::NodePtr _node, transport::PublisherPtr _visPub):
	parentName(_parentName),
	node(_node),
	visPub(_visPub)
{}

void CellVisual::redraw ()
{
	visPub->Publish (plane);
	//visPub->Publish (edge1);
	//visPub->Publish (edge2);
}

void CellVisual::updatePose (const Pose3d &pose)
{
	msgs::Set (plane.mutable_pose (), pose * Pose3d (0,0,HEIGHT, 0,0,0));
	msgs::Set (edge1.mutable_pose (), pose * Pose3d (0,0,HEIGHT, 0,0,0));
	msgs::Set (edge2.mutable_pose (), pose * Pose3d (0,0,HEIGHT, 0,0,0));
}

Vector3d CellMatrix::cellMap (int i, int j) {
	return Vector3d (xM + double (i) * cellSize, yM + double (j) * cellSize, 0);
}

void CellMatrix::build (double _cellSize)
{
	cells.resize (rows * cols);

	cellSize = _cellSize;

	for (int i = 0; i < rows; i++) 	{
		for (int j = 0; j < cols; j++) {
			CellVisual *curr = new CellVisual (parentName, node, visPub);

			curr->build (cols*i + j, cellSize, EDGE_SIZE);
			cells[cols*i + j] = curr;
		}
}

	updatePose (Pose3d ());
}

void CellMatrix::updatePose (const Pose3d &pose)
{
	for (int i = 0; i < rows; i++) 	{
		for (int j = 0; j < cols; j++) {
			Vector3d currPos = cellMap (i, j);
			CellVisual *curr = cells[i*cols + j];

			curr->updatePose (pose * Pose3d (currPos, Quaterniond::Identity));
			curr->redraw ();
		}
	}
}

GZ_REGISTER_MODEL_PLUGIN(VisibilityGrid)









































