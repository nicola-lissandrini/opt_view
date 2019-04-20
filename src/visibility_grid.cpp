#include "visibility_grid.h"

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;

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

	cellVisual = new CellVisual (node, visPub);

	cellVisual->build (0, CELL_SIZE, EDGE_SIZE);

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}


void VisibilityGrid::UpdateChild ()
{
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();

	cellVisual->updatePose (Pose3d (t*0.1, 0, 0, 0,0,0));
	cellVisual->redraw ();
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

void CellVisual::redraw ()
{
	visPub->Publish (plane);
	visPub->Publish (edge1);
	visPub->Publish (edge2);
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

void CellMatrix::build (double _cellSize, const string &parentName)
{
	cells.resize (rows * cols);

	cellSize = _cellSize;

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			CellVisual *curr = new CellVisual (node, visPub);
			Vector3d currPos = cellMap (i, j);

			curr->build (cols*i + j, cellSize, EDGE_SIZE);
			curr->updatePose (Pose3d (currPos, Quaterniond::Identity));
			curr->redraw ();
		}
	}



}

GZ_REGISTER_MODEL_PLUGIN(VisibilityGrid)









































