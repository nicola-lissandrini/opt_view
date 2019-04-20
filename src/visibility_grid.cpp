#include "visibility_grid.h"

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace std;

#define CELL_SIZE 0.1
#define EDGE_SIZE 0.0008
#define CSH (CELL_SIZE/2)
#define ESH (EDGE_SIZE/2)

void VisibilityGrid::Load (ModelPtr _model, sdf::ElementPtr _sdf)
{
	Color colorInner(0,0,0,0.3), colorZero (0,0,0,0);
	Color colorEdge (0,0,0,0.45);

	model = _model;

	node = transport::NodePtr (new transport::Node());
	node->Init (model->GetWorld ()->Name ());
	visPub = node->Advertise<msgs::Visual> ("~/visual",1000);


	buildPlane (colorInner, colorZero);
	buildEdge1 (colorEdge, colorZero);
	buildEdge2 (colorEdge, colorZero);

	visPub->Publish (edge1);
	visPub->Publish (edge2);
	visPub->Publish (plane);

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::buildPlane (const Color &ambientColor, const Color &emitColor)
{
	plane.set_name ("__CELL_PLANE__");
	plane.set_parent_name (model->GetScopedName ());

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

	msgs::Set (plane.mutable_pose (), Pose3d (0,0,EDGE_SIZE/2, 0,0,0));
}

void VisibilityGrid::buildEdge1 (const Color &ambientColor, const Color &emitColor)
{
	edge1.set_name ("__CELL_EDGE_1__");
	edge1.set_parent_name (model->GetScopedName ());
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

void VisibilityGrid::buildEdge2 (const Color &ambientColor, const Color &emitColor)
{
	edge2.set_name ("__CELL_EDGE_2__");
	edge2.set_parent_name (model->GetScopedName ());
	msgs::Geometry *edgesGeom = edge2.mutable_geometry ();

	edgesGeom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly  = edgesGeom->add_polyline ();

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

void VisibilityGrid::UpdateChild ()
{
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();

	model->SetWorldPose (Pose3d (t*0.1,0,0,0,0,0));
}

GZ_REGISTER_MODEL_PLUGIN(VisibilityGrid)
