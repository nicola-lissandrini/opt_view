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

	projectedViewVisual = new ProjectedViewVisual (_model->GetScopedName (),
												   node, visPub);

	if (!ros::isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "visibility_grid");
	}

	rosNode = new NodeHandle ("visibility_grid");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions so =
			SubscribeOptions::create<opt_view::ProjectedView> ("update_visibility_matrix", 1,
																  boost::bind(&VisibilityGrid::updateMatrix, this, _1),
																  ros::VoidPtr(), &rosQueue);

	rosSub = rosNode->subscribe (so);
	rosQueueThread = thread (bind (&VisibilityGrid::queueThread, this));

	updateConnection = event::Events::ConnectRender (
				boost::bind(&VisibilityGrid::UpdateChild, this));
}

void VisibilityGrid::updateMatrix (const opt_view::ProjectedViewConstPtr &projectedView)
{
	common::Time tt = model->GetWorld ()->SimTime ();
	double t = tt.Double ();
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

void ProjectedViewVisual::build ()
{
	viewVisual.set_parent_name (parentName);

	msgs::Geometry *viewGeom = viewVisual.mutable_geometry ();
	viewGeom->set_type (msgs::Geometry::POLYLINE);
	msgs::Polyline *poly  = viewGeom->add_polyline ();

	msgs::Set (poly->add_point (), Vector2d ());
	msgs::Set (poly->add_point (), Vector2d ());
	msgs::Set (poly->add_point (), Vector2d ());
	msgs::Set (poly->add_point (), Vector2d ());

	poly->set_height (0.005);
}

GZ_REGISTER_MODEL_PLUGIN(VisibilityGrid)









































