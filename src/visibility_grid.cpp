#include "visibility_grid.h"

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace ros;
using namespace std;

void VisibilityGrid::Load (VisualPtr _visual, sdf::ElementPtr _sdf)
{
	visual = _visual;

	scene = visual->GetScene ();
	scene->CreateGrid (1,1,Color(1,0,0));
}

void VisibilityGrid::UpdateChild ()
{

}

GZ_REGISTER_VISUAL_PLUGIN(VisibilityGrid)
