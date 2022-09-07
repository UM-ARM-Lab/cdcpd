#include <moveit/planning_scene/planning_scene.h>

#include <string>

planning_scene::PlanningScenePtr sdf_to_planning_scene(std::string const& sdf_filename,
    std::string const& frame_id);
