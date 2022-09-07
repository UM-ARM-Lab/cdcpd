#include <cdcpd/sdformat_to_planning_scene.h>

int main(int argc, char* argv[]) {
  std::string sdf_filename = "car5_real_cdcpd.world";
  ros::init(argc, argv, "test_sdf_to_planning_scene");
  auto const planning_scene = sdf_to_planning_scene(sdf_filename, "world");
  return EXIT_SUCCESS;
}