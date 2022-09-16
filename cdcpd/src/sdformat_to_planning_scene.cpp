#include "cdcpd/sdformat_to_planning_scene.h"


std::string const LOGNAME = "sdformat_to_planning_scene";

planning_scene::PlanningScenePtr sdf_to_planning_scene(std::string const& sdf_filename, std::string const& frame_id) {
  std::cout << "Loading SDF\n";

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.name = "cdcpd_world";
  planning_scene_msg.robot_model_name = "fake_robot";
  planning_scene_msg.is_diff = false;

  auto sdf_element = std::make_shared<sdf::SDF>();
  sdf::init(sdf_element);

  if (!sdf::readFile(sdf_filename, sdf_element)) {
    std::cerr << sdf_filename << " is not a valid SDF file!" << std::endl;
    return nullptr;
  }

  auto const root = sdf_element->Root();
  if (!root->HasElement("world")) {
    std::cerr << sdf_filename << " the root element is not <world>" << std::endl;
    return nullptr;
  }

  auto const world = root->GetElement("world");

  for (auto const& model : ElementIterator(world, "model")) {
    const auto model_name = model->Get<std::string>("name");
    const auto model_pose = model->Get<ignition::math::Pose3d>("pose");
    std::cout << "model " << model_name << " pose: " << model_pose << std::endl;

    for (auto const& link : ElementIterator(model, "link")) {
      const auto link_name = link->Get<std::string>("name");
      const auto link_pose = link->Get<ignition::math::Pose3d>("pose");
      const auto link_collision = link->GetElement("collision");
      const auto link_collision_name = link_collision->Get<std::string>("name");
      std::cout << " link " << link_name << " pose: " << link_pose << std::endl;
      auto link_pose_in_world_frame = model_pose + link_pose;

      for (auto const& link_collision_geometry : ElementIterator(link_collision, "geometry")) {
        std::cout << "  link collision geometry " << std::endl;

        moveit_msgs::CollisionObject collision_object;
        collision_object.id = model_name + "::" + link_name + "::" + link_collision_name;
        collision_object.header.frame_id = frame_id;
        collision_object.operation = moveit_msgs::CollisionObject::ADD;
        collision_object.pose.orientation.w = 1;

        geometry_msgs::Pose link_pose_msg;
        link_pose_msg.orientation.x = link_pose_in_world_frame.Rot().X();
        link_pose_msg.orientation.y = link_pose_in_world_frame.Rot().Y();
        link_pose_msg.orientation.z = link_pose_in_world_frame.Rot().Z();
        link_pose_msg.orientation.w = link_pose_in_world_frame.Rot().W();
        link_pose_msg.position.x = link_pose_in_world_frame.X();
        link_pose_msg.position.y = link_pose_in_world_frame.Y();
        link_pose_msg.position.z = link_pose_in_world_frame.Z();

        // for each collision geometry, convert it to the planning scene
        if (link_collision_geometry->HasElement("mesh")) {
          auto const mesh_element = link_collision_geometry->GetElement("mesh");
          auto const uri = mesh_element->GetElement("uri")->Get<std::string>();
          std::cout << "   mesh element " << uri << std::endl;

          Assimp::Importer importer;
          const aiScene* scene = importer.ReadFile(uri, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                                                            aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

          // If the import failed, report it
          if (scene == nullptr) {
            std::cout << importer.GetErrorString() << std::endl;
            continue;
          }

          for (auto i{0u}; i < scene->mNumMeshes; ++i) {
            auto mesh = scene->mMeshes[i];

            shape_msgs::Mesh mesh_msg;
            std::cout << "     Mesh has " << mesh->mNumFaces << " faces\n";
            for (auto face_idx{0u}; face_idx < mesh->mNumFaces; ++face_idx) {
              auto face = mesh->mFaces[face_idx];
              shape_msgs::MeshTriangle triangle_msg;
              triangle_msg.vertex_indices[0] = face.mIndices[0];
              triangle_msg.vertex_indices[1] = face.mIndices[1];
              triangle_msg.vertex_indices[2] = face.mIndices[2];
              mesh_msg.triangles.push_back(triangle_msg);
            }
            for (auto vertex_idx{0u}; vertex_idx < mesh->mNumVertices; ++vertex_idx) {
              auto vertex = mesh->mVertices[vertex_idx];
              geometry_msgs::Point point_msg;
              point_msg.x = vertex.x;
              point_msg.y = vertex.y;
              point_msg.z = vertex.z;
              mesh_msg.vertices.push_back(point_msg);
            }
            collision_object.meshes.push_back(mesh_msg);
            collision_object.mesh_poses.push_back(link_pose_msg);
          }
          std::cout << "Done here\n";
        } else if (link_collision_geometry->HasElement("box")) {
          auto const box_element = link_collision_geometry->GetElement("box");
          auto size = box_element->GetElement("size")->Get<ignition::math::Vector3d>();
          size *= 2;
          std::cout << "  box element " << size << std::endl;
          shape_msgs::SolidPrimitive primitive;
          primitive.type = shape_msgs::SolidPrimitive::BOX;
          primitive.dimensions.push_back(size.X());
          primitive.dimensions.push_back(size.Y());
          primitive.dimensions.push_back(size.Z());
          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(link_pose_msg);
        } else if (link_collision_geometry->HasElement("sphere")) {
          auto const sphere_element = link_collision_geometry->GetElement("sphere");
          auto const radius = sphere_element->GetElement("radius")->Get<double>();
          std::cout << "   sphere element " << radius << std::endl;
          shape_msgs::SolidPrimitive primitive;
          primitive.type = shape_msgs::SolidPrimitive::SPHERE;
          primitive.dimensions.push_back(radius);
          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(link_pose_msg);
        } else if (link_collision_geometry->HasElement("cylinder")) {
          auto const cylinder_element = link_collision_geometry->GetElement("cylinder");
          auto const height = 2 * cylinder_element->GetElement("length")->Get<double>();
          auto const radius = cylinder_element->GetElement("radius")->Get<double>();
          std::cout << "   cylinder element " << radius << " " << height << std::endl;
          shape_msgs::SolidPrimitive primitive;
          primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
          primitive.dimensions.push_back(height);
          primitive.dimensions.push_back(radius);
          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(link_pose_msg);
        }

        planning_scene_msg.world.collision_objects.push_back(collision_object);
      }
    }
  }

  std::cout << "Done with big code block\n";

  auto model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("fake_robot_description");
  std::cout << "before getModel\n";
  auto robot_model = model_loader->getModel();
  std::cout<<"before moveit_world construction\n";
  auto moveit_world = std::make_shared<collision_detection::World>();
  std::cout <<"before planning_scene construction\n";
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model, moveit_world);
  std::cout <<"before usePlanningSceneMsg\n" << std::flush;
  planning_scene->usePlanningSceneMsg(planning_scene_msg);

  return planning_scene;
}
