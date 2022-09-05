#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <assimp/Importer.hpp>
#include <cstddef>
#include <iterator>
#include <sdformat-9.7/sdf/sdf.hh>
#include <utility>

class ElementIterator {
 public:
  ElementIterator(sdf::ElementPtr parent, std::string name) : parent_(std::move(parent)), name_(std::move(name)) {}

  struct Iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = sdf::ElementPtr;
    using pointer = sdf::ElementPtr*;
    using reference = sdf::ElementPtr&;

    explicit Iterator(sdf::ElementPtr element, std::string name)
        : element_(std::move(element)), name_(std::move(name)) {}

    reference operator*() { return element_; }
    pointer operator->() { return &element_; }

    Iterator& operator++() {
      element_ = element_->GetNextElement(name_);
      return *this;
    }

    friend bool operator==(const Iterator& a, const Iterator& b) { return a.element_ == b.element_; };
    friend bool operator!=(const Iterator& a, const Iterator& b) { return a.element_ != b.element_; };

   private:
    sdf::ElementPtr element_;
    std::string name_;
  };

  Iterator begin() { return Iterator(parent_->GetElement(name_), name_); }
  Iterator end() { return Iterator(sdf::ElementPtr(), name_); }

 private:
  sdf::ElementPtr parent_;
  std::string name_;
};

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
      auto link_pose_in_world_frame =  link_pose + model_pose;

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

  auto model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("fake_robot_description");
  auto robot_model = model_loader->getModel();
  auto moveit_world = std::make_shared<collision_detection::World>();
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model, moveit_world);
  planning_scene->usePlanningSceneMsg(planning_scene_msg);

  return planning_scene;
}
