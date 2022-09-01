#include <assimp/postprocess.h>  // Post processing flags
#include <assimp/scene.h>        // Output data structure
#include <moveit/planning_scene/planning_scene.h>

#include <assimp/Importer.hpp>  // C++ importer interface
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

    explicit Iterator(sdf::ElementPtr element) : element_(std::move(element)) {}

    reference operator*() { return element_; }
    pointer operator->() { return &element_; }

    Iterator& operator++() {
      element_ = element_->GetNextElement("model");
      return *this;
    }

    friend bool operator==(const Iterator& a, const Iterator& b) { return a.element_ == b.element_; };
    friend bool operator!=(const Iterator& a, const Iterator& b) { return a.element_ != b.element_; };

   private:
    sdf::ElementPtr element_;
  };

  Iterator begin() { return Iterator(parent_->GetElement(name_)); }
  Iterator end() { return Iterator(sdf::ElementPtr()); }

 private:
  sdf::ElementPtr parent_;
  std::string name_;
};

std::string const LOGNAME = "sdformat_to_planning_scene";

int main(int argc, char* argv[]) {
  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.name = "cdcpd_world";
  planning_scene_msg.robot_model_name = "";
  planning_scene_msg.is_diff = false;

  std::string sdf_filename = "car5_real_cdcpd.world";

  auto sdf_element = std::make_shared<sdf::SDF>();
  sdf::init(sdf_element);

  if (!sdf::readFile(sdf_filename, sdf_element)) {
    std::cerr << sdf_filename << " is not a valid SDF file!" << std::endl;
    return EXIT_SUCCESS;
  }

  auto const root = sdf_element->Root();
  if (!root->HasElement("world")) {
    std::cerr << sdf_filename << " the root element is not <world>" << std::endl;
    return EXIT_SUCCESS;
  }

  auto const world = root->GetElement("world");

  for (auto const& model : ElementIterator(world, "model")) {
    const auto model_name = model->Get<std::string>("name");
    std::cout << "model " << model_name << std::endl;

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = model_name;
    collision_object.header.frame_id = "cdcpd_world";  // FIXME: what should this be?
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    for (auto const& link : ElementIterator(model, "link")) {
      const auto link_name = link->Get<std::string>("name");
      const auto link_pose = link->Get<ignition::math::Vector3d>("pose");
      const auto link_collision = link->GetElement("collision");
      std::cout << " link " << link_name << " pose: " << link_pose << std::endl;

      for (auto const& link_collision_geometry : ElementIterator(link_collision, "geometry")) {
        const auto link_geometry_name = link_collision_geometry->Get<std::string>("name");
        std::cout << "  link collision geometry " << link_geometry_name << std::endl;

        // for each collision geometry, convert it to the planning scene
        if (link_collision_geometry->HasElement("mesh")) {
          auto const mesh_element = link_collision_geometry->GetElement("mesh");
          auto const uri = mesh_element->GetElement("uri")->Get<std::string>();
          std::cout << "   mesh element " << uri << std::endl;

          Assimp::Importer importer;
          const aiScene* scene = importer.ReadFile(uri, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                                                            aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

          // If the import failed, report it
          if (nullptr != scene) {
            std::cout << importer.GetErrorString() << std::endl;
            continue;
          }

          shape_msgs::Mesh mesh_msg;
          // mesh_msg.triangles
          // mesh_msg.vertices
          collision_object.meshes.push_back(mesh_msg);
        }

        if (link_collision_geometry->HasElement("box")) {
          auto const box_element = link_collision_geometry->GetElement("box");
          auto const size_element = box_element->GetElement("size");
          std::cout << "   box element " << size_element->Get<ignition::math::Vector3d>() << std::endl;
          // collision_object.primitive_poses.push_back()
        }

        if (link_collision_geometry->HasElement("cylinder")) {
          auto const cylinder_element = link_collision_geometry->GetElement("cylinder");
          auto const radius = cylinder_element->GetElement("radius")->Get<double>();
          auto const length = cylinder_element->GetElement("length")->Get<double>();
          std::cout << "   cylinder element " << radius << std::endl;
          // collision_object.primitive_poses.push_back()
        }
      }
    }

    planning_scene_msg.world.collision_objects.push_back(collision_object);
  }

  auto empty_urdf = std::make_shared<urdf::ModelInterface>();
  auto empty_srdf = std::make_shared<srdf::Model>();
  auto robot_model = std::make_shared<robot_model::RobotModel>(empty_urdf, empty_srdf);
  auto moveit_world = std::make_shared<collision_detection::World>();
  planning_scene::PlanningScene planning_scene(robot_model, moveit_world);
  planning_scene.usePlanningSceneMsg(planning_scene_msg);
  return EXIT_SUCCESS;
}
