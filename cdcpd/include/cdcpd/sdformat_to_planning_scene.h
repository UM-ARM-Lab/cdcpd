#pragma once

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <assimp/Importer.hpp>
#include <cstddef>
#include <iterator>
// NOTE: Had to switch to 9.8 for my installation. Peter using 9.7
#include <sdformat-9.8/sdf/sdf.hh>
#include <utility>

#include <string>

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

planning_scene::PlanningScenePtr sdf_to_planning_scene(std::string const& sdf_filename,
    std::string const& frame_id);
