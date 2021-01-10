#pragma once

#include <moveit/collision_detection/collision_common.h>
#include <arc_utilities/ostream_operators.hpp>

static std::ostream &operator<<(std::ostream &out, collision_detection::CollisionResult const &cr) {
  if (not cr.collision) {
    out << "no collision";
  } else {
    if (cr.contacts.empty()) {
      out << "collision found but no contacts reported";
    } else {
      collision_detection::CollisionResult::ContactMap::key_type name;
      collision_detection::CollisionResult::ContactMap::mapped_type contacts;
      // TODO: upgrade to C++17 and use structured binding. Not upgrading because realtime is on 16.04
      std::tie(name, contacts) = *cr.contacts.cbegin();
      if (not contacts.empty()) {
        auto const contact = contacts.front();
        out << "collision between " << contact.body_name_1 << " and " << contact.body_name_2;
        bool multiple_contacts = cr.contacts.size() > 1 or contacts.size() > 1;
        if (multiple_contacts) {
          out << " among others...\n";
        }
      }
    }
  }
  return out;
}