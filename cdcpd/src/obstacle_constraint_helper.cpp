#include "cdcpd/obstacle_constraint_helper.h"

ObstacleConstraintHelper::ObstacleConstraintHelper(bool moveit_enabled,
    ros::Publisher const& contact_marker_pub, ros::Publisher const& scene_pub,
    std::string const camera_frame, std::string const sdf_filename,
    double const min_distance_threshold)
    : moveit_enabled_(moveit_enabled),
      contact_marker_pub_(std::make_shared<ros::Publisher>(contact_marker_pub)),
      scene_pub_(std::make_shared<ros::Publisher>(scene_pub)),
      camera_frame_(camera_frame),
      sdf_filename_(sdf_filename),
      min_distance_threshold_(min_distance_threshold)
{
    load_sdformat_file();
}

ObstacleConstraints ObstacleConstraintHelper::find_nearest_points_and_normals()
{
    collision_detection::CollisionRequest req;
    req.contacts = true;
    // req.distance = true;
    req.distance = false;

    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    auto const t0 = ros::Time::now();
    planning_scene_->checkCollisionUnpadded(req, res);
    auto const t1 = ros::Time::now();
    auto const dt = t1 - t0;
    ROS_DEBUG_STREAM_NAMED(PERF_LOGGER, "checkCollision = " << dt.toSec() << "s");

    // first fill up the contact markers with "zero" markers
    // rviz makes deleting markers hard, so it's easier to just publish a fixed number of markers
    // in the array
    vm::MarkerArray contact_markers;
    for (auto i{0}; i < MAX_CONTACTS_VIZ; ++i) {
      auto const [arrow, normal] = arrow_and_normal(i, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      contact_markers.markers.push_back(arrow);
      contact_markers.markers.push_back(normal);
    }

    ObstacleConstraints obstacle_constraints;
    auto contact_idx = 0u;
    for (auto const& [contact_names, contacts] : res.contacts) {
      if (contacts.empty()) {
        continue;
      }

      auto const contact = contacts[0];
      auto add_interaction_constraint = [&](int contact_idx, int body_idx, std::string body_name,
                                            Eigen::Vector3d const& tracked_point_cdcpd_frame,
                                            Eigen::Vector3d const& object_point_cdcpd_frame) {
        // NOTE: if the tracked_point is inside the object, contact.depth will be negative. In this
        // case, the normal points in the opposite direction, starting at object_point and going
        // _away_ from tracked_point.
        auto const normal_dir = contact.depth > 0.0 ? 1.0 : -1.0;
        Eigen::Vector3d const normal_cdcpd_frame =
            ((tracked_point_cdcpd_frame - object_point_cdcpd_frame) * normal_dir).normalized();
        auto get_point_idx = [&]() {
          unsigned int point_idx;
          sscanf(body_name.c_str(), (collision_body_prefix + "%u").c_str(), &point_idx);
          return point_idx;
        };
        auto const point_idx = get_point_idx();
        obstacle_constraints.emplace_back(
            ObstacleConstraint{point_idx, object_point_cdcpd_frame.cast<float>(),
                normal_cdcpd_frame.cast<float>()});

        // debug & visualize
        {
          ROS_DEBUG_STREAM_NAMED(
              LOGNAME + ".moveit",
              "nearest point: " << contact.nearest_points[0].x() << ", "
                  << contact.nearest_points[0].y() << ", " << contact.nearest_points[0].z()
                  << " on " << contact.body_name_1 << " and " << contact.nearest_points[1].x()
                  << ", " << contact.nearest_points[1].y() << ", " << contact.nearest_points[1].z()
                  << " on " << contact.body_name_2 << " depth " << contact.depth
                  << " (in camera frame)");
          auto const [arrow, normal] = arrow_and_normal(contact_idx, tracked_point_cdcpd_frame,
            object_point_cdcpd_frame, normal_cdcpd_frame);
          if (contact_idx < MAX_CONTACTS_VIZ) {
            contact_markers.markers[2 * contact_idx] = arrow;
            contact_markers.markers[2 * contact_idx + 1] = normal;
          }
        }
      };

      if (contact.depth > min_distance_threshold_) {
        continue;
      }
      std::vector<std::string> bodies_to_ignore{"leftgripper_link", "leftgripper2_link",
          "left_tool", "rightgripper_link", "rightgripper2_link", "right_tool",};
      if (std::find(bodies_to_ignore.cbegin(), bodies_to_ignore.cend(), contact.body_name_1) !=
          bodies_to_ignore.cend()) {
        ROS_DEBUG_STREAM_NAMED(LOGNAME + ".moveit", "Ignoring " << contact.body_name_1);
        continue;
      }
      if (contact.body_name_1.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 0, contact.body_name_1, contact.nearest_points[0],
                                   contact.nearest_points[1]);
      } else if (contact.body_name_2.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 1, contact.body_name_2, contact.nearest_points[1],
                                   contact.nearest_points[0]);
      } else {
        continue;
      }

      ++contact_idx;
    }

    ROS_DEBUG_STREAM_NAMED(LOGNAME + ".contacts", "contacts: " << contact_idx);
    contact_marker_pub_->publish(contact_markers);
    return obstacle_constraints;
  }

std::pair<vm::Marker, vm::Marker> ObstacleConstraintHelper::arrow_and_normal(int contact_idx,
    const Eigen::Vector3d& tracked_point_cdcpd_frame,
    const Eigen::Vector3d& object_point_cdcpd_frame,
    const Eigen::Vector3d& normal_cdcpd_frame) const
{
    vm::Marker arrow, normal;
    arrow.id = 100 * contact_idx + 0;
    arrow.action = vm::Marker::ADD;
    arrow.type = vm::Marker::ARROW;
    arrow.ns = "arrow";
    arrow.header.frame_id = camera_frame_;
    arrow.header.stamp = ros::Time::now();
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.2;
    arrow.scale.x = 0.001;
    arrow.scale.y = 0.002;
    arrow.scale.z = 0.002;
    arrow.pose.orientation.w = 1;
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_cdcpd_frame));
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(tracked_point_cdcpd_frame));
    normal.id = 100 * contact_idx + 0;
    normal.action = vm::Marker::ADD;
    normal.type = vm::Marker::ARROW;
    normal.ns = "normal";
    normal.header.frame_id = camera_frame_;
    normal.header.stamp = ros::Time::now();
    normal.color.r = 0.4;
    normal.color.g = 1.0;
    normal.color.b = 0.7;
    normal.color.a = 0.6;
    normal.scale.x = 0.0015;
    normal.scale.y = 0.0025;
    normal.scale.z = 0.0025;
    normal.pose.orientation.w = 1;
    normal.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_cdcpd_frame));
    Eigen::Vector3d const normal_end_point_cdcpd_frame =
        object_point_cdcpd_frame + normal_cdcpd_frame * 0.02;
    normal.points.push_back(ConvertTo<geometry_msgs::Point>(normal_end_point_cdcpd_frame));

    return {arrow, normal};
}

ObstacleConstraints ObstacleConstraintHelper::get_moveit_obstacle_constraints(
    PointCloud::ConstPtr tracked_points)
{
    // No sense in loading this every timestep if the scene is static, right?
    // planning_scene_ = sdf_to_planning_scene(sdf_filename, "mock_camera_link");
    // TODO(dylan.colli): Do we need to update this everytime?
    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);

    // Do we need to publish this everytime still?
    scene_pub_->publish(planning_scene_msg_);
    geometry_msgs::TransformStamped identity_transform_msg;
    identity_transform_msg.child_frame_id = "mock_camera_link";
    identity_transform_msg.header.frame_id = camera_frame_;
    identity_transform_msg.header.stamp = ros::Time::now();
    identity_transform_msg.transform.rotation.w = 1;
    br_.sendTransform(identity_transform_msg);

    // customize by excluding some objects
    auto& world = planning_scene_->getWorldNonConst();
    std::vector<std::string> objects_to_ignore{
        "collision_sphere.link_1",
        "ground_plane.link",
    };
    for (auto const& object_to_ignore : objects_to_ignore) {
      if (world->hasObject(object_to_ignore)) {
        auto success = world->removeObject(object_to_ignore);
        if (success) {
          ROS_DEBUG_STREAM_NAMED(LOGNAME + ".moveit", "Successfully removed " << object_to_ignore);
        } else {
          ROS_ERROR_STREAM_NAMED(LOGNAME + ".moveit", "Failed to remove " << object_to_ignore);
        }
      }
    }

    auto& robot_state = planning_scene_->getCurrentStateNonConst();

    // remove the attached "tool boxes"
    std::vector<std::string> objects_to_detach{"left_tool_box", "right_tool_box"};
    for (auto const& object_to_detach : objects_to_detach) {
      if (not robot_state.hasAttachedBody(object_to_detach)) {
          continue;
      }
      auto success = robot_state.clearAttachedBody(object_to_detach);
      if (not success) {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to detach " << object_to_detach);
      }
    }

    planning_scene_->setActiveCollisionDetector(
        collision_detection::CollisionDetectorAllocatorBullet::create());

    // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs
    // self/world, so we have to make the tracked points part of the robot, hence
    // "attached collision objects"
    for (auto const& [tracked_point_idx, point] : enumerate(*tracked_points)) {
      Eigen::Isometry3d tracked_point_pose_cdcpd_frame = Eigen::Isometry3d::Identity();
      tracked_point_pose_cdcpd_frame.translation() = point.getVector3fMap().cast<double>();

      std::stringstream collision_body_name_stream;
      collision_body_name_stream << collision_body_prefix << tracked_point_idx;
      auto const collision_body_name = collision_body_name_stream.str();

      // FIXME: not moveit frame, but the base link_frame, could those be different?
      auto collision_shape = std::make_shared<shapes::Box>(0.01, 0.01, 0.01);

      robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {collision_shape},
                             {tracked_point_pose_cdcpd_frame}, std::vector<std::string>{},
                             "mock_camera_link");
    }

    // visualize
    //    visual_tools_.publishRobotState(robot_state, rviz_visual_tools::CYAN);

    ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Finding nearest points and normals");
    // return find_nearest_points_and_normals(planning_scene_);
    return find_nearest_points_and_normals();
  }

ObstacleConstraints ObstacleConstraintHelper::get_obstacle_constraints(
    pcl::shared_ptr<PointCloud> tracked_points)
{
    ObstacleConstraints obstacle_constraints;
    if (moveit_enabled_)
    {
        obstacle_constraints = get_moveit_obstacle_constraints(tracked_points);
        ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Got moveit obstacle constraints");
    }
    return obstacle_constraints;
}

void ObstacleConstraintHelper::load_sdformat_file()
{
    planning_scene_ = sdf_to_planning_scene(sdf_filename_, "mock_camera_link");
}