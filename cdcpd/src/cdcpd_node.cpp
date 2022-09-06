#include "cdcpd/cdcpd_node.h"

std::string const LOGNAME = "cdcpd_node";
constexpr auto const MAX_CONTACTS_VIZ = 25;
constexpr auto const PERF_LOGGER = "perf";

Eigen::Vector3f extent_to_env_size(Eigen::Vector3f const& bbox_lower,
    Eigen::Vector3f const& bbox_upper)
{
    return (bbox_upper - bbox_lower).cwiseAbs() + 2 * bounding_box_extend;
};

Eigen::Vector3f extent_to_center(Eigen::Vector3f const& bbox_lower,
    Eigen::Vector3f const& bbox_upper)
{
    return (bbox_upper + bbox_lower) / 2;
};

cv::Mat getHsvMask(ros::NodeHandle const& ph, cv::Mat const& rgb) {
    auto const hue_min = ROSHelpers::GetParamDebugLog<double>(ph, "hue_min", 340.0);
    auto const sat_min = ROSHelpers::GetParamDebugLog<double>(ph, "saturation_min", 0.3);
    auto const val_min = ROSHelpers::GetParamDebugLog<double>(ph, "value_min", 0.4);
    auto const hue_max = ROSHelpers::GetParamDebugLog<double>(ph, "hue_max", 20.0);
    auto const sat_max = ROSHelpers::GetParamDebugLog<double>(ph, "saturation_max", 1.0);
    auto const val_max = ROSHelpers::GetParamDebugLog<double>(ph, "value_max", 1.0);

    cv::Mat rgb_f;
    rgb.convertTo(rgb_f, CV_32FC3);
    rgb_f /= 255.0;  // get RGB 0.0-1.0
    cv::Mat color_hsv;
    cvtColor(rgb_f, color_hsv, CV_RGB2HSV);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat hsv_mask;
    auto hue_min1 = hue_min;
    auto hue_max2 = hue_max;
    if (hue_min > hue_max) {
        hue_max2 = 360;
        hue_min1 = 0;
    }
    cv::inRange(color_hsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max2, sat_max, val_max), mask1);
    cv::inRange(color_hsv, cv::Scalar(hue_min1, sat_min, val_min), cv::Scalar(hue_max, sat_max, val_max), mask2);
    bitwise_or(mask1, mask2, hsv_mask);

    return hsv_mask;
}

void print_bodies(robot_state::RobotState const& state) {
    std::vector<robot_state::AttachedBody const*> bs;
    std::cout << "Attached Bodies:\n";
    state.getAttachedBodies(bs);
    for (auto const& b : bs) {
        std::cout << b->getName() << '\n';
    }
}

CDCPD_Publishers::CDCPD_Publishers(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
    original_publisher = nh.advertise<PointCloud>("cdcpd/original", 10);
    masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 10);
    downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 10);
    template_publisher = nh.advertise<PointCloud>("cdcpd/template", 10);
    pre_template_publisher = nh.advertise<PointCloud>("cdcpd/pre_template", 10);
    output_publisher = nh.advertise<PointCloud>("cdcpd/output", 10);
    order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = ph.advertise<vm::MarkerArray>("contacts", 10);
    bbox_pub = ph.advertise<jsk_recognition_msgs::BoundingBox>("bbox", 10);
}


CDCPD_Node_Parameters::CDCPD_Node_Parameters(ros::NodeHandle& nh, ros::NodeHandle& ph)
    : points_name(ROSHelpers::GetParam<std::string>(ph, "points", "")),
      rgb_topic(ROSHelpers::GetParam<std::string>(ph, "rgb_topic", "/camera/color/image_raw")),
      depth_topic(ROSHelpers::GetParam<std::string>(ph, "depth_topic", "/camera/depth/image_rect_raw")),
      info_topic(ROSHelpers::GetParam<std::string>(ph, "info_topic", "/camera/depth/camera_info")),
      camera_frame(ROSHelpers::GetParam<std::string>(ph, "camera_frame", "camera_color_optical_frame")),
      grippers_info_filename(ROSHelpers::GetParamRequired<std::string>(ph, "grippers_info", "cdcpd_node")),
      num_points(ROSHelpers::GetParam<int>(nh, "rope_num_points", 11)),
      max_rope_length(ROSHelpers::GetParam<float>(nh, "max_rope_length", 1.0)),
      moveit_enabled(ROSHelpers::GetParam<bool>(ph, "moveit_enabled", false))
{}


CDCPD_Moveit_Node::CDCPD_Moveit_Node(std::string const& robot_namespace)
    : robot_namespace_(robot_namespace),
      robot_description_(robot_namespace + "/robot_description"),
      ph("~"),
      publishers(nh, ph),
      node_params(nh, ph),
      cdcpd_params(ph),
      rope_configuration(node_params.num_points, node_params.max_rope_length),
      scene_monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description_)),
      model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(robot_description_)),
      model_(model_loader_->getModel()),
      visual_tools_("robot_root", "cdcpd_moveit_node", scene_monitor_),
      tf_listener_(tf_buffer_)
{
    auto const scene_topic = ros::names::append(robot_namespace,
        "move_group/monitored_planning_scene");
    auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
    scene_monitor_->startSceneMonitor(scene_topic);
    moveit_ready = scene_monitor_->requestPlanningSceneState(service_name);
    if (not moveit_ready)
    {
        ROS_WARN_NAMED(LOGNAME,
            "Could not get the moveit planning scene. This means no obstacle constraints.");
    }

    // For use with TF and "fixed points" for the constrain step
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Using frame " << node_params.camera_frame);


    // TODO(dylan.colli): Initialize gripper constraints here. This is what Peter has on his ICRA
    // branch but this should be done via the service.
    // cdcpd::GripperConstraint left_gripper_constraint;
    // left_gripper_constraint.frame_id = "mocap_left_hand_left_hand";
    // left_gripper_constraint.node_index = 0;
    // gripper_constraints_.push_back(left_gripper_constraint);
    // cdcpd::GripperConstraint right_gripper_constraint;
    // right_gripper_constraint.frame_id = "mocap_right_hand_right_hand";
    // right_gripper_constraint.node_index = 24;
    // gripper_constraints_.push_back(right_gripper_constraint);


    // Initial connectivity model of rope
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "max segment length " << rope_configuration.max_segment_length);

    // This is for the mesh loading refactor
    // planning_scene_ = sdf_to_planning_scene(sdf_filename, "mock_camera_link");
    // planning_scene_->getPlanningSceneMsg(planning_scene_msg_);

    gripper_service_ = ph.advertiseService("set_gripper_constraints",
        &CDCPD_Moveit_Node::set_gripper_constraints, this);

    // initialize start and end points for rope
    ConstraintPosDotIndices const init_gripper_constraints = get_q_config();
    auto const init_q_config = init_gripper_constraints.q_config;
    Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    end_position[2] += node_params.max_rope_length;
    // TODO(dylan.colli): Since a gripper could technically hold more than one point, should we
    // specify number of grippers in another way?
    uint const gripper_count = gripper_constraints_.size();
    if (gripper_count == 2u) {
        start_position = init_q_config[0].translation().cast<float>();
        end_position = init_q_config[1].translation().cast<float>();
    } else if (gripper_count == 1u) {
        start_position = init_q_config[0].translation().cast<float>();
        end_position = start_position;
        end_position[1] += node_params.max_rope_length;
    } else if (gripper_count == 0u) {
        start_position << -node_params.max_rope_length / 2, 0, 1.0;
        end_position << node_params.max_rope_length / 2, 0, 1.0;
    }

    // TODO: decide on rope versus cloth configuration here.
    rope_configuration.initializeTracking(start_position, end_position);

    std::unique_ptr<CDCPD> cdcpd_new(new CDCPD(nh, ph, rope_configuration.initial.points,
        rope_configuration.initial.edges,
        cdcpd_params.objective_value_threshold, cdcpd_params.use_recovery, cdcpd_params.alpha,
        cdcpd_params.beta, cdcpd_params.lambda, cdcpd_params.k_spring, cdcpd_params.zeta,
        cdcpd_params.obstacle_cost_weight, cdcpd_params.fixed_points_weight));
    cdcpd = std::move(cdcpd_new);

    // Define the callback wrappers we need to pass to ROS nodes.
    auto const callback_wrapper = [&](cv::Mat const& rgb, cv::Mat const& depth,
        cv::Matx33d const& intrinsics)
    {
        callback(rgb, depth, intrinsics);
    };
    auto const points_callback_wrapper = [&](const sensor_msgs::PointCloud2ConstPtr& points_msg)
    {
        points_callback(points_msg);
    };

    if (node_params.points_name.empty()) {
        ROS_INFO_NAMED(LOGNAME, "subscribing to RGB + Depth");
        auto camera_sub_setup = CameraSubSetup(node_params.rgb_topic, node_params.depth_topic,
          node_params.info_topic);
        // wait a second so the TF buffer can fill
        ros::Duration(0.5).sleep();

        KinectSub sub(callback_wrapper, camera_sub_setup);
        ros::waitForShutdown();
    } else {
        ROS_INFO_NAMED(LOGNAME, "subscribing to points");
        auto sub = nh.subscribe<sensor_msgs::PointCloud2>(node_params.points_name, 10,
            points_callback_wrapper);
        ros::spin();
    }
  }

ObstacleConstraints CDCPD_Moveit_Node::find_nearest_points_and_normals(
    planning_scene_monitor::LockedPlanningSceneRW planning_scene,
    Eigen::Isometry3d const& cdcpd_to_moveit)
{
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.distance = true;
    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    planning_scene->checkCollisionUnpadded(req, res);

    // first fill up the contact markers with "zero" markers
    // rviz makes deleting markers hard, so it's easier to just publish a fixed number of markers
    // in the array
    vm::MarkerArray contact_markers;
    for (auto i{0}; i < MAX_CONTACTS_VIZ; ++i) {
      auto const [arrow, normal] = arrow_and_normal(i, Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
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
                                            Eigen::Vector3d const& tracked_point_moveit_frame,
                                            Eigen::Vector3d const& object_point_moveit_frame) {
        // NOTE: if the tracked_point is inside the object, contact.depth will be negative. In this
        // case, the normal points in the opposite direction, starting at object_point and going
        // _away_ from tracked_point.
        auto const normal_dir = contact.depth > 0.0 ? 1.0 : -1.0;
        Eigen::Vector3d const object_point_cdcpd_frame =
            cdcpd_to_moveit.inverse() * object_point_moveit_frame;
        Eigen::Vector3d const tracked_point_cdcpd_frame =
            cdcpd_to_moveit.inverse() * tracked_point_moveit_frame;
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
                  << " (in moveit frame)");
          auto const [arrow, normal] =
              arrow_and_normal(contact_idx, tracked_point_moveit_frame, object_point_moveit_frame,
                               object_point_cdcpd_frame, normal_cdcpd_frame);
          if (contact_idx < MAX_CONTACTS_VIZ) {
            contact_markers.markers[2 * contact_idx] = arrow;
            contact_markers.markers[2 * contact_idx + 1] = normal;
          }
        }
      };

      if (contact.depth > min_distance_threshold) {
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
    publishers.contact_marker_pub.publish(contact_markers);
    return obstacle_constraints;
  }

std::pair<vm::Marker, vm::Marker> CDCPD_Moveit_Node::arrow_and_normal(int contact_idx,
    const Eigen::Vector3d& tracked_point_moveit_frame,
    const Eigen::Vector3d& object_point_moveit_frame,
    const Eigen::Vector3d& object_point_cdcpd_frame,
    const Eigen::Vector3d& normal_cdcpd_frame) const
{
    vm::Marker arrow, normal;
    arrow.id = 100 * contact_idx + 0;
    arrow.action = vm::Marker::ADD;
    arrow.type = vm::Marker::ARROW;
    arrow.ns = "arrow";
    arrow.header.frame_id = moveit_frame;
    arrow.header.stamp = ros::Time::now();
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.2;
    arrow.scale.x = 0.001;
    arrow.scale.y = 0.002;
    arrow.scale.z = 0.002;
    arrow.pose.orientation.w = 1;
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_moveit_frame));
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(tracked_point_moveit_frame));
    normal.id = 100 * contact_idx + 0;
    normal.action = vm::Marker::ADD;
    normal.type = vm::Marker::ARROW;
    normal.ns = "normal";
    normal.header.frame_id = node_params.camera_frame;
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

ObstacleConstraints CDCPD_Moveit_Node::get_moveit_obstacle_constriants(
    PointCloud::ConstPtr tracked_points)
{
    Eigen::Isometry3d cdcpd_to_moveit;
    try {
      auto const cdcpd_to_moveit_msg =
          tf_buffer_.lookupTransform(moveit_frame, node_params.camera_frame, ros::Time(0),
              ros::Duration(10));
      cdcpd_to_moveit = ehc::GeometryTransformToEigenIsometry3d(cdcpd_to_moveit_msg.transform);
    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(10.0, "Unable to lookup transform from " << node_params.camera_frame
          << " to " << moveit_frame << ": " << ex.what());
      return {};
    }

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor_);

    // customize by excluding some objects
    auto& world = planning_scene->getWorldNonConst();
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

    auto& robot_state = planning_scene->getCurrentStateNonConst();

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

    planning_scene->setActiveCollisionDetector(
        collision_detection::CollisionDetectorAllocatorBullet::create());

    // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs self/world,
    // so we have to make the tracked points part of the robot, hence "attached collision objects"
    for (auto const& [tracked_point_idx, point] : enumerate(*tracked_points)) {
      Eigen::Vector3d const tracked_point_cdcpd_frame = point.getVector3fMap().cast<double>();
      Eigen::Vector3d const tracked_point_moveit_frame =
        cdcpd_to_moveit * tracked_point_cdcpd_frame;
      Eigen::Isometry3d tracked_point_pose_moveit_frame = Eigen::Isometry3d::Identity();
      tracked_point_pose_moveit_frame.translation() = tracked_point_moveit_frame;

      std::stringstream collision_body_name_stream;
      collision_body_name_stream << collision_body_prefix << tracked_point_idx;
      auto const collision_body_name = collision_body_name_stream.str();

      // FIXME: not moveit frame, but the base link_frame, could those be different?
      auto sphere = std::make_shared<shapes::Box>(0.01, 0.01, 0.01);

      robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {sphere},
                             {tracked_point_pose_moveit_frame}, std::vector<std::string>{},
                             "base_link");
    }

    // visualize
    //    visual_tools_.publishRobotState(robot_state, rviz_visual_tools::CYAN);

    ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Finding nearest points and normals");
    return find_nearest_points_and_normals(planning_scene, cdcpd_to_moveit);
  }

void CDCPD_Moveit_Node::callback(cv::Mat const& rgb, cv::Mat const& depth,
    cv::Matx33d const& intrinsics)
{
    auto const t0 = ros::Time::now();
    ConstraintPosDotIndices gripper_info = get_q_config();

    publish_bbox();

    auto obstacle_constraints = get_obstacle_constraints();

    auto const hsv_mask = getHsvMask(ph, rgb);
    auto const out = (*cdcpd)(rgb, depth, hsv_mask, intrinsics,
                              rope_configuration.tracked.points,
                              obstacle_constraints, rope_configuration.max_segment_length,
                              gripper_info.q_dot, gripper_info.q_config,
                              gripper_info.gripper_indices);
    rope_configuration.tracked.points = out.gurobi_output;
    publish_outputs(t0, out);
    reset_if_bad(out);
};

void CDCPD_Moveit_Node::points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    auto const t0 = ros::Time::now();
    ConstraintPosDotIndices gripper_info = get_q_config();

    publish_bbox();
    publish_template();
    auto obstacle_constraints = get_obstacle_constraints();

    pcl::PCLPointCloud2 points_v2;
    pcl_conversions::toPCL(*points_msg, points_v2);
    auto points = boost::make_shared<PointCloudRGB>();
    pcl::fromPCLPointCloud2(points_v2, *points);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "unfiltered points: " << points->size());

    auto const out = (*cdcpd)(points, rope_configuration.tracked.points, obstacle_constraints,
        rope_configuration.max_segment_length, gripper_info.q_dot, gripper_info.q_config,
        gripper_info.gripper_indices);
    rope_configuration.tracked.points = out.gurobi_output;
    publish_outputs(t0, out);
    reset_if_bad(out);
}

void CDCPD_Moveit_Node::publish_bbox() const
{
    jsk_recognition_msgs::BoundingBox bbox_msg;
    bbox_msg.header.stamp = ros::Time::now();
    bbox_msg.header.frame_id = node_params.camera_frame;

    auto const bbox_size = extent_to_env_size(cdcpd->last_lower_bounding_box,
        cdcpd->last_upper_bounding_box);
    auto const bbox_center = extent_to_center(cdcpd->last_lower_bounding_box,
        cdcpd->last_upper_bounding_box);
    bbox_msg.pose.position.x = bbox_center.x();
    bbox_msg.pose.position.y = bbox_center.y();
    bbox_msg.pose.position.z = bbox_center.z();
    bbox_msg.pose.orientation.w = 1;
    bbox_msg.dimensions.x = bbox_size.x();
    bbox_msg.dimensions.y = bbox_size.y();
    bbox_msg.dimensions.z = bbox_size.z();
    publishers.bbox_pub.publish(bbox_msg);
}

void CDCPD_Moveit_Node::publish_template() const
{
    auto time = ros::Time::now();
    rope_configuration.tracked.points->header.frame_id = node_params.camera_frame;
    pcl_conversions::toPCL(time, rope_configuration.tracked.points->header.stamp);
    publishers.pre_template_publisher.publish(rope_configuration.tracked.points);
}

ObstacleConstraints CDCPD_Moveit_Node::get_obstacle_constraints()
{
    ObstacleConstraints obstacle_constraints;
    if (moveit_ready and node_params.moveit_enabled)
    {
        obstacle_constraints = get_moveit_obstacle_constriants(rope_configuration.tracked.points);
        ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Got moveit obstacle constraints");
    }
    return obstacle_constraints;
}

void CDCPD_Moveit_Node::publish_outputs(ros::Time const& t0, CDCPD::Output const& out)
{
    // Update the frame ids
    {
        out.original_cloud->header.frame_id = node_params.camera_frame;
        out.masked_point_cloud->header.frame_id = node_params.camera_frame;
        out.downsampled_cloud->header.frame_id = node_params.camera_frame;
        out.cpd_output->header.frame_id = node_params.camera_frame;
        out.gurobi_output->header.frame_id = node_params.camera_frame;
    }

    // Add timestamp information
    {
        auto time = ros::Time::now();
        pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.cpd_output->header.stamp);
        pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);
    }

    // Publish the point clouds
    {
        publishers.original_publisher.publish(out.original_cloud);
        publishers.masked_publisher.publish(out.masked_point_cloud);
        publishers.downsampled_publisher.publish(out.downsampled_cloud);
        publishers.template_publisher.publish(out.cpd_output);
        publishers.output_publisher.publish(out.gurobi_output);
    }

    // Publish markers indication the order of the points
    {
        auto rope_marker_fn = [&](PointCloud::ConstPtr cloud, std::string const& ns) {
            vm::Marker order;
            order.header.frame_id = node_params.camera_frame;
            order.header.stamp = ros::Time();
            order.ns = ns;
            order.type = visualization_msgs::Marker::LINE_STRIP;
            order.action = visualization_msgs::Marker::ADD;
            order.pose.orientation.w = 1.0;
            order.id = 1;
            order.scale.x = 0.01;
            order.color.r = 0.1;
            order.color.g = 0.6;
            order.color.b = 0.9;
            order.color.a = 1.0;

            for (auto pc_iter : *cloud)
            {
                geometry_msgs::Point p;
                p.x = pc_iter.x;
                p.y = pc_iter.y;
                p.z = pc_iter.z;
                order.points.push_back(p);
            }
            return order;
        };

        auto const rope_marker = rope_marker_fn(out.gurobi_output, "line_order");
        publishers.order_pub.publish(rope_marker);
    }

    // compute length and print that for debugging purposes
    auto output_length{0.0};
    for (auto point_idx{0}; point_idx < rope_configuration.tracked.points->size() - 1; ++point_idx)
    {
        Eigen::Vector3f const p =
          rope_configuration.tracked.points->at(point_idx + 1).getVector3fMap();
        Eigen::Vector3f const p_next =
          rope_configuration.tracked.points->at(point_idx).getVector3fMap();
        output_length += (p_next - p).norm();
    }
    ROS_DEBUG_STREAM_NAMED(LOGNAME + ".length", "length = " << output_length << " max length = "
        << node_params.max_rope_length);

    auto const t1 = ros::Time::now();
    auto const dt = t1 - t0;
    ROS_DEBUG_STREAM_NAMED(PERF_LOGGER, "dt = " << dt.toSec() << "s");
}

void CDCPD_Moveit_Node::reset_if_bad(CDCPD::Output const& out)
{
    if (out.status == OutputStatus::NoPointInFilteredCloud or
        out.status == OutputStatus::ObjectiveTooHigh)
    {
        // Recreate CDCPD from initial tracking.
        std::unique_ptr<CDCPD> cdcpd_new(new CDCPD(nh, ph, rope_configuration.initial.points,
            rope_configuration.initial.edges,
            cdcpd_params.objective_value_threshold, cdcpd_params.use_recovery, cdcpd_params.alpha,
            cdcpd_params.beta, cdcpd_params.lambda, cdcpd_params.k_spring, cdcpd_params.zeta,
            cdcpd_params.obstacle_cost_weight, cdcpd_params.fixed_points_weight));
        cdcpd = std::move(cdcpd_new);
    }
}

ConstraintPosDotIndices CDCPD_Moveit_Node::get_q_config()
{
    smmap::AllGrippersSinglePose q_config;
    std::vector<int> gripper_indices_vec;
    for (auto const gripper_constraint : gripper_constraints_) {
        auto const tf_name = gripper_constraint.frame_id;
        gripper_indices_vec.push_back(gripper_constraint.node_index);
        if (not tf_name.empty()) {
          try
          {
              auto const gripper = tf_buffer_.lookupTransform(node_params.camera_frame, tf_name,
                ros::Time(0), ros::Duration(10));
              auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
              ROS_DEBUG_STREAM_NAMED(LOGNAME + ".grippers", "gripper: " << config.translation());
              q_config.push_back(config);
          } catch (tf2::TransformException const& ex) {
              ROS_WARN_STREAM_THROTTLE(
                  10.0, "Unable to lookup transform from " << node_params.camera_frame << " to "
                    << tf_name << ": " << ex.what());
          }
        }
    }

    const smmap::AllGrippersSinglePoseDelta q_dot{gripper_constraints_.size(),
        kinematics::Vector6d::Zero()};

    Eigen::MatrixXi gripper_indices_eigen;
    gripper_indices_eigen.resize(1, gripper_indices_vec.size());
    for (auto const [i, gripper_idx] : enumerate(gripper_indices_vec)) {
    gripper_indices_eigen(0, i) = gripper_idx;
    }
    return ConstraintPosDotIndices{q_config, q_dot, gripper_indices_eigen};
}

bool CDCPD_Moveit_Node::set_gripper_constraints(cdcpd::SetGripperConstraints::Request& req,
        cdcpd::SetGripperConstraints::Response& res)
{
    gripper_constraints_ = req.constraints;
    return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");
  CDCPD_Moveit_Node cmn("hdt_michigan");
  return EXIT_SUCCESS;
}
