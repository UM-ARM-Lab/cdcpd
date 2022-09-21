#include "cdcpd/cdcpd_node.h"

// std::string const LOGNAME = "cdcpd_node";
// constexpr auto const MAX_CONTACTS_VIZ = 25;
// constexpr auto const PERF_LOGGER = "perf";

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
    scene_pub = ph.advertise<moveit_msgs::PlanningScene>("scene", 10);
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
      moveit_enabled(ROSHelpers::GetParam<bool>(ph, "moveit_enabled", false)),
      sdf_filename(ROSHelpers::GetParam<std::string>(ph, "sdf_filename", ""))
{}


CDCPD_Moveit_Node::CDCPD_Moveit_Node(std::string const& robot_namespace)
    : ph("~"),
      publishers(nh, ph),
      node_params(nh, ph),
      cdcpd_params(ph),
      rope_configuration(node_params.num_points, node_params.max_rope_length),
      tf_listener_(tf_buffer_),
      obstacle_constraint_helper_(node_params.moveit_enabled, publishers.contact_marker_pub,
                                  publishers.scene_pub, node_params.camera_frame,
                                  node_params.sdf_filename, cdcpd_params.min_distance_threshold)
{
    // For use with TF and "fixed points" for the constrain step
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Using frame " << node_params.camera_frame);

    ROS_INFO_STREAM_NAMED(LOGNAME, "Loading " << node_params.grippers_info_filename);
    grippers_info = YAML::LoadFile(node_params.grippers_info_filename);
    gripper_count = grippers_info.size();
    int gripper_idx = 0;
    gripper_indices(1, gripper_count);
    for (auto const gripper_info_i : grippers_info) {
        auto const tf_name = gripper_info_i.first.as<std::string>();
        auto const node_idx = gripper_info_i.second.as<int>();
        gripper_indices(0, gripper_idx) = node_idx;
        gripper_idx++;
    }
    if (gripper_count == 0) {
        gripper_indices = {};
    }

    // Initial connectivity model of rope
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "max segment length " << rope_configuration.max_segment_length);

    // initialize start and end points for rope
    auto [init_q_config, init_q_dot] = get_q_config();
    Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    end_position[2] += node_params.max_rope_length;
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

void CDCPD_Moveit_Node::callback(cv::Mat const& rgb, cv::Mat const& depth,
    cv::Matx33d const& intrinsics)
{
    auto const t0 = ros::Time::now();
    auto [q_config, q_dot] = get_q_config();

    publish_bbox();

    auto obstacle_constraints = obstacle_constraint_helper_.get_obstacle_constraints(
        rope_configuration.tracked.points);

    auto const hsv_mask = getHsvMask(ph, rgb);
    auto const out = (*cdcpd)(rgb, depth, hsv_mask, intrinsics,
                              rope_configuration.tracked.points,
                              obstacle_constraints, rope_configuration.max_segment_length, q_dot,
                              q_config, gripper_indices);
    rope_configuration.tracked.points = out.gurobi_output;
    publish_outputs(t0, out);
    reset_if_bad(out);
};

void CDCPD_Moveit_Node::points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    auto const t0 = ros::Time::now();
    auto [q_config, q_dot] = get_q_config();

    publish_bbox();
    publish_template();
    auto obstacle_constraints = obstacle_constraint_helper_.get_obstacle_constraints(
        rope_configuration.tracked.points);

    pcl::PCLPointCloud2 points_v2;
    pcl_conversions::toPCL(*points_msg, points_v2);
    auto points = boost::make_shared<PointCloudRGB>();
    pcl::fromPCLPointCloud2(points_v2, *points);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "unfiltered points: " << points->size());

    auto const out = (*cdcpd)(points, rope_configuration.tracked.points, obstacle_constraints,
        rope_configuration.max_segment_length, q_dot, q_config, gripper_indices);
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

std::tuple<smmap::AllGrippersSinglePose,
           const smmap::AllGrippersSinglePoseDelta> CDCPD_Moveit_Node::get_q_config()
{
    smmap::AllGrippersSinglePose q_config;
    for (auto const gripper_info_i : grippers_info) {
        auto const tf_name = gripper_info_i.first.as<std::string>();
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

    const smmap::AllGrippersSinglePoseDelta q_dot{gripper_count, kinematics::Vector6d::Zero()};

    return std::tuple{q_config, q_dot};
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");
  CDCPD_Moveit_Node cmn("hdt_michigan");
  return EXIT_SUCCESS;
}
