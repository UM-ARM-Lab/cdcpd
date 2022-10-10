#include "cdcpd/corner_candidate_detector.h"

CornerCandidateDetection::CornerCandidateDetection(int const detection_id,
    Eigen::Vector3f const corner_point_camera_frame,
    cv::Mat const template_affine_transform,
    std::vector<Eigen::Vector3f> const local_cloud_neighborhood_bounds,
    std::vector<Eigen::Vector3f> const detection_mask_bounds)
        : id_(detection_id),
          corner_point_camera_frame_(corner_point_camera_frame),
          template_affine_transform_(template_affine_transform),
          local_cloud_neighborhood_bounds_(local_cloud_neighborhood_bounds),
          detection_mask_bounds_(detection_mask_bounds)
{}

// Do a bounding box filter to get the local neighborhood.
boost::shared_ptr<PointCloudRGB> CornerCandidateDetection::get_local_point_cloud_neighborhood(boost::shared_ptr<PointCloudRGB> point_cloud_full) const
{
    auto points_cropped = boost::make_shared<PointCloudRGB>();

    // Find points closest and furthest from the origin (camera frame).
    Eigen::Vector3f pt_min;
    float pt_min_norm = 1e15;
    Eigen::Vector3f pt_max;
    float pt_max_norm = -1e15;
    for (auto const pt : local_cloud_neighborhood_bounds_)
    {
        float pt_norm = pt.norm();

        if (pt_norm < pt_min_norm)
        {
            pt_min = pt;
            pt_min_norm = pt_norm;
        }

        if (pt_norm > pt_max_norm)
        {
            pt_max = pt;
            pt_max_norm = pt_norm;
        }
    }
    Eigen::Vector3f bounding_box_z_extend(0, 0, .050);
    Eigen::Vector4f box_min = (pt_min - bounding_box_z_extend).homogeneous();
    Eigen::Vector4f box_max = (pt_max + bounding_box_z_extend).homogeneous();

    ROS_INFO_STREAM("Candidate ID#" << id_ << " local neighborhood bounding box min:" << box_min);
    ROS_INFO_STREAM("Candidate ID#" << id_ << " local neighborhood bounding box max:" << box_max);

    pcl::CropBox<PointRGB> box_filter;
    box_filter.setMin(box_min);
    box_filter.setMax(box_max);
    box_filter.setInputCloud(point_cloud_full);
    box_filter.filter(*points_cropped);

    ROS_INFO_STREAM("Candidate ID#" << id_ << " local neighborhood num points: " << points_cropped->size());
    ROS_INFO_STREAM("Candidate ID#" << id_ << " local neighborhood header: " << points_cropped->header);

    return points_cropped;
}

// Do a bounding box filter to get the masked points.
boost::shared_ptr<PointCloud> CornerCandidateDetection::get_masked_points(boost::shared_ptr<PointCloudRGB> point_cloud_full) const
{
    auto points_cropped_rgb = boost::make_shared<PointCloudRGB>();
    auto points_cropped = boost::make_shared<PointCloud>();

    // Find points closest and furthest from the origin (camera frame).
    Eigen::Vector3f pt_min;
    float pt_min_norm = 1e15;
    Eigen::Vector3f pt_max;
    float pt_max_norm = -1e15;
    for (auto const pt : detection_mask_bounds_)
    {
        float pt_norm = pt.norm();

        if (pt_norm < pt_min_norm)
        {
            pt_min = pt;
            pt_min_norm = pt_norm;
        }

        if (pt_norm > pt_max_norm)
        {
            pt_max = pt;
            pt_max_norm = pt_norm;
        }
    }
    Eigen::Vector3f bounding_box_z_extend(0, 0, .050);
    Eigen::Vector4f box_min = (pt_min - bounding_box_z_extend).homogeneous();
    Eigen::Vector4f box_max = (pt_max + bounding_box_z_extend).homogeneous();

    ROS_INFO_STREAM("Candidate ID#" << id_ << " mask bounding box min:" << box_min);
    ROS_INFO_STREAM("Candidate ID#" << id_ << " mask bounding box max:" << box_max);

    pcl::CropBox<PointRGB> box_filter;
    box_filter.setMin(box_min);
    box_filter.setMax(box_max);
    box_filter.setInputCloud(point_cloud_full);
    box_filter.filter(*points_cropped_rgb);

    pcl::copyPointCloud(*points_cropped_rgb, *points_cropped);

    ROS_INFO_STREAM("Candidate ID#" << id_ << " mask num points: " << points_cropped->size());
    ROS_INFO_STREAM("Candidate ID#" << id_ << " mask header: " << points_cropped->header);

    return points_cropped;
}

void CornerCandidateDetection::print() const
{
    ROS_INFO_STREAM("Printing corner candidate ID #" << id_);
    ROS_INFO_STREAM("\tCorner point camera frame: " << corner_point_camera_frame_);
    ROS_INFO_STREAM("\tAffine transform: " << template_affine_transform_);

    ROS_INFO("\tLocal cloud neighborhood bounds:");
    print_vector(local_cloud_neighborhood_bounds_);

    ROS_INFO_STREAM("\tDetection mask bounds:");
    print_vector(detection_mask_bounds_);
}

void CornerCandidateDetection::print_vector(std::vector<Eigen::Vector3f> vec_of_vecs) const
{
    std::stringstream output;
    for (auto const& vec : vec_of_vecs)
    {
        output << "\t\t" << vec << std::endl;
    }
    ROS_INFO(output.str().c_str());
}

CornerCandidateDetector::CornerCandidateDetector(){}

std::vector<CornerCandidateDetection> CornerCandidateDetector::do_corner_candidate_detection(
    const PointCloudRGB::Ptr &points_full_cloud)
{
    auto t_start = ros::Time::now();

    // Write the point cloud for the segmentation routine to use.
    // Might not even have to write the point cloud file.
    write_point_cloud(points_full_cloud);
    ROS_INFO("Finished saving point cloud");

    // Call the corner candidate detection script/routine
    run();

    auto candidate_detections = read_detector_output();

    auto t_end = ros::Time::now();

    // Outputting time of execution
    {

        auto t_delta = t_end - t_start;
        double num_secs = static_cast<double>(t_delta.nsec) / 1e9;
        std::stringstream msg;
        msg << "Took " << num_secs << " seconds to execute corner candidate detection routines";
        ROS_INFO(msg.str().c_str());
    }

    return candidate_detections;
}

std::vector<CornerCandidateDetection> CornerCandidateDetector::do_corner_candidate_detection(
        cv::Mat const& rgb_img, cv::Mat const& depth_img)
{
    auto t_start = ros::Time::now();

    // Write the RGB and Depth images for the segmentation routine to use.
    // Might not even have to write the point cloud file.
    ROS_INFO("Finished saving point cloud");
    write_cv_mat(rgb_output_path_, rgb_img);
    write_cv_mat(depth_output_path_, depth_img);

    // Call the corner candidate detection script/routine.
    run();

    auto candidate_detections = read_detector_output();


    auto t_end = ros::Time::now();

    // Outputting time of execution
    {

        auto t_delta = t_end - t_start;
        double num_secs = static_cast<double>(t_delta.nsec) / 1e9;
        std::stringstream msg;
        msg << "Took " << num_secs << " seconds to execute corner candidate detection routines";
        ROS_INFO(msg.str().c_str());
    }

    return candidate_detections;
}

std::vector<std::tuple<int const, int const>>
    CornerCandidateDetector::associate_corner_candidates_with_tracked_objects(
        std::vector<CornerCandidateDetection> const& corner_candidate_detections,
        std::map<int const, std::shared_ptr<DeformableObjectConfiguration>> deformable_object_configurations)
{
    std::vector<std::tuple<int const, int const>> associated_pairs;

    // Could just do something really dumb at first and find minimum distance between each tracked
    // object and the clusters
    int candidate_idx = 0;
    for (auto const& candidate : corner_candidate_detections)
    {
        double min_dist = 1e15;
        int min_dist_def_obj_id = -1;  // Initialize to invalid value.
        bool association_made = false;
        for (auto const& def_obj_pair : deformable_object_configurations)
        {
            // Should probably take centroid or something.
            auto const& def_obj_pnt = def_obj_pair.second->tracked_.vertices_.col(0);

            float const dist = (candidate.corner_point_camera_frame_ - def_obj_pnt).norm();

            // Calculate distance between candidate centroid and tracked_object_centroid
            min_dist_def_obj_id = dist < min_dist ? def_obj_pair.first : min_dist_def_obj_id;
            min_dist = dist < min_dist ? dist : min_dist;
        }

        if (min_dist > 100.0F)  // [mm]
        {
            min_dist_def_obj_id = -1;
        }

        // Make the tuple and store in the associated_pairs vector
        associated_pairs.push_back({candidate_idx, min_dist_def_obj_id});

        ++candidate_idx;
    }

    ROS_INFO("Need to iterate through def obj configurations to return those not seen this iteration.");

    return associated_pairs;
}

void CornerCandidateDetector::run()
{
    std::string cmd = "python3 /home/dcolli23/code/lab/iros_cloth_classic/launcher_interface.py";
    system(cmd.c_str());
    ROS_INFO("Finished running python script!");
}

void CornerCandidateDetector::write_point_cloud(const PointCloudRGB::Ptr &point_cloud)
{
    pcl::io::savePCDFile(full_cloud_output_path_, *point_cloud);
}

void CornerCandidateDetector::write_cv_mat(std::string const& output_path, cv::Mat const& img)
{
    cv::imwrite(output_path, img);
}

std::vector<CornerCandidateDetection> CornerCandidateDetector::read_detector_output()
{
    std::vector<CornerCandidateDetection> candidate_detections;
    // Read the YAML file output by the corner detection script (saved in a pre-determined location)
    // This contains the information in an array for all corner candidate detections.
    YAML::Node root_node = YAML::LoadFile(yaml_output_path_)["corner_candidate_detections"];
    assert(root_node.IsSequence());

    // Loop through the corner candidate information array
    for (size_t det_num = 0; det_num < root_node.size(); ++det_num)
    {
        YAML::Node det_node = root_node[det_num];
        // {
        //     std::stringstream outmsg;
        //     outmsg << "det_node type: " << det_node.Type();
        //     ROS_INFO(outmsg.str().c_str());
        // }

        // Read the corner point in the camera frame.
        YAML::Node const& cam_point_node = det_node["corner_point_camera_frame"];

        Eigen::Vector3f corner_point_camera_frame;
        for (size_t i = 0; i < cam_point_node.size(); ++i)
        {
            corner_point_camera_frame(i) = cam_point_node[i].as<float>();
        }
        // {
        //     std::stringstream outmsg;
        //     outmsg << "Point: " << corner_point_camera_frame;
        //     ROS_INFO(outmsg.str().c_str());
        // }

        // Read the local point cloud neighborhood.
        std::vector<Eigen::Vector3f> local_point_cloud_bounds;
        YAML::Node const& local_node = det_node["local_bbox_coordinates_camera_frame"];
        for (size_t i = 0; i < local_node.size(); ++i)
        {
            Eigen::Vector3f local_bound;
            local_bound(0) = local_node[i][0].as<float>();
            local_bound(1) = local_node[i][1].as<float>();
            local_bound(2) = local_node[i][2].as<float>();
            local_point_cloud_bounds.push_back(local_bound);
        }


        // Read the corner candidate mask
        std::vector<Eigen::Vector3f> mask_bounds;
        YAML::Node const& mask_node = det_node["mask_bbox_coordinates_camera_frame"];
        for (size_t i = 0; i < mask_node.size(); ++i)
        {
            Eigen::Vector3f mask_bound;
            mask_bound(0) = mask_node[i][0].as<float>();
            mask_bound(1) = mask_node[i][1].as<float>();
            mask_bound(2) = mask_node[i][2].as<float>();
            mask_bounds.push_back(mask_bound);
        }

        // Convert the template_to_corner_candidate_affine_transform into cv::Mat variable
        Eigen::Matrix<float, 4, 4> affine_transform_eigen;
        YAML::Node const& transform_node = det_node["P_corner_to_camera"];
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 0; j < 4; ++j)
            {
                affine_transform_eigen(i, j) = transform_node[i][j].as<float>();
            }
        }
        cv::Mat affine_transform;
        cv::eigen2cv(affine_transform_eigen, affine_transform);


        // CornerCandidateDetection candidate(corner_point_camera_frame, affine_transform,
        //     local_point_cloud_bounds, mask_bounds);

        // Initialize a new CornerCandidateDetection with the read variables and store the
        // candidate detection in our vector.
        candidate_detections.push_back({det_num, corner_point_camera_frame, affine_transform,
            local_point_cloud_bounds, mask_bounds});
    }

    return candidate_detections;
}