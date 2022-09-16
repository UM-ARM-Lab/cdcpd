#pragma once

#include <string>

#include "v-hacd/VHACD.h"
#include "v-hacd/FloatMath.h"
#include <sdformat-9.8/sdf/sdf.hh>

#include "cdcpd/sdformat_to_planning_scene.h"

// Preprocesses a concave SDFormat mesh
class SDFormatConvexDecomposer
{
public:
    SDFormatConvexDecomposer(std::string sdformat_filename_in, std::string sdformat_filename_out);

    bool decompose();



private:
    bool get_sdformat_file_world_element(std::shared_ptr<sdf::SDF> sdf_element);

    bool decompose_single_mesh(std::vector<VHACD::Vertex> const& verts, std::vector<VHACD::Triangle> const& triangles);

    std::string sdformat_filename_in_;
    std::string sdformat_filename_out_;
};