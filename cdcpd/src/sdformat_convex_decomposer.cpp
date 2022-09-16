// Flag for compiling VHACD header-only file.
// NOTE: ONLY NEEDS TO BE DONE ONCE!
#define ENABLE_VHACD_IMPLEMENTATION 1

#include "cdcpd/sdformat_convex_decomposer.h"

const char *lastDot(const char *str)
{
	const char *ret = nullptr;

	while ( *str )
	{
		if ( *str == '.' )
		{
			ret = str;
		}
		str++;
	}

	return ret;
}

SDFormatConvexDecomposer::SDFormatConvexDecomposer(std::string sdformat_filename_in,
    std::string sdformat_filename_out)
    : sdformat_filename_in_(sdformat_filename_in),
      sdformat_filename_out_(sdformat_filename_out)
{}

bool SDFormatConvexDecomposer::decompose()
{
    // Load the SDFormat file.
    auto sdf_element = std::make_shared<sdf::SDF>();
    sdf::init(sdf_element);

    if (!sdf::readFile(sdformat_filename_in_, sdf_element))
    {
        std::cerr << sdformat_filename_in_ << " is not a valid SDFormat file!" << std::endl;
        return false;
    }

    auto const root = sdf_element->Root();
    if (!root->HasElement("world"))
    {
        std::cerr << sdformat_filename_in_ << " the root element is now <world>" << std::endl;
        return false;
    }

    auto const world = root->GetElement("world");

    // Loop through all of the elements we need to get to a single mesh.
    for (auto const& model : ElementIterator(world, "model"))
    {
        const auto model_name = model->Get<std::string>("name");
        const auto model_pose = model->Get<ignition::math::Pose3d>("pose");
        std::cout << "model " << model_name << " pose: " << model_pose << std::endl;

        for (auto const& link : ElementIterator(model, "link"))
        {
            const auto link_name = link->Get<std::string>("name");
            const auto link_pose = link->Get<ignition::math::Pose3d>("pose");
            const auto link_collision = link->GetElement("collision");
            const auto link_collision_name = link_collision->Get<std::string>("name");
            std::cout << " link " << link_name << " pose: " << link_pose << std::endl;
            auto link_pose_in_world_frame = model_pose + link_pose;

            for (auto const& link_collision_geometry : ElementIterator(link_collision, "geometry"))
            {
                std::cout << "  link collision geometry " << std::endl;

                // for each collision geometry, convert it to the planning scene
                if (link_collision_geometry->HasElement("mesh"))
                {
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

                    for (auto i{0u}; i < scene->mNumMeshes; ++i)
                    {
                        auto mesh = scene->mMeshes[i];

                        // Get the mesh vertices in a form that works with VHACD
                        std::vector<VHACD::Vertex> mesh_vertices;
                        for (std::size_t vertex_idx = 0u; vertex_idx < mesh->mNumVertices; ++vertex_idx)
                        {
                            auto const& mesh_vert = mesh->mVertices[vertex_idx];
                            VHACD::Vertex vert{mesh_vert.x, mesh_vert.y, mesh_vert.z};
                            mesh_vertices.push_back(vert);
                        }

                        // Get the mesh faces in a form that works with VHACD
                        std::vector<VHACD::Triangle> mesh_triangles;
                        for (std::size_t face_idx = 0u; face_idx < mesh->mNumFaces; ++face_idx)
                        {
                            auto const& face = mesh->mFaces[face_idx];
                            VHACD::Triangle triangle{face.mIndices[0], face.mIndices[1], face.mIndices[2]};
                            mesh_triangles.push_back(triangle);
                        }

                        decompose_single_mesh(mesh_vertices, mesh_triangles);
                    }
                }
            }
        }
    }

    // Call the decompose_single_mesh function.
    return false;
}

// bool SDFormatConvexDecomposer::get_sdformat_file_world_element()
// {
//     return false;
// }

bool SDFormatConvexDecomposer::decompose_single_mesh(const std::vector<VHACD::Vertex>& verts, const std::vector<VHACD::Triangle>& triangles)
{


    // We'll need an array of points and an array of triangles here.

    // Convex decomposition.
    // This is for a single thread. VHACD does have the option for multithreading. We can
    // investigate this if this is determined to be a bottleneck.
    VHACD::IVHACD *iface = VHACD::CreateVHACD();

    // There's a LOT of configuration that can be done with these parameters.
    // Runs asynchronously by default.
    VHACD::IVHACD::Parameters params;
    // params.m_maxConvexHulls = 10;
    // params.m_maxConvexHulls = 2;

    bool canceled = false;
    {
        std::cout << "Computing Convex Decomposition" << std::endl;
        iface->Compute(verts, triangles, params);
        while ( !iface->IsReady() )
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
        }
    }
    if ( !canceled && iface->GetNConvexHulls() )
    {
        // Save the decomposition as a single STL file
        std::string baseName = sdformat_filename_in_.substr(0, sdformat_filename_in_.size() - 4);
        std::cout << "baseName: " << baseName << std::endl;

        {
            FILE *fph = fopen("temp.stl","wb");
            if ( fph )
            {
                printf("Saving convex hull results to a single file 'temp.stl'\n");
                for (uint32_t i=0; i<iface->GetNConvexHulls(); i++)
                {
                    VHACD::IVHACD::ConvexHull ch;
                    iface->GetConvexHull(i,ch);
                    uint32_t baseIndex = 1;
                    if ( fph )
                    {
                        char hullName[2048];
                        snprintf(hullName,sizeof(hullName),"%s%03d", baseName.c_str(), i);
                        fprintf(fph,"solid %s\n", hullName);
                        for (uint32_t j = 0; j < ch.m_triangles.size(); j++)
                        {
                            uint32_t i1 = ch.m_triangles[j].mI0;
                            uint32_t i2 = ch.m_triangles[j].mI1;
                            uint32_t i3 = ch.m_triangles[j].mI2;

                            const VHACD::Vertex& p1 = ch.m_points[i1];
                            const VHACD::Vertex& p2 = ch.m_points[i2];
                            const VHACD::Vertex& p3 = ch.m_points[i3];

                            double normal[3];
                            FLOAT_MATH::fm_computePlane((double*)&p1,
                                                        (double*)&p2,
                                                        (double*)&p3,
                                                        normal);
                            fprintf(fph," facet normal %0.9f %0.9f %0.9f\n", normal[0], normal[1], normal[2]);
                            fprintf(fph,"  outer loop\n");
                            fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p1.mX, p1.mY, p1.mZ);
                            fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p2.mX, p2.mY, p2.mZ);
                            fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p3.mX, p3.mY, p3.mZ);
                            fprintf(fph,"  endloop\n");
                            fprintf(fph," endfacet\n");
                        }
                        fprintf(fph,"endsolid %s\n", hullName);
                    }
                }
                fclose(fph);
            }
        }
    }

    iface->Release();

    return true;
}