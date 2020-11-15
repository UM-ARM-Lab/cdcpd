#include <ros/ros.h>
#include <arc_utilities/eigen_helpers.hpp>

namespace smmap
{
    uint64_t SerializePointSet(
            const std::string& frame,
            const EigenHelpers::VectorVector3d& points,
            std::vector<uint8_t>& buffer);

    std::pair<std::pair<std::string, EigenHelpers::VectorVector3d>, uint64_t> DeserializePointSet(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    void SerializeAndStorePointSet(const std::string& frame,
            const EigenHelpers::VectorVector3d& points,
            const std::string& folder_name,
            const std::string& file_name);

    std::pair<std::string, EigenHelpers::VectorVector3d> LoadAndDeserializePointSet(
            const std::string& folder_name,
            const std::string& file_name);
}
