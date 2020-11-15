#include "deformable_manipulation_experiment_params/serialization.h"

#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/zlib_helpers.hpp>

using namespace arc_utilities;

namespace smmap
{
    uint64_t SerializePointSet(
            const std::string& frame,
            const EigenHelpers::VectorVector3d& points,
            std::vector<uint8_t>& buffer)
    {
        // Serialize the frame
        const size_t starting_size = buffer.size();
        SerializeString(frame, buffer);

        // Serialize the point vector
        const std::function<uint64_t(const Eigen::Vector3d&, std::vector<uint8_t>&)> value_serializer =
                [] (const Eigen::Vector3d& value, std::vector<uint8_t>& buffer)
        {
            return SerializeEigen(value, buffer);
        };
        SerializeVector(points, buffer, value_serializer);

        // Determine how many bytes were written
        const size_t bytes_written = buffer.size() - starting_size;
        return bytes_written;
    }

    std::pair<std::pair<std::string, EigenHelpers::VectorVector3d>, uint64_t> DeserializePointSet(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        size_t current_position = current;

        // Read in the frame
        const auto frame_deserialized = arc_utilities::DeserializeString<char>(buffer, current_position);
        const std::string& frame = frame_deserialized.first;
        current_position += frame_deserialized.second;
        const auto value_deserializer = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return arc_utilities::DeserializeEigen<Eigen::Vector3d>(buffer, current);
        };

        // Read in the point vector
        const auto points_deserialized =
                DeserializeVector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>(
                    buffer, current_position, value_deserializer);
        const EigenHelpers::VectorVector3d& points = points_deserialized.first;
        current_position += points_deserialized.second;

        // Assemble and return the results
        const size_t bytes_read = current_position - current;
        return {{frame, points}, bytes_read};
    }

    void SerializeAndStorePointSet(
            const std::string& frame,
            const EigenHelpers::VectorVector3d& points,
            const std::string& folder_name,
            const std::string& file_name)
    {
        try
        {
            ROS_DEBUG_NAMED("dmi_serialization", "Serializing data");
            std::vector<uint8_t> buffer;
            SerializePointSet(frame, points, buffer);

            // Compress and save to file
            ROS_DEBUG_NAMED("dmi_serialization", "Compressing and writing data");
            const std::string file_path = folder_name + "/" + file_name;
            CreateDirectory(folder_name);
            ZlibHelpers::CompressAndWriteToFile(buffer, file_path);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM_NAMED("dmi_serialization", "Saving point set data to file failed: " << e.what());
        }
    }

    std::pair<std::string, EigenHelpers::VectorVector3d> LoadAndDeserializePointSet(
            const std::string& folder_name,
            const std::string& file_name)
    {
        const std::string file_path = folder_name + "/" + file_name;
        ROS_DEBUG_STREAM_NAMED("dmi_serialization", "Reading contents of file " << file_path << " and decompressing");
        const std::vector<uint8_t> decompressed_perception_data = ZlibHelpers::LoadFromFileAndDecompress(file_path);

        ROS_DEBUG_NAMED("dmi_serialization", "Deserializing data");
        const auto deserialized_results = DeserializePointSet(decompressed_perception_data, 0);
        const auto deserialized_bytes_read = deserialized_results.second;
        if (deserialized_bytes_read != decompressed_perception_data.size())
        {
            throw_arc_exception(std::runtime_error, "deserialization error, read bytes does not match expected bytes");
        }
        return deserialized_results.first;
    }
}
