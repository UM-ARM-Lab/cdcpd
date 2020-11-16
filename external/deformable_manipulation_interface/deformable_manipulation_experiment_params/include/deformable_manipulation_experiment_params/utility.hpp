#pragma once

#include <string>
#include <type_traits>
#include <cstdint>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/ros_helpers.hpp>

namespace smmap
{
    // Desiged to work with any integer type, not floats
    template<typename T>
    inline int GetNumberOfDigits(T i)
    {
        // Safety check on the type we've been called with
        static_assert(std::is_same<T, uint8_t>::value
                      || std::is_same<T, uint16_t>::value
                      || std::is_same<T, uint32_t>::value
                      || std::is_same<T, uint64_t>::value
                      || std::is_same<T, int8_t>::value
                      || std::is_same<T, int16_t>::value
                      || std::is_same<T, int32_t>::value
                      || std::is_same<T, int64_t>::value,
                      "Type must be a fixed-size integral type");
        if (i < 0)
        {
            i = -i;
        }
        return i > 0 ? (int)std::log10((double)i) + 1 : 1;
    }

    inline int PressAnyKeyToContinue(const std::string& message = "Press any key to continue ")
    {
        if (!arc_helpers::IsDebuggerPresent())
        {
            std::cout << message << std::flush;
            auto key = arc_helpers::GetChar();
            if (key != '\n')
            {
                std::cout << std::endl;
            }
            return key;
        }
        else
        {
            std::cout << "Process is under debugger, use breakpoints for "
                      << "interactive flow control instead. Message: "
                      << message << std::endl;
            return '\n';
        }
    }

    template<typename T>
    std::string IntToHex(const T i)
    {
        std::stringstream stream;
        stream << std::setfill ('0') << std::setw(sizeof(T) * 2)
               << std::hex << i;
        return stream.str();
    }

    inline std_msgs::ColorRGBA GetColorFromParamSever(
            ros::NodeHandle& nh,
            const std::string& base_name)
    {
        using namespace ROSHelpers;
        float r, g, b, a = 1.0;
        try
        {
            r = GetParamOptional<float>(nh, base_name + "_r", __func__).GetImmutable();
            g = GetParamOptional<float>(nh, base_name + "_g", __func__).GetImmutable();
            b = GetParamOptional<float>(nh, base_name + "_b", __func__).GetImmutable();
            a = GetParam<float>(nh, base_name + "_a", 1.0f);
        }
        catch (const std::invalid_argument& /* ex */)
        {
            const auto stdvec = GetVectorRequired<float>(nh, base_name, __func__);
            switch(stdvec.size())
            {
                case 4:
                    a = stdvec[3];
                    [[fallthrough]];
                case 3:
                    r = stdvec[0];
                    g = stdvec[1];
                    b = stdvec[2];
                    break;

                default:
                    throw_arc_exception(std::invalid_argument, base_name + " must be 3 or 4 elements; length is " + std::to_string(stdvec.size()));
            }
        }

        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    inline Eigen::Vector3d GetVector3FromParamServer(
            ros::NodeHandle& nh,
            const std::string& base_name)
    {
        using namespace ROSHelpers;
        try
        {
            return Eigen::Vector3d(
                        GetParamOptional<double>(nh, base_name + "_x", __func__).GetImmutable(),
                        GetParamOptional<double>(nh, base_name + "_y", __func__).GetImmutable(),
                        GetParamOptional<double>(nh, base_name + "_z", __func__).GetImmutable());
        }
        catch (const std::invalid_argument& /* ex */)
        {
            const auto stdvec = GetVectorRequired<double>(nh, base_name, __func__);
            if (stdvec.size() != 3)
            {
                throw_arc_exception(std::invalid_argument, "Parameter is the wrong size; length is " + std::to_string(stdvec.size()));
            }
            return Eigen::Vector3d(stdvec[0], stdvec[1], stdvec[2]);
        }
    }

    inline Eigen::Quaterniond GetQuaternionFromParamServer(
            ros::NodeHandle& nh,
            const std::string& base_name)
    {
        using namespace ROSHelpers;
        try
        {
            return Eigen::Quaterniond(
                        GetParamOptional<double>(nh, base_name + "_w", __func__).GetImmutable(),
                        GetParamOptional<double>(nh, base_name + "_x", __func__).GetImmutable(),
                        GetParamOptional<double>(nh, base_name + "_y", __func__).GetImmutable(),
                        GetParamOptional<double>(nh, base_name + "_z", __func__).GetImmutable());
        }
        catch (const std::invalid_argument& /* ex */)
        {
            const auto stdvec = GetVectorRequired<double>(nh, base_name, __func__);
            if (stdvec.size() != 4)
            {
                throw_arc_exception(std::invalid_argument, "Parameter is the wrong size; length is " + std::to_string(stdvec.size()));
            }
            // Assumed order on the parameter server is (x, y, z, w)
            // Eigen takes the inputs as (w, x, y, z) (and then stores internally as (x, y, z, w))
            return Eigen::Quaterniond(stdvec[3], stdvec[0], stdvec[1], stdvec[2]);
        }

    }

    inline Eigen::Isometry3d GetPoseFromParamServer(
            ros::NodeHandle& nh,
            const std::string& base_name,
            const bool rotation_optional = true)
    {
        const Eigen::Translation3d trans(GetVector3FromParamServer(nh, base_name + "_pos"));
        try
        {
            const Eigen::Quaterniond quat = GetQuaternionFromParamServer(nh, base_name + "_quat");
            return trans * quat;
        }
        catch (const std::invalid_argument& ex)
        {
            if (!rotation_optional)
            {
                throw;
            }
            (void)ex;
            return Eigen::Isometry3d(trans);
        }
    }

    template <typename IntType>
    inline std::string ToStrFill0(const IntType num_trials, const IntType trial_idx)
    {
        static_assert(std::is_integral<IntType>::value, "Inegral type required");
        const auto num_digits = [&]
        {
            assert(num_trials >= 0);
            assert(num_trials < 10000000000);
            return (num_trials < 10 ? 1 :
                   (num_trials < 100 ? 2 :
                   (num_trials < 1000 ? 3 :
                   (num_trials < 10000 ? 4 :
                   (num_trials < 100000 ? 5 :
                   (num_trials < 1000000 ? 6 :
                   (num_trials < 10000000 ? 7 :
                   (num_trials < 100000000 ? 8 :
                   (num_trials < 1000000000 ? 9 :
                   10)))))))));
        }();

        std::stringstream ss;
        ss << std::setw(num_digits) << std::setfill('0') << trial_idx;
        return ss.str();
    }

    template <typename Derived>
    inline WriteToCSVFile(const std::string filename, const Eigen::MatrixBase<Derived>& data)
    {
        const static Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream file(filename);
        file << data.matrix().format(csv_format);
    }
}
