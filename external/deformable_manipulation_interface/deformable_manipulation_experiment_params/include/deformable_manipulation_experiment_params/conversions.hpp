#pragma once

#include <memory>
#include <vector>

namespace smmap
{
    template<typename T>
    inline std::vector<std::shared_ptr<const T>> ToConstPtr(
            const std::vector<std::shared_ptr<T>>& vec)
    {
        std::vector<std::shared_ptr<const T>> ret;
        ret.reserve(vec.size());
        for (const auto& ptr : vec)
        {
            ret.push_back(ptr);
        }
        return ret;
    }
}
