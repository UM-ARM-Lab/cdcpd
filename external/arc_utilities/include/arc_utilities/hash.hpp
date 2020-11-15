#ifndef ARC_UTILITIES_HASH_HPP
#define ARC_UTILITIES_HASH_HPP

// http://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

namespace std
{

    template <typename T>
    struct hash<std::complex<T>>
    {
        std::size_t operator()(const std::complex<T>& val) const
        {
            return (std::hash<T>()(val.real()) ^ ((std::hash<T>()(val.imag()) << 1) >> 1));
        }
    };

    template <>
    struct hash<Eigen::Vector3d>
    {
        std::size_t operator()(const Eigen::Vector3d& vector) const
        {
            return (std::hash<double>()(vector.x()) ^ ((std::hash<double>()(vector.y()) << 1) >> 1) ^ (std::hash<double>()(vector.z()) << 1));
        }
    };

    // Hash function for Eigen vector.
    // Based on here: https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
    template<typename _Scalar, int _Rows>
    struct hash<Eigen::Matrix<_Scalar, _Rows, 1>>
    {
        std::size_t operator() (const Eigen::Matrix<_Scalar, _Rows, 1>& vector) const
        {
            std::size_t hash_val = 0;
            for (ssize_t idx = 0; idx < vector.size(); idx++)
            {
                hash_combine(hash_val, vector(idx));
            }
            return hash_val;
        }
    };

    template <typename T1, typename T2>
    struct hash<std::pair<T1, T2>>
    {
        std::size_t operator()(const std::pair<T1, T2>& val) const
        {
            std::size_t seed = 0;
            hash_combine(seed, val.first);
            hash_combine(seed, val.second);
            return seed;
        }
    };

}

#endif // ARC_UTILITIES_HASH_HPP
