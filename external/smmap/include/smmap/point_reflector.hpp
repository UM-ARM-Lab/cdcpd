#ifndef POINT_REFLECTOR_HPP
#define POINT_REFLECTOR_HPP

#include <Eigen/Dense>

namespace smmap
{
    class PointReflector
    {
        public:
            PointReflector()
                : PointReflector(0, 0, 0)
            {}

            PointReflector(double mid_x, double min_y, double max_y)
                : mid_x_(mid_x)
                , min_y_(min_y)
                , max_y_(max_y)
            {}

            Eigen::Vector3d reflect(const Eigen::Vector3d& in) const
            {
                Eigen::Vector3d out = in;
                out[0] = in[0] - 2*(in[0] - mid_x_);
                return out;
            }

            Eigen::Vector3d reflect(Eigen::Vector3d&& in) const
            {
                Eigen::Vector3d out = in;
                out[0] = in[0] - 2*(in[0] - mid_x_);
                return out;
            }

            double get_mid_x() const
            {
                return mid_x_;
            }

            double get_min_y() const
            {
                return min_y_;
            }

            double get_max_y() const
            {
                return max_y_;
            }

        private:
            const double mid_x_, min_y_, max_y_;
    };
}


#endif // POINT_REFLECTOR_HPP

