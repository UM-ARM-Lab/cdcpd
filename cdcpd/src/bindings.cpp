#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pyrosmsg/pyrosmsg.h>

#include <cdcpd/cdcpd.h>

namespace py = pybind11;

PYBIND11_MODULE(pycdcpd, m)
{
  m.doc() = "python bindings to cdcpd";
  py::class_<CDCPD>(m, "PyCDCPD")
      .def(py::init<PointCloud::ConstPtr,
               const Eigen::Matrix2Xi &,
               const bool,
               const double,
               const double,
               const double,
               const double,
               const float,
               const bool>(),
           py::arg("template_cloud"),
           py::arg("template_edges"),
           py::arg("use_recovery"),
           py::arg("alpha"),
           py::arg("beta"),
           py::arg("lambda"),
           py::arg("k"),
           py::arg("zeta"),
           py::arg("is_sim")
      );
}