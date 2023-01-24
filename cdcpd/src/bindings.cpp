#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pyrosmsg/pyrosmsg.h>

#include <cdcpd/cdcpd.h>
#include <cdcpd/tracking_map.h>
#include <cdcpd/deformable_object_configuration.h>

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
      ) // Constructor
    .def("run", &CDCPD::run);

  py::class_<CDCPDIterationInputs>(m, "CDCPDIterationInputs")
    .def(py::init<>())
    .def_readwrite("Y", &CDCPDIterationInputs::Y)
    .def_readwrite("Y_emit_prior", &CDCPDIterationInputs::Y_emit_prior)
    .def_readwrite("X", &CDCPDIterationInputs::X)
    .def_readwrite("obstacle_constraints", &CDCPDIterationInputs::obstacle_constraints)
    .def_readwrite("max_segment_length", &CDCPDIterationInputs::max_segment_length)
    .def_readwrite("pred_fixed_points", &CDCPDIterationInputs::pred_fixed_points)
    .def_readwrite("tracking_map", &CDCPDIterationInputs::tracking_map)
    // .def_readwrite("q_dot", &CDCPDIterationInputs::q_dot)
    // .def_readwrite("q_config", &CDCPDIterationInputs::q_config)
    .def_readwrite("pred_choice", &CDCPDIterationInputs::pred_choice);

  // How many of these do I actually have to expose? Right now I'm thinking close to all.
  py::class_<TrackingMap>(m, "TrackingMap");

  py::class_<DeformableObjectConfiguration>(m, "DeformableObjectConfiguration");

  py::class_<DeformableObjectTracking>(m, "DeformableObjectTracking");

  py::class_<RopeConfiguration>(m, "RopeConfiguration");

  py::class_<ClothConfiguration>(m, "ClothConfiguration");
}