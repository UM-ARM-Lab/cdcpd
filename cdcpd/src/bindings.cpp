#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pyrosmsg/pyrosmsg.h>

#include <cdcpd/cdcpd.h>
#include <cdcpd/cpd.h>
#include <cdcpd/tracking_map.h>
#include <cdcpd/deformable_object_configuration.h>

namespace py = pybind11;

PYBIND11_MODULE(pycdcpd, m)
{
  m.doc() = "Python bindings to CDCPD";

  // Somewhat forward declaration to avoid having C++ type names in the generated Python docs.
  auto pyCDCPD = py::class_<CDCPD>(m, "PyCDCPD");
  auto pyCDCPDOutput = py::class_<CDCPD::Output>(m, "CDCPDOutput");
  auto pyCDCPDIterationInputs = py::class_<CDCPDIterationInputs>(m, "CDCPDIterationInputs");
  auto pyTrackingMap = py::class_<TrackingMap>(m, "TrackingMap");
  auto pyDeformableObjectConfiguration =
    py::class_<DeformableObjectConfiguration,
               std::shared_ptr<DeformableObjectConfiguration> >(m, "DeformableObjectConfiguration");
  auto pyRopeConfiguration =
    py::class_<RopeConfiguration,
               DeformableObjectConfiguration,
               std::shared_ptr<RopeConfiguration> >(m, "RopeConfiguration");
  auto pyDeformableObjectTracking =
    py::class_<DeformableObjectTracking>(m, "DeformableObjectTracking");
  auto pyCPDInterface = py::class_<CPDInterface, std::shared_ptr<CPDInterface> >(m, "CPDInterface");
  auto pyCPD = py::class_<CPD, CPDInterface, std::shared_ptr<CPD> >(m, "CPD");
  auto pyCPDMultiTemplate =
    py::class_<CPDMultiTemplate,
               CPDInterface,
               std::shared_ptr<CPDMultiTemplate> >(m, "CPDMultiTemplate");
  auto pyCPDMultiTemplateMahalanobis =
    py::class_<CPDMultiTemplateMahalanobis,
               CPDMultiTemplate,
               std::shared_ptr<CPDMultiTemplateMahalanobis> >(m, "CPDMultiTemplateMahalanobis");
  auto pyCPDMultiTemplateExternalPointAssignment =
    py::class_<CPDMultiTemplateExternalPointAssignment,
               CPDMultiTemplate,
               std::shared_ptr<CPDMultiTemplateExternalPointAssignment> >(
                 m,
                 "CPDMultiTemplateExternalPointAssignment"
               );

  pyCDCPD.def(py::init<TrackingMap const&,
                       const float,
                       const bool,
                       const double,
                       const double,
                       const double,
                       const double,
                       const float,
                       const float,
                       const float>(),
              py::arg("tracking_map"),
              py::arg("objective_value_threshold"),
              py::arg("use_recovery"),
              py::arg("alpha"),
              py::arg("beta"),
              py::arg("lambda_val"),
              py::arg("k"),
              py::arg("zeta"),
              py::arg("obstacle_cost_weight"),
              py::arg("fixed_points_weight")
      ) // Constructor
    .def("run", &CDCPD::run)
    // .def_readonly("m_lle_", &CDCPD::m_lle_)
    .def_readwrite("cpd_runner", &CDCPD::cpd_runner_)
    ;

  pyCDCPDOutput.def("get_cpd_output", &CDCPD::Output::get_cpd_output)
    .def("get_gurobi_output", &CDCPD::Output::get_gurobi_output)
    ;

  pyCDCPDIterationInputs.def(py::init<>())
    .def_readwrite("Y_emit_prior", &CDCPDIterationInputs::Y_emit_prior)
    .def_readwrite("X", &CDCPDIterationInputs::X)
    .def_readwrite("obstacle_constraints", &CDCPDIterationInputs::obstacle_constraints)
    .def_readwrite("pred_fixed_points", &CDCPDIterationInputs::pred_fixed_points)
    .def_readwrite("tracking_map", &CDCPDIterationInputs::tracking_map)
    .def_readwrite("pred_choice", &CDCPDIterationInputs::pred_choice);

  pyTrackingMap.def(py::init<>())
    .def("add_def_obj_configuration", &TrackingMap::add_def_obj_configuration)
    .def("update_def_obj_vertices_from_mat", &TrackingMap::update_def_obj_vertices_from_mat)
    .def("form_vertices_cloud", &TrackingMap::form_vertices_cloud)
    .def("form_edges_matrix", &TrackingMap::form_edges_matrix)
    .def("get_total_num_points", &TrackingMap::get_total_num_points)
    .def_readonly("tracking_map", &TrackingMap::tracking_map)
    ;

  // This is an abstract class so we don't need to define the constructor.
  pyDeformableObjectConfiguration
    .def_readwrite("num_points_", &DeformableObjectConfiguration::num_points_)
    .def_readwrite("max_segment_length_", &DeformableObjectConfiguration::max_segment_length_)
    .def_readwrite("tracked_", &DeformableObjectConfiguration::tracked_)
    .def_readwrite("initial_", &DeformableObjectConfiguration::initial_);

  pyRopeConfiguration
    .def(py::init<int const, float const, Eigen::Vector3f const, Eigen::Vector3f const>(),
      py::arg("num_points_"),
      py::arg("max_rope_length_"),
      py::arg("rope_start_position"),
      py::arg("rope_end_position"))
    .def("initializeTracking", &RopeConfiguration::initializeTracking)
    .def_readonly("max_rope_length_", &RopeConfiguration::max_rope_length_)
    .def_readonly("rope_start_position_initial_", &RopeConfiguration::rope_start_position_initial_)
    .def_readonly("rope_end_position_initial_", &RopeConfiguration::rope_end_position_initial_)
    ;

  // py::class_<ClothConfiguration>(m, "ClothConfiguration");

  pyDeformableObjectTracking
    .def(py::init<>())

    // To do overloading, you have to cast the methods to function pointers.
    // .def_property("vertices_",
    //   py::overload_cast<std::vector<cv::Point3f> const&>(&DeformableObjectTracking::setVertices),
    //    &DeformableObjectTracking::getVertices)
    // .def_readonly("", &DeformableObjectTracking::)
    // .def_readonly("", &DeformableObjectTracking::)
    // .def_readonly("", &DeformableObjectTracking::)
    // .def_readonly("", &DeformableObjectTracking::)
    .def("getVerticesCopy", &DeformableObjectTracking::getVerticesCopy)
    .def("getEdgesCopy", &DeformableObjectTracking::getEdgesCopy);
    ;

    // pyCPDInterface
    //   // .def(py::init<std::string const,
    //   //               double const,
    //   //               int const,
    //   //               double const,
    //   //               double const,
    //   //               double const,
    //   //               double const,
    //   //               double const,
    //   //               double const,
    //   //               MatrixXf const>(),
    //   //     py::arg("log_name_base"),
    //   //     py::arg("tolerance"),
    //   //     py::arg("max_iterations"),
    //   //     py::arg("initial_sigma_scale"),
    //   //     py::arg("w"),
    //   //     py::arg("alpha"),
    //   //     py::arg("beta"),
    //   //     py::arg("zeta"),
    //   //     py::arg("start_lambda"),
    //   //     py::arg("m_lle"))
    // ;
    pyCPDInterface
      .def("get_point_assignments", &CPDInterface::get_point_assignments)
      .def("set_point_assignments", &CPDInterface::set_point_assignments);










    ;
}