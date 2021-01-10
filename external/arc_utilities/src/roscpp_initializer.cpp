#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;

std::unique_ptr<ros::AsyncSpinner> spinner;

void init_node(std::string const &name, std::vector<std::string> argv = {}, int const rospy_log_level = 2,
               bool anonymous = false, bool disable_rosout = false, bool disable_signals = false) {
  switch (rospy_log_level) {
    case 0:
      ros::console::set_logger_level(name, ros::console::Level::Debug);
      break;
    case 1:
      ros::console::set_logger_level(name, ros::console::Level::Info);
      break;
    case 2:
      ros::console::set_logger_level(name, ros::console::Level::Warn);
      break;
    case 3:
      ros::console::set_logger_level(name, ros::console::Level::Error);
      break;
    case 4:
      ros::console::set_logger_level(name, ros::console::Level::Fatal);
      break;
    default:
      throw std::invalid_argument("log level must be 0-4");
  }

  using InitOptionUnderlyingType = std::underlying_type_t<ros::init_options::InitOption>;
  InitOptionUnderlyingType options = {0};
  if (anonymous) {
    options |= static_cast<InitOptionUnderlyingType>(ros::init_options::AnonymousName);
  }
  if (disable_signals) {
    options |= static_cast<InitOptionUnderlyingType>(ros::init_options::NoSigintHandler);
  }
  if (disable_rosout) {
    options |= static_cast<InitOptionUnderlyingType>(ros::init_options::NoRosout);
  }
  auto argc = static_cast<int>(argv.size());
  std::vector<char *> argv_pointers;
  auto const convert = [](const std::string &s) {
    char *pc = new char[s.size() + 1];
    std::strcpy(pc, s.c_str());
    return pc;
  };
  std::transform(argv.begin(), argv.end(), std::back_inserter(argv_pointers), convert);
  ros::init(argc, argv_pointers.data(), name);

  spinner = std::make_unique<ros::AsyncSpinner>(1);
  spinner->start();
}

void shutdown() { ros::shutdown(); }

PYBIND11_MODULE(roscpp_initializer, m) {
  m.doc() = "roscpp initializer module";
  m.def("init_node", &init_node, "init node but for C++", py::arg("name"), py::arg("argv") = std::vector<std::string>{},
        py::arg("log_level") = 2, py::arg("anonymous") = false, py::arg("disable_rosout") = false,
        py::arg("disable_signals") = false);
  m.def("shutdown", &shutdown, "shutdown");
}
