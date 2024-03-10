#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(traj_opt, m) {
  auto pyGPMPOptimizer = py::class_<GPMPOptimizer>(m, "GPMPOptimizer");
  pyGPMPOptimizer.def(py::init<>())
      .def("set_debug", &GPMPOptimizer::SetDebug)
      .def("get_result_matrix", &GPMPOptimizer::GetResultMatrix)
      .def("get_layers", &GPMPOptimizer::GetResultLayers)
      .def("get_heights", &GPMPOptimizer::GetResultHeight)
      .def("get_ceilings", &GPMPOptimizer::GetResultCeiling)
      .def("get_opt_init_value", &GPMPOptimizer::GetOptInitValue)
      .def("get_opt_init_layer", &GPMPOptimizer::GetOptInitLayer)
      .def("get_heading_rate", &GPMPOptimizer::GetHeadingRate);

  auto pyGPMPOptimizerWnoa =
      py::class_<GPMPOptimizerWnoa>(m, "GPMPOptimizerWnoa");
  pyGPMPOptimizerWnoa.def(py::init<>())
      .def("set_debug", &GPMPOptimizerWnoa::SetDebug)
      .def("gp_prior_test", &GPMPOptimizerWnoa::GPPriorTest)
      .def("get_result_matrix", &GPMPOptimizerWnoa::GetResultMatrix)
      .def("get_layers", &GPMPOptimizerWnoa::GetResultLayers)
      .def("get_heights", &GPMPOptimizerWnoa::GetResultHeight)
      .def("get_opt_init_value", &GPMPOptimizerWnoa::GetOptInitValue)
      .def("get_opt_init_layer", &GPMPOptimizerWnoa::GetOptInitLayer);
}