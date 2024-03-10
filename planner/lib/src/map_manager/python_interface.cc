#include "map_manager/dense_elevation_map.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(py_map_manager, m) {
  auto pyDenseElevationMap =
      py::class_<DenseElevationMap>(m, "DenseElevationMap");
  pyDenseElevationMap.def(py::init<>())
      .def("update_layer", &DenseElevationMap::UpdateLayer)
      .def("update_layer_safe", &DenseElevationMap::UpdateLayerSafe)
      .def("set_debug", &DenseElevationMap::SetDebug);
}