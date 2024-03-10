add_library(map_manager SHARED
    dense_elevation_map.cc
)
pybind11_add_module(py_map_manager
  python_interface.cc
)
target_link_libraries(py_map_manager PRIVATE map_manager)