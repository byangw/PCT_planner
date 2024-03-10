add_library(a_star_search SHARED
  a_star_search.cc
)

pybind11_add_module(a_star
  python_interface.cc
)
target_link_libraries(a_star PRIVATE a_star_search)
