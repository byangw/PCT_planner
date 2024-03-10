add_library(ele_planner_lib SHARED
    offline_ele_planner.cc
)
target_link_libraries(ele_planner_lib PUBLIC
    a_star_search
    gpmp_optimizer
    map_manager
)
pybind11_add_module(ele_planner
    python_interface.cc
)
target_link_libraries(ele_planner PRIVATE
    ele_planner_lib
)