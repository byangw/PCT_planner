set(osqp_DIR ${PROJECT_SOURCE_DIR}/3rdparty/osqp/install/lib/cmake/osqp)

find_package(osqp REQUIRED)

add_library(common_smoothing SHARED
    affine_constraint.cc
    discrete_points_math.cc
    osqp_spline1d_solver.cc
    osqp_spline2d_solver.cc
    polynomialxd.cc
    spline1d_constraint.cc
    spline1d_kernel_helper.cc
    spline1d_kernel.cc
    spline1d_seg.cc
    spline1d.cc
    spline2d_constraint.cc
    spline2d_kernel.cc
    spline2d_seg.cc
    spline2d.cc
    solver/osqp/osqp_interface.cc
    utils/math.cc
)
target_include_directories(common_smoothing PUBLIC
  ${PROJECT_SOURCE_DIR}/3rdparty/osqp/install/include
)
target_link_libraries(common_smoothing
  osqp::osqpstatic
)