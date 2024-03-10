add_library(gpmp_optimizer SHARED
  gpmp_optimizer/factors/gp_prior_factor.cc
  gpmp_optimizer/factors/gp_obstacle_factor.cc
  gpmp_optimizer/factors/gp_interpolate_obstacle_factor.cc
  gpmp_optimizer/factors/gp_heading_rate_factor.cc
  gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.cc
  gpmp_optimizer/factors_wnoa/gp_prior_factor.cc
  gpmp_optimizer/factors_wnoa/gp_obstacle_factor.cc
  gpmp_optimizer/factors_wnoa/gp_interpolate_obstacle_factor.cc
  gpmp_optimizer/factors_origin/gp_prior_factor.cc
  gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.cc
  gpmp_optimizer/interpolator/wnoa_trajectory_interpolator.cc
  gpmp_optimizer/gpmp_optimizer.cc
  gpmp_optimizer/gpmp_optimizer_wnoa.cc
  height_smoother/height_smoother.cc
)
target_link_libraries(gpmp_optimizer PUBLIC
  gtsam
  map_manager
  common_smoothing
)

pybind11_add_module(traj_opt
  python_interface.cc
)

target_link_libraries(traj_opt PRIVATE gpmp_optimizer)

# add_executable(test_height_smoother
#   height_smoother/test_height_smoother.cc
#   height_smoother/height_smoother.cc
# )
# target_link_libraries(test_height_smoother PUBLIC
#   common_smoothing)