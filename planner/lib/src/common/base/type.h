/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

template<typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;

using vector_Eigen2d = vector_Eigen<Eigen::Vector2d>;
using vector_Eigen3d = vector_Eigen<Eigen::Vector3d>;