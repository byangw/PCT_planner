#!/bin/bash

ROOT_DIR=$(cd $(dirname "$0"); pwd)
# echo "ROOT_DIR: ${ROOT_DIR}"

cd lib

# rm -rf build
mkdir build

cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j6
cp ./src/a_star/a_star*.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cd ..

# # optional
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/lib/build/src/common/smoothing
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/lib
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
