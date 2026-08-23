cmake_minimum_required(VERSION 3.16)
project(agx_arm_retiming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pybind11_vendor REQUIRED)
# pybind11's newer CMake tooling builds on FindPython3, which it does not pull
# in itself; without this pybind11_add_module fails on python3_add_library.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# The extension lands inside the Python package so `from agx_arm_retiming import
# _totg` works without a second install path.
pybind11_add_module(_totg SHARED src/retiming_bindings.cpp)
# ament_target_dependencies, not a bare target_link_libraries: moveit_core pulls
# moveit_msgs and rosidl_runtime_cpp headers transitively, and a manual include
# path misses them.
ament_target_dependencies(_totg PUBLIC moveit_core)
target_link_libraries(_totg PRIVATE Eigen3::Eigen)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS _totg
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_retiming test)
endif()

ament_package()
