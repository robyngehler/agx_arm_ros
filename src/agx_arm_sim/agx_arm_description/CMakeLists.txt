cmake_minimum_required(VERSION 3.8)
project(agx_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装功能包资源目录
install(
  DIRECTORY
    launch
    urdf
    config
    rviz
    scripts
    meshes
  DESTINATION share/${PROJECT_NAME}/
)

# 安装仓库内置的 Nero/Revo2 URDF + mesh 资源
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/agx_arm_urdf")
  install(
    DIRECTORY agx_arm_urdf/
    DESTINATION share/${PROJECT_NAME}/agx_arm_urdf/
  )
  message(STATUS "[agx_arm_description] 安装本地 agx_arm_urdf 资产树。")
else()
  message(WARNING
    "[agx_arm_description] 未找到本地 agx_arm_urdf 资产树！"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
