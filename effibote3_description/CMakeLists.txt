cmake_minimum_required(VERSION 3.5)
project(effibote3_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/effibote3_description.py)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_effibote3_description test/test_effibote3_description.py)
endif()

ament_package()
