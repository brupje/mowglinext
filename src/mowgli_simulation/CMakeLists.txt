cmake_minimum_required(VERSION 3.22)
project(mowgli_simulation)

# Enforce C++20 globally even though this package contains no compiled sources,
# so that any future C++ additions start from the correct standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Install data directories
# All simulation assets are pure data — no compiled artefacts.
# ---------------------------------------------------------------------------
install(
  DIRECTORY
    worlds/
    models/
    launch/
    config/
    rviz/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
