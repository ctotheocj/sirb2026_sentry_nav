cmake_minimum_required(VERSION 3.8)
project(sirb2026_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

set(SIRB2026_NAV_BRINGUP_SHARE_DIRS
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  list(APPEND SIRB2026_NAV_BRINGUP_SHARE_DIRS docs)
endif()

install(PROGRAMS
  scripts/tf_jump_monitor.py
  scripts/apply_motion_profile.py
  scripts/check_navigation_chain.py
  scripts/visualize_hole_regions.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  ${SIRB2026_NAV_BRINGUP_SHARE_DIRS}
)
