cmake_minimum_required(VERSION 3.10)

# ---------------------------------------------------------------
# test simple dbscan example
# ---------------------------------------------------------------

add_executable(test_simple_dbscan
  ${CMAKE_CURRENT_LIST_DIR}/TestDBSCAN.cpp
)

target_link_libraries(test_simple_dbscan
  PUBLIC
    ${PROJECT_NAME}
)

# ---------------------------------------------------------------
# test point cloud clustering with dbscan
# ---------------------------------------------------------------

find_package(PCL QUIET)

if (NOT ${PCL_FOUND})
  message(STATUS "PCL not found\n")
  return()
endif()

add_executable(test_pointcloud_clustering
  ${CMAKE_CURRENT_LIST_DIR}/DBSCANPointCloud.cpp
)

target_link_libraries(test_pointcloud_clustering
  PUBLIC
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
)

target_include_directories(test_pointcloud_clustering
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)
