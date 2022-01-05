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

find_package(PCL REQUIRED)

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

# ---------------------------------------------------------------
# test point cloud clustering with gpu dbscan
# ---------------------------------------------------------------

if(NOT CMAKE_CUDA_COMPILER OR NOT USE_GPU)
  return()
endif()

add_executable(test_pointcloud_clustering_gpu
  ${CMAKE_CURRENT_LIST_DIR}/DBSCANPointCloudGPU.cpp
)

target_link_libraries(test_pointcloud_clustering_gpu
  PUBLIC
    dbscan_cuda
    ${PCL_LIBRARIES}
)

target_include_directories(test_pointcloud_clustering_gpu
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)
