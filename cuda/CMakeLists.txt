cmake_minimum_required(VERSION 3.10)

project(dbscan_cuda)

set (CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

add_library(dbscan_cuda
  SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DBSCAN.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CudaUtils.cu
)

target_include_directories(dbscan_cuda
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  PRIVATE
      ${PCL_INCLUDE_DIRS}
)

target_link_libraries(dbscan_cuda
  PRIVATE
    ${PCL_LIBRARIES}
)

target_compile_options(dbscan_cuda
  PRIVATE
     $<$<COMPILE_LANGUAGE:CUDA>:-expt-extended-lambda -Xcompiler -fPIC -Xcudafe --diag_suppress=esa_on_defaulted_function_ignored>
)
