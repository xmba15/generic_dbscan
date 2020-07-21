cmake_minimum_required(VERSION 3.10)

project(dbscan)

set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenMP QUIET)

add_compile_options(
  "$<$<CONFIG:DEBUG>:-DDEBUG>"
)

add_subdirectory(src)

if(BUILD_APPS)
  add_subdirectory(examples)
endif(BUILD_APPS)

if(BUILDING_TEST)
  enable_testing()
  add_subdirectory(tests)
endif()
