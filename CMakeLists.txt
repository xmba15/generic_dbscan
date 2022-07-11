cmake_minimum_required(VERSION 3.10)

project(dbscan)

set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# ------------------------------------------------
# cpu library
# ------------------------------------------------
add_library(dbscan
  INTERFACE
)

target_include_directories(dbscan
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(dbscan
  INTERFACE
     $<$<CONFIG:Debug>:-O0 -g -Wall -Werror>
     $<$<CONFIG:Release>:-O3>
)

# ------------------------------------------------
# gpu library
# ------------------------------------------------
if(USE_GPU)
  include(CheckLanguage)
  check_language(CUDA)
  if (CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    add_subdirectory(cuda)
  else()
    message(STATUS "CUDA not found")
    return()
  endif()
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(BUILD_EXAMPLES)

if(BUILDING_TEST)
  enable_testing()
  add_subdirectory(tests)
endif()
