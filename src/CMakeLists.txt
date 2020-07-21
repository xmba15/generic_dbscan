cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME ${PROJECT_NAME})

add_library(${LIBRARY_NAME}
  INTERFACE
)


target_include_directories(${LIBRARY_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${LIBRARY_NAME}
  INTERFACE
     $<$<CONFIG:Debug>:-O0 -g -Wall -Werror>
     $<$<CONFIG:Release>:-O3>
)

list(APPEND
  PUBLIC_LIBS
)

if (OpenMP_CXX_FOUND)
  message(STATUS "use openmp")
  list(APPEND PUBLIC_LIBS OpenMP::OpenMP_CXX)
endif()

target_link_libraries(${LIBRARY_NAME}
  INTERFACE
    ${PUBLIC_LIBS}
)
