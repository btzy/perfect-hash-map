cmake_minimum_required(VERSION 3.2)
project(phm)

add_subdirectory(src)

option(PHM_BUILD_TESTS "Build tests for BIG" OFF)
if(${PHM_BUILD_TESTS})
  add_subdirectory(vendor/catch2)
  add_subdirectory(test)
endif()
