cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MAJOR_PLANES_DETECTOR)

find_package(PCL 1.2 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(helper_lib helper.cpp helper.h)
add_executable(pcl_detect_planes
               pcl_detect_planes.cpp )
target_link_libraries(pcl_detect_planes ${PCL_LIBRARIES} helper_lib)

################################
# GTest
################################
ADD_SUBDIRECTORY (googletest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

##############
# Unit Tests
##############
add_executable(runUnitTests test_pcl_detect_planes.cpp)

# Standard linking to gtest stuff.
target_link_libraries(runUnitTests gtest gtest_main)

# Extra linking for the project.
target_link_libraries(runUnitTests ${PCL_LIBRARIES} helper_lib)
