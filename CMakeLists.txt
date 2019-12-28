cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(MAJOR_PLANES_DETECTOR)

set (OpenCV_DIR C:/libs/opencv/build)
find_package(PCL 1.2 REQUIRED COMPONENTS)
find_package( OpenCV 4.1.2 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}
					${OpenCV_INCLUDE_DIRS})
message(PCL_LIBRARY_DIRS:  ${PCL_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_library(helper_lib helper.cpp helper.h)
add_executable(pcl_detect_planes
               pcl_detect_planes.cpp )
add_executable(pcl_playback
               pcl_playback.cpp )
			   
message(${PCL_LIBRARIES})
message(${OpenCV_LIBS})
target_link_libraries(pcl_detect_planes ${PCL_LIBRARIES} helper_lib)
target_link_libraries(pcl_playback ${PCL_LIBRARIES} ${OpenCV_LIBS} helper_lib)
#target_link_libraries(pcl_playback ${PCL_LIBRARIES} helper_lib)

################################
# GTest
################################
ADD_SUBDIRECTORY (googletest/googletest)
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
