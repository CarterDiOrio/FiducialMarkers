cmake_minimum_required(VERSION 3.22)
project(vicon_comparator)

find_package(aruco REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED )
find_package(pi_eink REQUIRED)
find_package(apriltag REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


# find and import mrcal
add_library(mrcal SHARED IMPORTED)
set_target_properties(mrcal PROPERTIES
  IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libmrcal.so"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/mrcal/"
)


#find and import mrgingham
add_library(mrgingham SHARED IMPORTED)
set_target_properties(mrgingham PROPERTIES
  IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libmrgingham.so"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/mrgingham/"
)


# add vicon library
add_library(datastream SHARED IMPORTED)
set_target_properties(datastream PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/datastream/libViconDataStreamSDK_CPP.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/lib/datastream/"
)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(comparator
              src/tracked_object.cpp 
              src/vicon.cpp
              src/measurement.cpp
              src/extrinsic_observation.cpp
              src/mrcal_reprojected_camera.cpp
              src/mount.cpp
              src/chessboard.cpp
              src/extrinsic_calibration.cpp
              src/measurement_runner.cpp
              src/scaled_camera.cpp)

add_executable(calibrate_extrinsics src/calibrate_extrinsics.cpp)
add_executable(take_measurements src/take_measurements.cpp)
add_executable(realsense_depth_test src/realsense_depth_test.cpp)

find_package(
    Boost REQUIRED COMPONENTS program_options filesystem 
)

target_link_libraries(comparator PUBLIC
                                 datastream 
                                 Eigen3::Eigen 
                                 realsense2 
                                 ${OpenCV_LIBS} 
                                 ${Boost_LIBRARIES} 
                                 Sophus::Sophus
                                 mrcal
                                 mrgingham
                                 Ceres::ceres
                                 aruco
                                 apriltag::apriltag
                                 pi_eink::eink_client)
target_link_libraries(calibrate_extrinsics comparator)
target_link_libraries(take_measurements comparator)
target_link_libraries(realsense_depth_test comparator)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(comparator
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

target_include_directories(calibrate_extrinsics
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

target_include_directories(take_measurements
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>)

target_include_directories(realsense_depth_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)

target_compile_options(comparator PUBLIC -Wall -Wextra -pedantic)
target_compile_options(calibrate_extrinsics PUBLIC -Wall -Wextra -pedantic)
# target_compile_options(chessboard_test PUBLIC -Wall -Wextra -pedantic)

target_compile_features(comparator PUBLIC cxx_std_23)
target_compile_features(calibrate_extrinsics PUBLIC cxx_std_23)
# target_compile_features(chessboard_test PUBLIC cxx_std_23)

