cmake_minimum_required(VERSION 3.22)
project(rsapriltag)

find_package(realsense2 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED )
find_package(apriltag REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(detect src/main.cpp src/ncam_pnp.cpp src/tagCustom20h10.c)
target_link_libraries(detect realsense2 
                              ${OpenCV_LIBS} 
                              Sophus::Sophus
                              Ceres::ceres
                              apriltag::apriltag)

target_include_directories(detect
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)


target_compile_options(detect PUBLIC -Wall -Wextra -pedantic)
target_compile_features(detect PUBLIC cxx_std_23)