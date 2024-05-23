# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(vicon_comparator)

# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(realsense2 REQUIRED)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# add vicon library
add_library(datastream SHARED IMPORTED)
set_target_properties(datastream PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/datastream/libViconDataStreamSDK_CPP.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/datastream/"
)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_executable(comparator 
                src/client.cpp 
                src/tracked_object.cpp 
                src/vicon.cpp
                src/mount.cpp)

target_link_libraries(comparator datastream Eigen3::Eigen realsense2 ${OpenCV_LIBS})

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

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(comparator PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(comparator PUBLIC cxx_std_20)


# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
#target_link_libraries(Name Eigen3::Eigen libname)

# install the include files by copying the whole include directory
# install(DIRECTORY include/mylibrary DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
# install(TARGETS Name libname EXPORT project_name-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
#install(EXPORT vicon_comparator-targets
#        FILE vicon_comparator-config.cmake
#        NAMESPACE vicon_comparator::
#        DESTINATION lib/cmake/${PROJECT_NAME})