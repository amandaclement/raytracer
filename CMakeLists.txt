cmake_minimum_required(VERSION 3.8)

project(raytracer) 

#IMPORTANT COMPILING DIRECTIVES

add_compile_options(-RAYTRACER)

# When testing large scenes the debug mode will be very slow
# so switch to release
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_PREFIX_PATH
    /encs # For ENCS lab computers
    /opt/local # Macports
)

# ENCS Eigen3 CMake setup
if (WIN32)
include_directories(C:/Users/ambec/Desktop/labs/COMP371_all/COMP371_RaytracerBase/code/eigen-3.4.0/)# for windows you can directly hardcode the paths to your libraries in the include
else() 
set(ENV{EIGEN3_ROOT} /encs/pkg/eigen-3.3.7/root)
set(CMAKE_MODULE_PATH /encs/pkg/eigen-3.3.7/root/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
endif()
# If you need too manually add the paths to your Eigen library, you can add it here
# do not delete the other 

# These folders include some important header files including the header files for your own solution
#do not remove
include_directories(src/)
include_directories(external/)

#internal includes
aux_source_directory(external SOURCE)
aux_source_directory(src SOURCE)

add_executable(raytracer main.cpp ${SOURCE}) #The name of the cpp file and its path can vary

