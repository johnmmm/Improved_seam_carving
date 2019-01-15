cmake_minimum_required(VERSION 3.6)
project(test)

include(FindPkgConfig)
find_package(PkgConfig) 
pkg_check_modules(OpenCv REQUIRED opencv)



set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp Improved_seam.hpp Improved_seam.cpp)

set(CMAKE_BUILD_TYPE Debug)

link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include, /usr/include/opencv)

add_executable(seam_carving ${SOURCE_FILES})

target_link_libraries(seam_carving ${OpenCv_LIBRARIES})
target_include_directories(seam_carving PUBLIC ${OpenCv_INCLUDE_DIRS})
target_compile_options(seam_carving PUBLIC ${OpenCv_CFLAGS})