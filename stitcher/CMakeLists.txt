cmake_minimum_required(VERSION 3.6)

project("airmap-panorama-stitcher")

find_package(OpenCV 4.2 REQUIRED)

# Boost is a development dependency and this binary has very
# little to ask from Boost, so linking statically
set(Boost_USE_STATIC_LIBS ON)
find_package(
    Boost REQUIRED
    program_options
    filesystem
)

include_directories(
    include
    3rdParty
    ${OpenCV_INCLUDE_DIRS}
)

add_library(
    airmap_stitching
    src/camera.cpp
    src/camera_models.cpp
    src/cubemap.cpp
    src/cropper.cpp
    src/distortion.cpp
    src/gimbal.cpp
    src/images.cpp
    src/monitor/estimator.cpp
    src/monitor/monitor.cpp
    src/monitor/timer.cpp
    src/opencv/forward.cpp
    src/opencv/seam_finders.cpp
    src/panorama.cpp
    src/stitcher.cpp
    src/stitcher_configuration.cpp
    3rdParty/TinyEXIF/TinyEXIF.cpp
    3rdParty/TinyEXIF/tinyxml2.cpp
)

add_executable (
    airmap_stitcher
    main.cpp
)

target_link_libraries(
    airmap_stitching
    ${OpenCV_LIBS}
)

target_link_libraries(
    airmap_stitcher
    airmap_stitching
    Boost::program_options
    Boost::filesystem
)

#
# Installation
#
install(TARGETS airmap_stitcher
        RUNTIME DESTINATION bin)

#
# Testing
#
enable_testing()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test/bats)

include(${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/CMakeLists.txt)
