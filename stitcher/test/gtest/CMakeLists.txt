cmake_minimum_required(VERSION 3.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/CMakeLists-googletest.txt)
googletest_setup()

include(${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/opencv_assert/CMakeLists.txt)

add_executable(cameraTests test/gtest/camera.cpp)
add_executable(cameraModelsTests test/gtest/camera_models.cpp)
add_executable(distortionTests test/gtest/distortion.cpp)
add_executable(panoramaTests test/gtest/panorama.cpp)
add_executable(gimbalTests test/gtest/gimbal.cpp)
add_executable(imagesTests test/gtest/images.cpp)

target_link_libraries(cameraTests gtest gtest_main airmap_stitching)
target_link_libraries(cameraModelsTests gtest gtest_main airmap_stitching Boost::filesystem)
target_link_libraries(distortionTests gtest gtest_main airmap_stitching Boost::filesystem)
target_link_libraries(panoramaTests gtest gtest_main airmap_stitching Boost::filesystem)
target_link_libraries(gimbalTests gtest gtest_main airmap_stitching)
target_link_libraries(imagesTests gtest gtest_main airmap_stitching opencv_assert Boost::filesystem)

add_test(cameraTests cameraTests)
add_test(cameraModelsTests cameraModelsTests)
add_test(distortionTests distortionTests)
add_test(panoramaTests panoramaTests)
add_test(gimbalTests gimbalTests)
add_test(imagesTests imagesTests)
