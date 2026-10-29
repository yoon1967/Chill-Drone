cmake_minimum_required(VERSION 3.16)
project(orb_video LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)
add_executable(orb_video bridge.cpp)
target_compile_options(orb_video PRIVATE -march=native)
target_link_options(orb_video PRIVATE -rdynamic)
add_executable(sim3_geometry_test sim3_geometry_test.cpp)
add_executable(statistics_test statistics_test.cpp)
target_link_libraries(sim3_geometry_test PRIVATE Eigen3::Eigen)
target_include_directories(orb_video PRIVATE /opt/ORB_SLAM3 /opt/ORB_SLAM3/include
    /opt/ORB_SLAM3/include/CameraModels /opt/ORB_SLAM3/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(orb_video PRIVATE /opt/ORB_SLAM3/lib/libORB_SLAM3.so
    Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Threads::Threads)
