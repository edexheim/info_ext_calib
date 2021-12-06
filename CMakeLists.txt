cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15")

project(multi_calib)

find_package(OpenCV)
find_package(Eigen3 3.3 REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
find_package(GTSAM REQUIRED)
find_package(pangolin REQUIRED)

include_directories(
    include/
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} STATIC "")
add_subdirectory(src/backend)
add_subdirectory(src/camera_models)
add_subdirectory(src/data_reader)
add_subdirectory(src/frontend)
add_subdirectory(src/utils)
add_subdirectory(src/stereo)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  PUBLIC
    ${OpenCV_LIBS}
    tbb
    gtsam
 )

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate PUBLIC ${PROJECT_NAME} pangolin)