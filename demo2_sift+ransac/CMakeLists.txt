cmake_minimum_required(VERSION 3.10)

project(FeatureMatch)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(OpenCV REQUIRED)

message(STATUS "    version: ${OpenCV_VERSION}")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(FeatureMatch RANSAC.cpp)

target_link_libraries(FeatureMatch PRIVATE
        ${OpenCV_LIBS}
)
