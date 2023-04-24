cmake_minimum_required (VERSION 3.10)
set(PACKAGE_NAME PointCloud-Registration-Implementation)
project(${PACKAGE_NAME} LANGUAGES CXX)

add_subdirectory(modules)

add_executable(icp_test examples/icp_test.cpp)
target_link_libraries(icp_test PRIVATE
    REGISTRATION
)