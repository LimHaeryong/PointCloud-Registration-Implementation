cmake_minimum_required (VERSION 3.10)
set(PACKAGE_NAME PointCloud-Registration-Implementation)
project(${PACKAGE_NAME} LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -O3") 

add_subdirectory(modules)

set(SOURCE_CLOUD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/resources/room_scan1.pcd)
set(TARGET_CLOUD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/resources/room_scan2.pcd)

add_executable(icp_test examples/icp_test.cpp)
target_link_libraries(icp_test PRIVATE
    REGISTRATION
)
target_compile_definitions(icp_test PUBLIC
    SOURCE_CLOUD_PATH="${SOURCE_CLOUD_PATH}"
    TARGET_CLOUD_PATH="${TARGET_CLOUD_PATH}"
)

add_executable(kdtree_test examples/kdtree_test.cpp)
target_link_libraries(kdtree_test PRIVATE
    REGISTRATION
)
target_compile_definitions(kdtree_test PUBLIC
    SOURCE_CLOUD_PATH="${SOURCE_CLOUD_PATH}"
    TARGET_CLOUD_PATH="${TARGET_CLOUD_PATH}"
)