cmake_minimum_required (VERSION 3.10)
set(PACKAGE_NAME REGISTRATION)
project(${PACKAGE_NAME} LANGUAGES CXX)

find_package(PCL REQUIRED)
if(${PCL_FOUND})
    message(STATUS "Found PCL - ${PCL_DIR}")
endif()

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP Found!")
    set(OpenMP_LIBS OpenMP::OpenMP_CXX)
endif()


set(REGISTRATION_SOURCE_FILES
    src/icp.cpp
    src/gicp.cpp
    src/ndt.cpp
)

add_library(${PACKAGE_NAME}
    ${REGISTRATION_SOURCE_FILES}
)
target_include_directories(${PACKAGE_NAME} PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PACKAGE_NAME} PRIVATE
    ${PCL_LIBRARIES}
    ${OpenMP_LIBS}
)
