cmake_minimum_required(VERSION 2.8)
project(test_kuka CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

find_package(PkgConfig REQUIRED)

PKG_CHECK_MODULES(PC_EIGEN eigen3 REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

PKG_CHECK_MODULES(PC_ORCISIR_ISIRCONTROLLER orcisir_ISIRController REQUIRED)
if(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)
    message("Failed to detect orcisir_ISIRController, check PKG_CONFIG_PATH." )
endif(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)


include_directories(
    ${PC_EIGEN_INCLUDE_DIRS}
    ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS}
)

#========================================
add_library(
    ModelKukaFixed
    SHARED
    kukafixed.cpp
)
add_library(
    ModelKukaFree
    SHARED
    kukafree.cpp
)

target_link_libraries(
    ModelKukaFixed
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
)
target_link_libraries(
    ModelKukaFree
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
)

#========================================
add_executable(
    test_kuka
    main.cpp
)

target_link_libraries(
    test_kuka
    ModelKukaFixed
    ModelKukaFree
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
)

