cmake_minimum_required(VERSION 3.16)
project(rxs_ballcalibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io segmentation)
find_package(Eigen3 REQUIRED)

# DLL library
add_library(BallCalibrationDLL SHARED
    BallCalibrationDLL/BallCalibration.cpp
    BallCalibrationDLL/Ransac3D.cpp
    BallCalibrationDLL/dllmain.cpp
    BallCalibrationDLL/pch.cpp
)

target_include_directories(BallCalibrationDLL PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BallCalibrationDLL
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(BallCalibrationDLL PRIVATE ${PCL_LIBRARIES})
target_precompile_headers(BallCalibrationDLL PRIVATE BallCalibrationDLL/pch.h)

if(MSVC)
    target_compile_definitions(BallCalibrationDLL PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX BALLEXPORT_EXPORTS)
    target_compile_options(BallCalibrationDLL PRIVATE /W3 /utf-8)
endif()

# Test application
add_executable(${PROJECT_NAME} 源.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BallCalibrationDLL
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE BallCalibrationDLL ${PCL_LIBRARIES})
