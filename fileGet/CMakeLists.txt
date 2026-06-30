cmake_minimum_required(VERSION 3.16)
project(rxs_fileget LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}
    源.cpp
    ${CMAKE_SOURCE_DIR}/czxDependence/czxTool_std.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/czxDependence
)

if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3 /utf-8)
endif()
