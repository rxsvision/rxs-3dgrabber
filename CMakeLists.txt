cmake_minimum_required(VERSION 3.16)
project(rxs-3dgrabber LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(RXS_BUILD_BALLCALIBRATION "Build BallCalibration DLL + test" ON)
option(RXS_BUILD_CYLINDER "Build Cylinder detection" ON)
option(RXS_BUILD_FILEGET "Build fileGet utility" ON)

if(RXS_BUILD_BALLCALIBRATION)
    add_subdirectory(3d-detect/BallCalibration)
endif()
if(RXS_BUILD_CYLINDER)
    add_subdirectory(3d-detect/Cylinder)
endif()
if(RXS_BUILD_FILEGET)
    add_subdirectory(fileGet)
endif()
