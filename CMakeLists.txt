cmake_minimum_required(VERSION 3.21)
project(VTR)

set(CMAKE_CXX_STANDARD 20)

##################################### system #####################################
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "current platform: Linux ")
    set(platform "linux")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "current platform: Windows")
    set(platform "windows")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    MESSAGE(STATUS "current platform: FreeBSD")
ELSE ()
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
    FATAL_ERROR("unknow platform")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

##################################### VTR #####################################
include_directories(${PROJECT_SOURCE_DIR}/include/)

##################################### json #####################################
#set(nlohmann_json_DIR /home/cheng/sources/json)
find_package(nlohmann_json REQUIRED)

##################################### Eigen #####################################
include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

##################################### open3d #####################################
#set(Open3D_DIR /home/cheng_chen/source/open3d13/install_server/lib/cmake/Open3D)
#set(Open3D_DIR /home/cheng/sources/open3d13/install_release/lib/cmake/Open3D)
set(Open3D_DIR /home/cheng/.local/lib/cmake/Open3D)
find_package(Open3D REQUIRED)
link_directories(${Open3D_LIBRARY_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D installed to: ${Open3D_LIBRARY_DIRS}")
endif()

add_executable(VTR main.cpp)

##################################### output bin #####################################
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

target_link_libraries(VTR PUBLIC ${Open3D_LIBRARIES})
target_link_libraries(VTR PRIVATE nlohmann_json::nlohmann_json)