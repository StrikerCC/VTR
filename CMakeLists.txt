cmake_minimum_required(VERSION 3.21)
project(VTR)
set(CMAKE_CXX_STANDARD 17)

##################################### system #####################################
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "current platform: Linux ")
    set(platform "linux")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "current platform: Windows")
    set(platform "Windows")
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
if(${platform} STREQUAL "Windows")
    MESSAGE(STATUS "current platform: Windows")
    set(nlohmann_json_DIR D:/source/json/install/lib/cmake/nlohmann_json)
endif()
find_package(nlohmann_json REQUIRED)
include_directories(${nlohmann_json_INCLUDE_DIRS})
link_directories(${nlohmann_json_LIBRARY_DIRS})
if (nlohmann_json_FOUND)
    message(STATUS "Found nlohmann_json ${nlohmann_json_VERSION}")
    message(STATUS "nlohmann_json installed to: ${nlohmann_json_LIBRARY_DIRS}")
endif()


##################################### Eigen #####################################
if(${platform} STREQUAL "linux")
    include_directories("/usr/include/eigen3")
elseif(${platform} STREQUAL "Windows")
    include_directories("D:/source/eigen-3.4.0/Eigen")
endif()
#find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

##################################### open3d #####################################
if(${platform} STREQUAL "linux")
    set(Open3D_DIR /home/cheng/.local/lib/cmake/Open3D)
elseif(${platform} STREQUAL "Windows")
    set(Open3D_DIR D:/source/Open3D/install/open3d-release/CMake)
endif()

find_package(Open3D REQUIRED)
link_directories(${Open3D_LIBRARY_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D installed to: ${Open3D_LIBRARY_DIRS}")
endif()

add_executable(VTR main.cpp src/Cam.cpp include/Cam.h)

##################################### output bin #####################################
# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

target_link_libraries(VTR PRIVATE nlohmann_json::nlohmann_json)

if (${platform} STREQUAL "linux")
    target_link_libraries(VTR PUBLIC ${Open3D_LIBRARIES})
elseif (${platform} STREQUAL "Windows")
    target_link_libraries(VTR PRIVATE Open3D::Open3D)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET VTR POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                #    ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                ${Open3D_DIR}/../bin/Open3D.dll   
                                ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()

