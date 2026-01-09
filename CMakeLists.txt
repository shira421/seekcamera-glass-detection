cmake_minimum_required(VERSION 3.10)
project(thermal_distance_sensor VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    src/main.cpp
    src/thermal_distance_sensor.cpp
    src/rendering.cpp
    src/map_view.cpp
)

# Include directories
include_directories(include)

# Platform-specific configuration
if(WIN32)
    # Windows with vcpkg
    find_package(SDL2 CONFIG REQUIRED)
    find_package(SDL2_ttf CONFIG REQUIRED)
    
    # Seek SDK paths (relative to SDK examples folder)
    set(SEEKCAMERA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
    set(SEEKCAMERA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib")
    
    include_directories(${SEEKCAMERA_INCLUDE_DIR})
    link_directories(${SEEKCAMERA_LIB_DIR})
    
    add_executable(${PROJECT_NAME} ${SOURCES})
    
    target_link_libraries(${PROJECT_NAME} PRIVATE
        $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
        $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
        $<IF:$<TARGET_EXISTS:SDL2_ttf::SDL2_ttf>,SDL2_ttf::SDL2_ttf,SDL2_ttf::SDL2_ttf-static>
        seekcamera
    )
else()
    # Linux (including Raspberry Pi)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2 REQUIRED sdl2)
    pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)
    
    # Seek SDK paths for Linux
    set(SEEKCAMERA_INCLUDE_DIR "/usr/local/include")
    set(SEEKCAMERA_LIB_DIR "/usr/local/lib")
    
    include_directories(
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_TTF_INCLUDE_DIRS}
        ${SEEKCAMERA_INCLUDE_DIR}
    )
    
    link_directories(
        ${SDL2_LIBRARY_DIRS}
        ${SDL2_TTF_LIBRARY_DIRS}
        ${SEEKCAMERA_LIB_DIR}
    )
    
    add_executable(${PROJECT_NAME} ${SOURCES})
    
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDL2_LIBRARIES}
        ${SDL2_TTF_LIBRARIES}
        seekcamera
        pthread
    )
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()