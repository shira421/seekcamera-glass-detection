cmake_minimum_required(VERSION 3.10)
project(thermal_distance_sensor VERSION 1.0.0 LANGUAGES CXX)

# C++11 standard required
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find SDL2
find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_ttf CONFIG REQUIRED)

# Seek Thermal SDK paths
get_filename_component(SDK_ROOT "${PROJECT_SOURCE_DIR}/../.." ABSOLUTE)
set(SEEKCAMERA_INCLUDE_DIR "${SDK_ROOT}/include" CACHE PATH "Seek Camera SDK include directory")
set(SEEKCAMERA_LIB_DIR "${SDK_ROOT}/lib" CACHE PATH "Seek Camera SDK library directory")

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${SEEKCAMERA_INCLUDE_DIR}
)

# Link directories
link_directories(${SEEKCAMERA_LIB_DIR})

# Source files
set(SOURCES
    src/main.cpp
    src/thermal_distance_sensor.cpp
    src/rendering.cpp
)

# Header files
set(HEADERS
    include/thermal_distance_sensor.h
    include/rendering.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    $<IF:$<TARGET_EXISTS:SDL2_ttf::SDL2_ttf>,SDL2_ttf::SDL2_ttf,SDL2_ttf::SDL2_ttf-static>
    seekcamera
)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES ${HEADERS} DESTINATION include/thermal_distance_sensor)

# CPack configuration for packaging
set(CPACK_PACKAGE_NAME "thermal-distance-sensor")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Thermal triangulation distance sensor")
include(CPack)