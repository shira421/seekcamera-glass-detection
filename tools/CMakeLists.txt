cmake_minimum_required(VERSION 3.10)
project(capture_shape)

set(CMAKE_CXX_STANDARD 17)

# SDL2 via vcpkg
find_package(SDL2 CONFIG REQUIRED)

set(SEEKCAMERA_SDK_DIR "C:/Program Files/Seek Thermal/Seek Thermal SDK/4.4.1.0")
set(SEEKCAMERA_INCLUDE_DIRS "${SEEKCAMERA_SDK_DIR}/include")
set(SEEKCAMERA_LIBRARIES "${SEEKCAMERA_SDK_DIR}/lib/x64/seekcamera.lib")

add_executable(capture_shape capture_shape.cpp)

target_include_directories(capture_shape PRIVATE 
    ${SEEKCAMERA_INCLUDE_DIRS}
)

target_link_libraries(capture_shape 
    SDL2::SDL2
    SDL2::SDL2main
    ${SEEKCAMERA_LIBRARIES}
)

add_custom_command(TARGET capture_shape POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SEEKCAMERA_SDK_DIR}/lib/x64/seekcamera.dll"
    $<TARGET_FILE_DIR:capture_shape>
)