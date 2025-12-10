cmake_minimum_required(VERSION 3.15)

#--------------------------------------------------------------------------------------------------------------------------#
# Project configuration
#--------------------------------------------------------------------------------------------------------------------------#
project(seekcamera-glass-detector 
    VERSION 1.0.0
    DESCRIPTION "Seek Thermal SDK - Glass Detection Application"
    LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------------------------------------------------------------------------------------#
# Manually specify all library paths
#--------------------------------------------------------------------------------------------------------------------------#

# Seek Thermal SDK paths
set(SEEKCAMERA_SDK_DIR "C:/Users/vkshe/OneDrive/Documents/NUS/internships/prox/seekthermal/Seek_Thermal_SDK_4.4.2.20/x64-windows")
set(SEEKCAMERA_INCLUDE "${SEEKCAMERA_SDK_DIR}/include")
set(SEEKCAMERA_LIB "${SEEKCAMERA_SDK_DIR}/lib/seekcamera.lib")
set(SEEKCAMERA_DLL "${SEEKCAMERA_SDK_DIR}/lib/seekcamera.dll")

# SDL2 paths
set(SDL2_DIR "C:/SDL2-2.32.10")
set(SDL2_INCLUDE_DIR "${SDL2_DIR}/include")
set(SDL2_LIBRARY "${SDL2_DIR}/lib/x64/SDL2.lib")
set(SDL2_DLL "${SDL2_DIR}/lib/x64/SDL2.dll")

# SDL2_ttf paths
set(SDL2_TTF_DIR "C:/SDL2_ttf-2.24.0")
set(SDL2_TTF_INCLUDE_DIR "${SDL2_TTF_DIR}/include")
set(SDL2_TTF_LIBRARY "${SDL2_TTF_DIR}/lib/x64/SDL2_ttf.lib")
set(SDL2_TTF_DLL "${SDL2_TTF_DIR}/lib/x64/SDL2_ttf.dll")

#--------------------------------------------------------------------------------------------------------------------------#
# Verify all paths exist
#--------------------------------------------------------------------------------------------------------------------------#
if(NOT EXISTS ${SEEKCAMERA_INCLUDE})
    message(FATAL_ERROR "Seek SDK include not found at: ${SEEKCAMERA_INCLUDE}")
endif()

if(NOT EXISTS ${SEEKCAMERA_LIB})
    message(FATAL_ERROR "Seek SDK library not found at: ${SEEKCAMERA_LIB}")
endif()

if(NOT EXISTS ${SDL2_INCLUDE_DIR})
    message(FATAL_ERROR "SDL2 include not found at: ${SDL2_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${SDL2_LIBRARY})
    message(FATAL_ERROR "SDL2 library not found at: ${SDL2_LIBRARY}")
endif()

if(NOT EXISTS ${SDL2_TTF_LIBRARY})
    message(FATAL_ERROR "SDL2_ttf library not found at: ${SDL2_TTF_LIBRARY}")
endif()

#--------------------------------------------------------------------------------------------------------------------------#
# Executable configuration
#--------------------------------------------------------------------------------------------------------------------------#
add_executable(${PROJECT_NAME}
    src/seekcamera-glass-detector.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${SEEKCAMERA_INCLUDE}
    ${SDL2_INCLUDE_DIR}
    ${SDL2_TTF_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${SEEKCAMERA_LIB}
    ${SDL2_LIBRARY}
    ${SDL2_TTF_LIBRARY}
)

#--------------------------------------------------------------------------------------------------------------------------#
# Copy DLLs to build directory
#--------------------------------------------------------------------------------------------------------------------------#
if(WIN32)
    # Copy seekcamera.dll
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SEEKCAMERA_DLL}"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/seekcamera.dll
        COMMENT "Copying seekcamera.dll"
    )
    
    # Copy SDL2.dll
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL2_DLL}"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/SDL2.dll
        COMMENT "Copying SDL2.dll"
    )
    
    # Copy SDL2_ttf.dll
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL2_TTF_DLL}"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/SDL2_ttf.dll
        COMMENT "Copying SDL2_ttf.dll"
    )
endif()

#--------------------------------------------------------------------------------------------------------------------------#
# Print configuration summary
#--------------------------------------------------------------------------------------------------------------------------#
message(STATUS "")
message(STATUS "=== ${PROJECT_NAME} Configuration ===")
message(STATUS "Seek SDK: ${SEEKCAMERA_SDK_DIR}")
message(STATUS "Seek include: ${SEEKCAMERA_INCLUDE}")
message(STATUS "Seek library: ${SEEKCAMERA_LIB}")
message(STATUS "SDL2 include: ${SDL2_INCLUDE_DIR}")
message(STATUS "SDL2 library: ${SDL2_LIBRARY}")
message(STATUS "SDL2_ttf include: ${SDL2_TTF_INCLUDE_DIR}")
message(STATUS "SDL2_ttf library: ${SDL2_TTF_LIBRARY}")
message(STATUS "=================================")
message(STATUS "")