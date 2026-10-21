cmake_minimum_required(VERSION 3.3.0)

project(CTRE_PhoenixPlatform)

# Specific library installation location on target
# SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/frc/third-party/lib")

# Include header files (Phoenix includes, HAL includes, Mau Includes
include_directories(/home/pi/Phoenix-Linux-SocketCAN-Example/include ../../include/ ..)

# Create shared library
add_library(${PROJECT_NAME} SHARED Platform_mau.cpp)

# Configure the hal as a dependency of this project
# This ensures the library is created before it is linked to
add_dependencies(${PROJECT_NAME} hal)

if (BUILD_SHARED_LIBS)
	target_link_libraries(${PROJECT_NAME} PRIVATE hal)
endif()

# Install Library
install(TARGETS ${PROJECT_NAME} DESTINATION frc/third-party/lib)
