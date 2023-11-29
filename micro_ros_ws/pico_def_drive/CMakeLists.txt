cmake_minimum_required(VERSION 3.12)



# Name project
SET(ProjectName pico_def_drive)
include(pico_sdk_import.cmake)

project(${ProjectName} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()


# Set any variables required for importing libraries
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../include/FreeRTOS-Kernel)

# Import those libraries
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../include/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
include(micro_ros.cmake)

# Add subdirectories
add_subdirectory(src)

# if you have anything in "lib" folder then uncomment below - remember to add a CMakeLists.txt
# file to the "lib" directory
#add_subdirectory(lib)