# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/amir/rspi_pico/pico/pico-sdk/tools/pioasm"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pioasm"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
