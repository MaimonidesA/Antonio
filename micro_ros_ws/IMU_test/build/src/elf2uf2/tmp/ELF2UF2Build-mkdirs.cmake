# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/amir/rspi_pico/pico/pico-sdk/tools/elf2uf2"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/elf2uf2"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/tmp"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/src/ELF2UF2Build-stamp"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/src"
  "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/amir/antonio_ws/Antonio/micro_ros_ws/IMU_test/build/src/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()
