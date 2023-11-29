# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# compile ASM with /usr/bin/arm-none-eabi-gcc
# compile C with /usr/bin/arm-none-eabi-gcc
# compile CXX with /usr/bin/arm-none-eabi-g++
ASM_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFREERTOS_KERNEL_SMP=0 -DLIB_FREERTOS_KERNEL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_CONFIG_RTOS_ADAPTER_HEADER=/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include/freertos_sdk_config.h -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DEFAULT_UART_RX_PIN=16 -DPICO_DEFAULT_UART_TX_PIN=17 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"pico_def_drive\" -DPICO_USE_BLOCKED_RAM=0

ASM_INCLUDES = -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_stdlib/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_base/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/generated/pico_base -I/home/amir/rspi_pico/pico/pico-sdk/src/boards/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_platform/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_regs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_base/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_structs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_claim/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_irq/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_time/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_timer/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_util/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_resets/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pll/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_runtime/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_printf/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_bit_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_double/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_float/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_malloc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_binary_info/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_flash/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_usb_reset_interface/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_mem_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/boot_stage2/include -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src/common -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/hw -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_multicore/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_exception/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk/libmicroros/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_rand/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pwm/include

ASM_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections

C_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFREERTOS_KERNEL_SMP=0 -DLIB_FREERTOS_KERNEL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_CONFIG_RTOS_ADAPTER_HEADER=/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include/freertos_sdk_config.h -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DEFAULT_UART_RX_PIN=16 -DPICO_DEFAULT_UART_TX_PIN=17 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"pico_def_drive\" -DPICO_USE_BLOCKED_RAM=0

C_INCLUDES = -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_stdlib/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_base/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/generated/pico_base -I/home/amir/rspi_pico/pico/pico-sdk/src/boards/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_platform/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_regs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_base/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_structs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_claim/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_irq/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_time/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_timer/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_util/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_resets/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pll/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_runtime/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_printf/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_bit_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_double/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_float/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_malloc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_binary_info/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_flash/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_usb_reset_interface/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_mem_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/boot_stage2/include -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src/common -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/hw -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_multicore/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_exception/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk/libmicroros/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_rand/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pwm/include

C_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections -std=gnu11

CXX_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFREERTOS_KERNEL_SMP=0 -DLIB_FREERTOS_KERNEL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_MULTICORE=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_CONFIG_RTOS_ADAPTER_HEADER=/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include/freertos_sdk_config.h -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DEFAULT_UART_RX_PIN=16 -DPICO_DEFAULT_UART_TX_PIN=17 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"pico_def_drive\" -DPICO_USE_BLOCKED_RAM=0

CXX_INCLUDES = -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/src -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_stdlib/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_base/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/build/generated/pico_base -I/home/amir/rspi_pico/pico/pico-sdk/src/boards/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_platform/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_regs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_base/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2040/hardware_structs/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_claim/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_irq/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_sync/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_time/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_timer/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_util/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_resets/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pll/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_runtime/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_printf/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_bit_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_divider/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_double/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_float/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_malloc/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_binary_info/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_flash/include -I/home/amir/rspi_pico/pico/pico-sdk/src/common/pico_usb_reset_interface/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_mem_ops/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/boot_stage2/include -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/src/common -I/home/amir/rspi_pico/pico/pico-sdk/lib/tinyusb/hw -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_multicore/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/include/FreeRTOS-Kernel/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_exception/include -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk -I/home/amir/antonio_ws/Antonio/micro_ros_ws/pico_def_drive/../include/micro_ros_raspberrypi_pico_sdk/libmicroros/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/pico_rand/include -I/home/amir/rspi_pico/pico/pico-sdk/src/rp2_common/hardware_pwm/include

CXX_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit -std=gnu++17

