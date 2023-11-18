
add_executable(${ProjectName}
    main.c
    pico_uart_transport.c
)

target_include_directories(${ProjectName} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    pico_multicore
    FreeRTOS-Kernel-Heap4 
    micro_ros
    )

pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(${ProjectName})