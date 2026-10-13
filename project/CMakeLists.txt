cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# NOTE: this file is generated only once by CubeMX and never overwritten,
#       so user sources go here -- NOT in cmake/stm32cubemx/CMakeLists.txt,
#       which is regenerated on every CubeMX code generation.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/signal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/signal_slow.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/uart_echo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/adc_vofa.c

    # 频率跟踪（锁频）：ADC 采对方的波 → 测频 → DAC 输出同频正弦
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/track.c

    # SEGGER RTT 调试输出（走 SWD 内存轮询，不占 UART / 引脚）
    # ★ 放在 Port/ 而不是 Middlewares/ —— CubeMX 重新生成时会把 Middlewares/
    #   整个删掉（2026-10-12 实测，RTT 中间件被清空导致工程编译不过）。
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/debug_rtt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/SEGGER_RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/SEGGER_RTT/SEGGER_RTT_printf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/SEGGER_RTT/SEGGER_RTT_ASM_ARMv7M.S
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Port
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/SEGGER_RTT
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Generate the exact image used by the flash step after EVERY successful link.
# Do not create project.hex in the source root: that is NOT the flash input.
#
# Only the HEX is generated here. The post-flash read-back verification parses
# this same HEX and compares exactly the bytes it programs, so no intermediate
# binary is needed. (An earlier attempt generated a --gap-fill binary instead:
# that made objcopy size the output by a RAM section VMA and produced a ~400 MB
# file, which is exactly the kind of silent mismatch this whole flow exists to
# prevent. Parsing the HEX has no such failure mode.)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/project.hex
    COMMENT "Generating the flash image from the linked ELF"
    VERBATIM
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
