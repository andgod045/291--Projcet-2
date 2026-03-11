cmake_minimum_required(VERSION 3.15)
project(controller C)

# -------------------------
# Source files
# -------------------------
set(SOURCES
        controller.c
        joystick.c
        lcd.c
        ir_rc5.c
)

# -------------------------
# Build ELF executable
# -------------------------
add_executable(controller.elf ${SOURCES})

# -------------------------
# Include directories for PIC32MX DFP
# -------------------------
target_include_directories(controller.elf PRIVATE
        /Applications/microchip/mplabx/v6.30/packs/Microchip/PIC32MX_DFP/1.6.369/include/
        /Applications/microchip/mplabx/v6.30/packs/Microchip/PIC32MX_DFP/1.6.369/include/proc/
)

# -------------------------
# Post-build HEX conversion
# -------------------------
add_custom_command(TARGET controller.elf POST_BUILD
        COMMAND /Applications/microchip/xc32/v5.00/bin/xc32-bin2hex $<TARGET_FILE:controller.elf>
        COMMENT "Generating HEX file from ELF..."
)