# xc32-toolchain.cmake
set(CMAKE_SYSTEM_NAME Generic)

# -------------------------
# XC32 Compiler Paths
# -------------------------
set(XC32_BIN "/Applications/microchip/xc32/v5.00/bin")
set(CMAKE_C_COMPILER "${XC32_BIN}/xc32-gcc")
set(CMAKE_CXX_COMPILER "${XC32_BIN}/xc32-g++")

# -------------------------
# Target MCU and DFP
# -------------------------
set(MCU 32MX130F064B)
set(PIC32_DFP "/Applications/microchip/mplabx/v6.30/packs/Microchip/PIC32MX_DFP/1.6.369")

# -------------------------
# Compiler Flags
# -------------------------
set(CMAKE_C_FLAGS "-mprocessor=${MCU} -mdfp=${PIC32_DFP} -O2")
set(CMAKE_CXX_FLAGS "-mprocessor=${MCU} -mdfp=${PIC32_DFP} -O2")

# -------------------------
# Linker Flags
# -------------------------
set(CMAKE_EXE_LINKER_FLAGS "-mprocessor=${MCU} -mdfp=${PIC32_DFP}")