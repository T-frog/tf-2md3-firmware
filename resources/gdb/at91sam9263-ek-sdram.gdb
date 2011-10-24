# SDRAM initialization script for the AT91SAM9263
#------------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# Set PLLA to 200MHz
set *0xFFFFFC28 = 0x206DBF09
while ((*0xFFFFFC68 & 0x2) == 0)
end

# Set PLLB for USB usage
set *0xFFFFFC2C = 0x20AF3F0F
while ((*0xFFFFFC68 & 0x4) == 0)
end

# Select prescaler
set *0xFFFFFC30 = 0x00000100
while ((*0xFFFFFC68 & 0x8) == 0)
end

# Select master clock
set *0xFFFFFC30 = 0x00000102
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo Master clock ok.\n
echo Configuring the SDRAM controller...\n

# Configure PIOD as peripheral (D16/D31)
#    __writeMemory32(0xFFFF0000,0xFFFFF870,"Memory");
set *0xFFFFF870 = 0xFFFF0000
#    __writeMemory32(0x00000000,0xFFFFF874,"Memory");
set *0xFFFFF874 = 0x00000000
#    __writeMemory32(0xFFFF0000,0xFFFFF804,"Memory");
set *0xFFFFF804 = 0xFFFF0000

# Enable EBI chip select for the SDRAM
set *0xFFFFED20 = 0x2

# SDRAM configuration
set *0xFFFFE208 = 0x85227259

set *0xFFFFE200 = 0x1
set *0x20000000 = 0

set *0xFFFFE200 = 0x2
set *0x20000000 = 0

set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0
set *0xFFFFE200 = 0x4
set *0x20000000 = 0

set *0xFFFFE200 = 0x3
set *0x20000000 = 0

set *0xFFFFE200 = 0x0
set *0x20000000 = 0

set *0xFFFFE204 = 0x2B7

echo SDRAM configuration ok.\n
