# SDRAM initialization script for the AT91SAM9XE
#------------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# Set PLLA to 200MHz
set *0xFFFFFC28 = 0x2060BF09
while ((*0xFFFFFC68 & 0x2) == 0)
end

# Set PLLB for USB usage
set *0xFFFFFC2C = 0x207C7F0C
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

# Enable EBI chip select for the SDRAM
set *0xFFFFEF1C = 0x2

# SDRAM configuration
set *0xFFFFEA08 = 0x85227259

set *0xFFFFEA00 = 0x1
set *0x20000000 = 0

set *0xFFFFEA00 = 0x2
set *0x20000000 = 0

set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0
set *0xFFFFEA00 = 0x4
set *0x20000000 = 0

set *0xFFFFEA00 = 0x3
set *0x20000000 = 0

set *0xFFFFEA00 = 0x0
set *0x20000000 = 0

set *0xFFFFEA04 = 0x2B7

echo SDRAM configuration ok.\n
