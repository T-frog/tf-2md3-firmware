# SDRAM initialization script for the AT91SAM9RL
#------------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# Set PLL to 200MHz
set *0xFFFFFC28 = 0x2031BF03
while ((*0xFFFFFC68 & 0x2) == 0)
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
set *0xFFFFEF20 = 0x2

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

set *0xFFFFEA04 = 0x2BC

echo SDRAM configuration ok.\n