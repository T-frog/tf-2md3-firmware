# SDRAM initialization script for the AT91SAM9G20
#------------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# Set PLLA to 800MHz
set *0xFFFFFC28 = 0x202A3F01
while ((*0xFFFFFC68 & 0x2) == 0)
end

# Set PLLB for USB usage
set *0xFFFFFC2C = 0x10193F05
while ((*0xFFFFFC68 & 0x4) == 0)
end

# Select prescaler
set *0xFFFFFC30 = 0x00001300
while ((*0xFFFFFC68 & 0x8) == 0)
end

# Select master clock
set *0xFFFFFC30 = 0x00001302
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo Master clock ok.\n
echo Configuring the SDRAM controller...\n

# Enable EBI chip select for the SDRAM
set *0xFFFFEF1C = 0x2

# Enable PC16-PC31 pins
set *0xFFFFF870 = 0xFFFF0000
set *0xFFFFF874 = 0x00000000
set *0xFFFFF804 = 0xFFFF0000

# SDRAM configuration
set *0xFFFFEA08 = 0x96338379

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

set *0xFFFFEA04 = 0x39D

echo SDRAM configuration ok.\n
