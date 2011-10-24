# SDRAM initialization script for the AT91CAP9-DK
#------------------------------------------------
# Configure master clock
echo Configuring the master clock...\n
# Enable main oscillator
set *0xFFFFFC20 = 0x00004001
while ((*0xFFFFFC68 & 0x1) == 0)
end

# set PLLA to 200MHz
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

# configure pins Sdram
set *0xFFFFF870 = 0xFFFF0000
set *0xFFFFF874 = 0x00000000
set *0xFFFFF804 = 0xFFFF0000

# 32bit MODE; SDRAM type
set *0xFFFFE61C = 0x00000000

# row = 13 =  column = 9 SDRAM CAS = 3
set *0xFFFFE608 = 0x00000039

# Low power register => Low-power is inhibited
set *0xFFFFE618 = 0x00000000

# NOP command
set *0xFFFFE600 = 0x1
set *0x70000000 = 0

# NOP command
set *0xFFFFE600 = 0x1
set *0x70000000 = 0

# NOP command
set *0xFFFFE600 = 0x1
set *0x70000000 = 0

# Precharge All Banks command
set *0xFFFFE600 = 0x2
set *0x70000000 = 0

# AutoRefresh command
set *0xFFFFE600 = 0x4 
set *0x70000000 = 0

# AutoRefresh command
set *0xFFFFE600 = 0x4 
set *0x70000000 = 0

# set MR JEDEC compliant : Load mode Register command
set *0xFFFFE600 = 0x3 
set *0x70000000 = 0

# set Normal mode : Any access to the DDRSDRAMC is decoded normally
set *0xFFFFE600 = 0x0
set *0x70000000 = 0

# set Refresh Timer (ex: ((64 x 10^-3)/8192) x 50 x 10^6 )         # 781 for 100 MHz
set *0xFFFFE604 = 781

# High speed register : Optimization is disabled
set *0xFFFFE614 = 0x2

echo SDRAM configuration ok.\n
