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
echo Configuring the DDRAM controller...\n

# AT91C_BASE_PMC->PMC_SCER = (0x1 << 2);
set *0xFFFFFC00 = 0x00000002

# Ensure that Matrix is connected to enable fetch Data & PDC
set *0xFFFFEB00 = 0x00000007
  
# First, enable the clock of the PIO / Enable PIO A B C D
set *0xFFFFFC00 = 0x00000085

#------------------------------------------------------------------------------------

# Memory Device Register 
# Mobile DDRAM type | 16bit MODE
# ddrc->SDDRC_MDR  = 0x00000013;
set *0xFFFFE61C = 0x00000013   
   
# Configuration Register
# Weak driver strength(1) | Disable DLL reset(0) | SDRAM CAS = 3 | row = 13 | column = 9
# ddrc->SDDRC_CR   = 0x00000138;  
set *0xFFFFE608 = 0x00000138
      
# Timing 0 Parameter Register
# tmrd = 2 | twtr = 1 | trrd = 2 | trrd = 2 | trp = 3 | trc = 8 | twr = 2 | trcd = 3 | tras = 5   
# ddrc->SDDRC_T0PR = 0x21238235;
set *0xFFFFE60C = 0x21238235
      
# Timing 1 Parameter Register
# txp = 4 | txsrd = 0xC | txsnr = 0xC | trfc = 9
# ddrc->SDDRC_T1PR = 0x040C0C09;
set *0xFFFFE610 = 0x040D0D09
   
# Low-power Register   
# Low power register => Low-power is inhibited
# all bank refresh during self refresh (PASR = b000)   
# ddrc->SDDRC_LPR  = 0x00000000; 
set *0xFFFFE618 = 0x00000000

#------------------------------------------------------------------------------------

# NOP command
# ddrc->SDDRC_MR = 0x00000001; 
set *0xFFFFE600 = 0x00000001
set *0x70000000 = 0x00000000

# Precharge All Banks command
# ddrc->SDDRC_MR = 0x00000002;  
set *0xFFFFE600 = 0x00000002
set *0x70000000 = 0x00000000 
   
# AutoRefresh command
# ddrc->SDDRC_MR = 0x00000004;  
set *0xFFFFE600 = 0x00000004
set *0x70000000 = 0x00000000
   
# AutoRefresh command
# ddrc->SDDRC_MR = 0x00000004;   
set *0xFFFFE600 = 0x00000004
set *0x70000000 = 0x00000000

#----------------------------------------------------------------------------------

# Mode Register Set command
# ddrc->SDDRC_MR = 0x00000003;
set *0xFFFFE600 = 0x00000003
set *0x70000000 = 0x00000000
   
# Extended Mode Register Set command
# ddrc->SDDRC_MR = 0x00000005;   
set *0xFFFFE600 = 0x00000005
set *0x100000 = *0x71000000
   
# Set Normal mode
# ddrc->SDDRC_MR = 0x00000000;
set *0xFFFFE600 = 0x00000000
set *0x100000 = *0x70000000
set *0x70000000 = 0x00000000
   
# Set Refresh Timer : ((64 x 10^-3)/8192) x 48 x 10^6   --->  375 for 48 MHz
# Set Refresh Timer : ((64 x 10^-3)/8192) x 100 x 10^6   --->  781 for 100 MHz   
set *0xFFFFE604 = 0x0000044C
   
# High speed register : Optimization is enabled
set *0xFFFFE614 = 0x00000000

echo DDRAM configuration ok.\n
