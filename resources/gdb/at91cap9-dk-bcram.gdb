# BCRAM initialization script for the AT91CAP9-DK
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
echo Configuring the BCRAM controller...\n

# Select EBI CSA
set *0xFFFFEB20 = 0x00000002

# Configure A23 and A24 PIO as Periph A
set *0xFFFFF804 = 0x00003000
set *0xFFFFF870 = 0x00003000
            
# The Cellular Ram memory type must be set in the BCRAMC Memory Device Register.
# Burst CellularRAM Version 1.5         
set *0xFFFFE410 = 0x00000001
    
# Temperature compensated self refresh (TCSR) and partial array 
# refresh (PAR) must be set in the BCRAMC Low Power register.    
set *0xFFFFE40C = 0x00000000
       
# High Speed Register
set *0xFFFFE408 = 0x00000000

# Asynchronous timings (TCKA, TCRE..) must be set in the BCRAMC Timing Register.    
set *0xFFFFE404 = 0x00000023
                                      
# Cellular Ram  features must be set in the HBCRAMC Configuration Register:
# number rows, latency, drive strength (DS), the data bus width and cram_enabled bit must be high.       
set *0xFFFFE400 = 0x00001131
        
# __sleep(100000);            
        
# Perform a write to the Cellular Ram  device and the Bus Configuration Register (BCR) and 
# Refresh Configuration Register (RCR) are programmed automatically.         
# Dummy write to access BCRAM : validate preceeding command
set *0x20000000 = 0x00000000

echo BCRAM configuration ok.\n
