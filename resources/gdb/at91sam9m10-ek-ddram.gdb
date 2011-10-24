#------------------------------------------------
# DDRAM initialization script for the AT91SAM9M10
#------------------------------------------------

echo Configuring the master clock...\n
# Enable main oscillator
# CKGR_MOR
set *0xFFFFFC20 = 0x00004001
# PMC_SR
while ((*0xFFFFFC68 & 0x1) == 0)
end

echo set plla\n
# Set PLLA to 800MHz
# CKGR_PLLAR
set *0xFFFFFC28 = 0x20C73F03
# PMC_SR
while ((*0xFFFFFC68 & 0x2) == 0)
end
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo select prescaler\n
# Select prescaler
# PMC_MCKR
set *0xFFFFFC30 = 0x00001300
# PMC_SR
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo select master clock\n
# Select master clock
# PMC_MCKR
set *0xFFFFFC30 = 0x00001302
# PMC_SR
while ((*0xFFFFFC68 & 0x8) == 0)
end

echo Master clock ok.\n

echo Configuring the DDRAM controller...\n
echo MICRON DDRAM configuration\n
# 0xFFFFE600 DDR2C Base Address

# Enable DDR2 clock x2 in PMC
# AT91C_BASE_PMC, PMC_SCER, AT91C_PMC_DDR
set *0xFFFFFC00 = 0x04

# Configure the DDR controller
# HDDRSDRC2_MDR, AT91C_DDRC2_DBW_16_BITS   |     // 16-bit DDR
#                AT91C_DDRC2_MD_DDR2_SDRAM       // DDR2
set *0xFFFFE620 = 0x16

# Program the DDR Controller
# HDDRSDRC2_CR, AT91C_DDRC2_NC_DDR10_SDR9  |     // 10 column bits (1K)
#               AT91C_DDRC2_NR_14          |     // 14 row bits    (8K)
#               AT91C_DDRC2_CAS_3          |     // CAS Latency 3
#               AT91C_DDRC2_DLL_RESET_DISABLED   // DLL not reset
set *0xFFFFE608 = 0x3D

# assume timings for 7.5ns min clock period
# HDDRSDRC2_T0PR, AT91C_DDRC2_TRAS_6       |     //  6 * 7.5 = 45   ns
#                 AT91C_DDRC2_TRCD_2       |     //  3 * 7.5 = 22.5 ns
#                 AT91C_DDRC2_TWR_2        |     //  2 * 7.5 = 15   ns
#                 AT91C_DDRC2_TRC_8        |     // 10 * 7.5 = 75   ns
#                 AT91C_DDRC2_TRP_2        |     //  3 * 7.5 = 22.5 ns
#                 AT91C_DDRC2_TRRD_1       |     //  2 * 7.5 = 15   ns
#                 AT91C_DDRC2_TWTR_1       |     //  1 clock cycle
#                 AT91C_DDRC2_TMRD_2             //  2 clock cycles
set *0xFFFFE60C = 0x21128226

# pSDDRC->HDDRSDRC2_T1PR = 0x00000008;
# HDDRSDRC2_T1PR, AT91C_DDRC2_TXP_2  |           //  2 * 7.5 = 15 ns
#                 200 << 16          |           // 200 clock cycles, TXSRD: Exit self refresh delay to Read command
#                 27 << 8            |           // 27 * 7.5 = 202 ns TXSNR: Exit self refresh delay to non read command
#                 AT91C_DDRC2_TRFC_14 << 0       // 19 * 7.5 = 142 ns (must be 140 ns for 1Gb DDR)
set *0xFFFFE610 = 0x02C81B0E

# HDDRSDRC2_T2PR, AT91C_DDRC2_TRTP_2   |         //  2 * 7.5 = 15 ns
#                 AT91C_DDRC2_TRPA_2   |         //  2 * 7.5 = 15 ns
#                 AT91C_DDRC2_TXARDS_7 |         //  7 clock cycles
#                 AT91C_DDRC2_TXARD_7            //  2 clock cycles
set *0xFFFFE614 = 0x02020707

# Initialization Step 1 + 2: NOP command -> allow to enable clk
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD  
set *0xFFFFE600 = 0x1
set *0x70000000 = 0
         
# TODO Initialization Step 3 (must wait 200 us) (6 core cycles per iteration, core is at 396MHz: min 13200 loops)
# for (i = 0; i < 13300; i++) {
#    asm("    nop"  
# }

# NOP command -> allow to enable cke
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD  
set *0xFFFFE600 = 0x1
set *0x70000000 = 0
            
# wait 400 ns min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 4: Set All Bank Precharge
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD  
set *0xFFFFE600 = 0x2
set *0x70000000 = 0

# wait 400 ns min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 5: Set EMR operation (EMRS2)
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD  
set *0xFFFFE600 = 0x5
set *0x74000000 = 0x0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 6: Set EMR operation (EMRS3)
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD
set *0xFFFFE600 = 0x5
set *0x76000000 = 0x0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 7: Set EMR operation (EMRS1)
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD  
set *0xFFFFE600 = 0x5
set *0x72000000 = 0x0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 8a: enable DLL reset
# HDDRSDRC2_CR, cr | AT91C_DDRC2_DLL_RESET_ENABLED
set *0xFFFFE608 |= 0xBD
            
# Initialization Step 8b: reset DLL
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD
set *0xFFFFE600 = 0x5
set *0x70000000 = 0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 9: Set All Bank Precharge
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD
set *0xFFFFE600 = 0x2
set *0x70000000 = 0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 11: Set 1st CBR
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD
set *0xFFFFE600 = 0x4
set *0x70000000 = 0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Set 2nd CBR
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD
set *0xFFFFE600 = 0x4
set *0x70000000 = 0

# wait 2 cycles min
# for (i = 0; i < 100; i++) {
#     asm("    nop"  
# }

# Initialization Step 12: disable DLL reset
# HDDRSDRC2_CR, cr & (~AT91C_DDRC2_DLL_RESET_ENABLED)  
set *0xFFFFE608 = 0x3D

# Initialization Step 13: Set LMR operation
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD
set *0xFFFFE600 = 0x3
set *0x70000000 = 0
         
# Skip Initialization Step 14 to 17 (not supported by the DDR2 model)

# Initialization Step 18: Set Normal mode
# HDDRSDRC2_MR, AT91C_DDRC2_MODE_NORMAL_CMD
set *0xFFFFE600 = 0x0
set *0x70000000 = 0

# Set Refresh timer
# HDDRSDRC2_RTR, 0x00000520
set *0xFFFFE604 = 0x00000520

# OK now we are ready to work on the DDRSDR

# wait for end of calibration
# for (i = 0; i < 500; i++) {
#     asm("    nop"  
# }

echo DDRAM configuration ok.\n

