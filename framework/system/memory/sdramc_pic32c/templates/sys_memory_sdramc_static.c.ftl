/*******************************************************************
  Company:
    Microchip Technology Inc.
    Memory System Service SDRAMC Initialization File

  File Name:
    sys_memory_sdramc_static.c.ftl

  Summary:
    SDRAMC Controller (SDRAMC). 

  Description:
    The SDRAMC System Memory interface provides a simple interface to manage
    the externally connected SDRAM device. This file contains the source code
    to initialize the SDRAMC controller

  *******************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

<#-- SDRAMC Instance 0 -->
<#if CONFIG_USE_SYS_MEMORY_SDRAMC == true>
#include "framework/system/memory/sdramc_pic32c/src/sys_memory_sdramc_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global data Definitions
// *****************************************************************************
// *****************************************************************************
SYS_SDRAMC_INIT sdramcInit;

// *****************************************************************************
// *****************************************************************************
// Section: SDRAMC Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

/* Function:
    void SYS_MEMORY_SDRAMC_Initialize( void )

  Summary:
    Initializes hardware and data for the given instance of the SDRAMC module.

  Description:
    This function initializes the SDRAM Controller to manage the externally
    connected SDRAM device.

  Returns:
  None.
 */

void SYS_MEMORY_SDRAMC_Initialize
(
    void
)
{
    volatile uint32_t config = 0;
    volatile uint32_t i = 0;
    uint8_t *pSdramBaseAddress = (uint8_t *)SDRAM_CS_ADDR;
    uint32_t clock = 0;

    /* Enable the SDRAMC clock. */
    _PMC_REGS->PMC_PCER1.w |= 1 << (ID_SDRAMC - 32);

    /* Enable the SDRAMC Chip Select */
    _MATRIX_REGS->CCFG_SMCNFCS.w |= (1 << CCFG_SMCNFCS_SDRAMEN_Pos);


    /* Step 1: Configure the SDRAMC Configuration Register. */
    config = (SDRAMC_CR_NC(SYS_MEMORY_SDRAMC_NUM_COLS) |
            SDRAMC_CR_NR(SYS_MEMORY_SDRAMC_NUM_ROWS) |
            ((SYS_MEMORY_SDRAMC_NUM_BANKS & 0x01) << SDRAMC_CR_NB_Pos) |
            SDRAMC_CR_CAS(SYS_MEMORY_SDRAMC_CAS_LATENCY) |
            (1 << SDRAMC_CR_DBW_Pos) |
            SDRAMC_CR_TWR(SYS_MEMORY_SDRAMC_TWR_DELAY) |
            SDRAMC_CR_TRC_TRFC(SYS_MEMORY_SDRAMC_TRC_TRFC_DELAY) |
            SDRAMC_CR_TRP(SYS_MEMORY_SDRAMC_TRP_DELAY) |
            SDRAMC_CR_TRCD(SYS_MEMORY_SDRAMC_TRCD_DELAY) |
            SDRAMC_CR_TRAS(SYS_MEMORY_SDRAMC_TRAS_DELAY));
    _SDRAMC_REGS->SDRAMC_CR.w = config;

    /* Support unaligned access. */
    config = SDRAMC_CFR1_TMRD(SYS_MEMORY_SDRAMC_TMRD_DELAY) | (1 << SDRAMC_CFR1_UNAL_Pos);
    _SDRAMC_REGS->SDRAMC_CFR1.w = config;


    /* Step 2: Configure the Low Power Register for mobile/Low power SDRAM
     * devices. */
    <#if CONFIG_SYS_SDRAMC_DEVICE_TYPE == "Low-power SDRAM">
    config = (SDRAMC_LPR_LPCB(SYS_MEMORY_SDRAMC_LOW_POWER_CONFIG) |
            SDRAMC_LPR_PASR(SYS_MEMORY_SDRAMC_PASR) |
            SDRAMC_LPR_TCSR(SYS_MEMORY_SDRAMC_TCSR) |
            SDRAMC_LPR_DS(SYS_MEMORY_SDRAMC_DS) |
            SDRAMC_LPR_TIMEOUT(SYS_MEMORY_SDRAMC_LOW_POWER_TIMEOUT));

    _SDRAMC_REGS->SDRAMC_LPR.w = config;
    _SDRAMC_REGS->SDRAMC_CR.w |= SDRAMC_CR_TXSR(SYS_MEMORY_SDRAMC_TXSR_DELAY);
    <#else>
    /* Set the LPR to zero. */
    _SDRAMC_REGS->SDRAMC_LPR.w = 0;
    </#if>


    /* Step 3: Select the SDRAM memory device type. */
    _SDRAMC_REGS->SDRAMC_MDR.MD = SYS_MEMORY_SDRAMC_DEVICE_TYPE;


    /* Step 4: A pause of atleast 200 us must be observed before a signal
     * toggle. */
    clock = SYS_CLK_FrequencyGet (SYS_CLK_PROCESSOR);
    /* A minimum pause of 200 us is provided to precede any signal toggle. (6
     * core cycles per iteration). */
    for (i = 0; i < ((clock / 1000000) * 1000 / 6); i++)
    {
        ;
    }


    /* Step 5: Issue a NOP command to the SDRAM device by writing to the Mode
     * Register. Read back the Mode Register and add a memory barrier assembly
     * instruction just after the read. Perform a write access to any SDRAM
     * address. */
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_NOP_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    *pSdramBaseAddress = 0x0;


    /* Step 6: Issue an All banks precharge command by writing to the Mode
     * Register. Read back the Mode Register and add a memory barrier assembly
     * instruction just after the read. Perform a write access to any SDRAM
     * address. */
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_ALLBANKS_PRECHARGE_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    *pSdramBaseAddress = 0x0;


    /* Step 7: Issue 8 Auto Refresh(CBR) cycles by writing to the Mode
     * Register. Read back the Mode Register and add a memory barrier assembly
     * instruction just after the read. Perform a write access to any SDRAM
     * address 8 times. */
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_AUTO_REFRESH_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    for (i = 0; i < 8; i++)
    {
        *pSdramBaseAddress = i;
    }


    /* Step 8: Issue Mode Register Set(MRS) to program the parameters of the
     * SDRAM. Read back the Mode Register and add a memory barrier assembly
     * instruction just after the read. Perform a write access to the SDRAM.
     * The address must be chosen such that BA[1:0] are set to zero. */
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_LOAD_MODEREG_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    /* This configures the SDRAM with the following parameters in the mode
     * register:
     * - bits 0 to 2: burst length
     * - bit 3: burst type: sequential
     * - bits 4 to 6: CAS latency
     * - bits 7 to 8: operating mode: standard operation (00b);
     * - bit 9: write burst mode: programmed burst length (0b);
     * - all other bits: reserved: 0b.
     */
    config = (SYS_MEMORY_SDRAMC_CAS_LATENCY << 4) | (SYS_MEMORY_SDRAMC_BURST_TYPE << 3) | SYS_MEMORY_SDRAMC_BURST_LENGTH;
    *((uint16_t *)(pSdramBaseAddress + config)) = 0x0;


    /* Step 9: Issue Extended Mode Register Set(EMRS) for mobile SDRAM to
     * program the parameters of the SDRAM(TCSR, PASR, DS). Read back the Mode
     * Register and add a memory barrier assembly instruction just after the
     * read. Perform a write access to the SDRAM. The address must be chosen
     * such that BA[1] or BA[0] are set to one. */
    <#if CONFIG_SYS_SDRAMC_DEVICE_TYPE == "Low-power SDRAM">
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_EXT_LOAD_MODEREG_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    /* Address line consists of M0, Column, Row and BA. */
    config = (SYS_MEMORY_SDRAMC_NUM_ROWS_BITS + SYS_MEMORY_SDRAMC_NUM_COLS_BITS + 1);
    pSdramBaseAddress = (uint8_t *)(SDRAM_CS_ADDR | (1 << config));
    *pSdramBaseAddress = 0x0;
    </#if>


    /* Step 10: Transition the SDRAM to NORMAL mode. Read back the Mode
     * Register and add a memory barrier assembly instruction just after the
     * read. Perform a write access to any SDRAM address. */
    _SDRAMC_REGS->SDRAMC_MR.MODE = SDRAMC_MR_MODE_NORMAL_Val;
    config = _SDRAMC_REGS->SDRAMC_MR.w;
    __DMB();
    *pSdramBaseAddress = 0x0;


    /* Step 11: Configure the Refresh Timer Register. */
    clock = SYS_CLK_FrequencyGet (SYS_CLK_MASTER);
    clock /= 1000;
    config = (clock * 32) / (1 << SYS_MEMORY_SDRAMC_NUM_ROWS_BITS);
    _SDRAMC_REGS->SDRAMC_TR.COUNT = config;

    <#if CONFIG_SYS_SDRAMC_MEM_SCRAMBLING == true>

    /* Enable the Off-chip Memory scrambling. */    
    _SDRAMC_REGS->SDRAMC_OCMS_KEY1.KEY1 = ${CONFIG_SYS_SDRAMC_OCMS_KEY1};
    _SDRAMC_REGS->SDRAMC_OCMS_KEY2.KEY2 = ${CONFIG_SYS_SDRAMC_OCMS_KEY2};
    _SDRAMC_REGS->SDRAMC_OCMS.SDR_SE = SDRAMC_OCMS_SDR_SE_Msk;
    </#if>

} /* SYS_MEMORY_SDRAMC_Initialize */

</#if>

/*******************************************************************************
 End of File
*/
