/*******************************************************************
  Company:
    Microchip Technology Inc.
    Memory System Service SMC Initialization File

  File Name:
    sys_memory_smc_static.c.ftl

  Summary:
    Static Memory Controller (SMC).
  This file contains the source code to initialize the SMC controller

  Description:
    SMC configuration interface
    The SMC System Memory interface provides a simple interface to manage 8-bit and 16-bit
    parallel devices.

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

<#-- SMC Instance 0 -->
<#if CONFIG_USE_SYS_MEMORY_SMC == true>
#include "framework/system/memory/smc_pic32c/src/sys_memory_smc_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global data Definitions
// *****************************************************************************
// *****************************************************************************
SYS_SMC_INIT smcInit;

// *****************************************************************************
// *****************************************************************************
// Section: SMC Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
<#if CONFIG_USE_BSP== false>
/* Function:
static void SYS_MEMORY_SMC_SetupPort( void )

  Summary:
    Setup the SMC lines from the initialization structure.

  Description:
    This function sets up the hardware according to the interface settings and the PORTs

  Remarks:
    None.
*/
void SYS_MEMORY_SMC_SetupPort( void )
{
  uint32_t buswidth;

  buswidth=0;

   /* Set the peripheral function for the SMC Mux signals */
  //******************************************************************************
  /* Chip Select */
  //******************************************************************************
<#if CONFIG_SYS_MEMORY_SMC_CS0 == true>
  /* Enable SMC Chip select on PC14 - Peripheral A PORT0 */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w |= PORT_PDR_P14_Msk;
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_ABCDSR_P14_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P14_Msk);

  <#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS0 == "SMC_DATA_BUS_WIDTH_16BIT">
  if (buswidth == 0)
  {
    /* Enable  8 MSB for 16-bit data bus width */
    buswidth=1;
    /* on PE0-5 - Peripheral A  PORT4*/
    VCAST(port_registers_t, PORTS_ID_4)->PORT_PDR.w |= (PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    /* on PA15-16 - Peripheral A  PORT0*/
    VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= (PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
  }
  </#if>
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS1 == true>
  /* Enable SMC Chip select on PC15 - Peripheral A */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w |= PORT_PDR_P15_Msk;
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_ABCDSR_P15_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P15_Msk);

  /* Enable SMC Chip select on PD18 - Peripheral A PORT3 */
  VCAST(port_registers_t, PORTS_ID_3)->PORT_PDR.w |= PORT_PDR_P18_Msk;
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[0].w &= ~(PORT_ABCDSR_P18_Msk);
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P18_Msk);

<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS1 == "SMC_DATA_BUS_WIDTH_16BIT">
  if (buswidth == 0)
  {
    /* Enable  8 MSB for 16-bit data bus width */
    buswidth=1;
    /* on PE0-5 - Peripheral A PORT4 */
    VCAST(port_registers_t, PORTS_ID_4)->PORT_PDR.w |= (PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    /* on PA15-16 - Peripheral A PORT0 */
    VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= (PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
  }
  </#if>
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS2 == true>
  /* Enable SMC Chip select on PA22 - Peripheral C PORT0 */
  VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= PORT_PDR_P22_Msk;
  VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w |= PORT_ABCDSR_P22_Msk;
  VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P22_Msk);

  <#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS2 == "SMC_DATA_BUS_WIDTH_16BIT">
  if (buswidth == 0)
  {
    /* Enable  8 MSB for 16-bit data bus width */
    buswidth=1;
    /* on PE0-5 - Peripheral A -PORT4 */
    VCAST(port_registers_t, PORTS_ID_4)->PORT_PDR.w |= (PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk| PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    /* on PA15-16 - Peripheral A PORT0 */
    VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= (PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
  }
  </#if>
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS3 == true>
  /* Enable SMC Chip select on PC12 - Peripheral A PORT2 */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w |= PORT_PDR_P12_Msk;
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_ABCDSR_P12_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P12_Msk);

  /* Enable SMC Chip select on PD19 - Peripheral A PORT3 */
  VCAST(port_registers_t, PORTS_ID_3)->PORT_PDR.w |= PORT_PDR_P19_Msk;
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[0].w &= ~(PORT_ABCDSR_P19_Msk);
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[1].w &= ~(PORT_ABCDSR_P19_Msk);

<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS3 == "SMC_DATA_BUS_WIDTH_16BIT">
  if (buswidth == 0)
  {
    /* Enable  8 MSB for 16-bit data bus width */
    buswidth=1;
    /*on PE0-5 - Peripheral A PORT4*/
    VCAST(port_registers_t, PORTS_ID_4)->PORT_PDR.w |= (PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk| PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    VCAST(port_registers_t, PORTS_ID_4)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk);
    /*on PA15-16 - Peripheral A  PORT0 */
    VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= (PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
    VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P15_Msk | PORT_PDR_P16_Msk);
  }
  </#if>
</#if>

  //******************************************************************************
  /* Address lines */
  //******************************************************************************
  /* Enable Address line on PC16 PC19-31 - Peripheral A PORT2 */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w|= (PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk | PORT_PDR_P21_Msk | PORT_PDR_P22_Msk | PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk | PORT_PDR_P26_Msk | PORT_PDR_P27_Msk | PORT_PDR_P28_Msk | PORT_PDR_P29_Msk | PORT_PDR_P30_Msk | PORT_PDR_P31_Msk | PORT_PDR_P16_Msk | PORT_PDR_P17_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk | PORT_PDR_P21_Msk | PORT_PDR_P22_Msk | PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk | PORT_PDR_P26_Msk | PORT_PDR_P27_Msk | PORT_PDR_P28_Msk | PORT_PDR_P29_Msk | PORT_PDR_P30_Msk | PORT_PDR_P31_Msk | PORT_PDR_P16_Msk | PORT_PDR_P17_Msk );
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk | PORT_PDR_P21_Msk | PORT_PDR_P22_Msk | PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk | PORT_PDR_P26_Msk | PORT_PDR_P27_Msk | PORT_PDR_P28_Msk | PORT_PDR_P29_Msk | PORT_PDR_P30_Msk | PORT_PDR_P31_Msk | PORT_PDR_P16_Msk | PORT_PDR_P17_Msk);
  /* Enable Address line on PA0-1 PA18-20 PA23-25- Peripheral C PORT0*/
  VCAST(port_registers_t, PORTS_ID_0)->PORT_PDR.w |= (PORT_PDR_P0_Msk |PORT_PDR_P1_Msk | PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk| PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk);
  VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[0].w |= (PORT_PDR_P0_Msk |PORT_PDR_P1_Msk | PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk| PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk );
  VCAST(port_registers_t, PORTS_ID_0)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk |PORT_PDR_P1_Msk | PORT_PDR_P18_Msk |PORT_PDR_P19_Msk | PORT_PDR_P20_Msk| PORT_PDR_P23_Msk | PORT_PDR_P24_Msk | PORT_PDR_P25_Msk);

  //******************************************************************************
  /* Data lines */
  //******************************************************************************
  /* Enable  8 LSB on PC0-7 - Peripheral A PORT2 */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w |= (PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk | PORT_PDR_P6_Msk | PORT_PDR_P7_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk | PORT_PDR_P6_Msk | PORT_PDR_P7_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P0_Msk| PORT_PDR_P1_Msk | PORT_PDR_P2_Msk | PORT_PDR_P3_Msk | PORT_PDR_P4_Msk | PORT_PDR_P5_Msk | PORT_PDR_P6_Msk | PORT_PDR_P7_Msk);

  //******************************************************************************
  /* NRD,NWAIT,NWR0,NWR1 lines */
  //******************************************************************************

  /* Enable  NRD, NWAIT,NWR0 PC11-13-8 - Peripheral A  PORT2 */
  VCAST(port_registers_t, PORTS_ID_2)->PORT_PDR.w |= (PORT_PDR_P11_Msk| PORT_PDR_P13_Msk | PORT_PDR_P8_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[0].w &= ~(PORT_PDR_P11_Msk| PORT_PDR_P13_Msk | PORT_PDR_P8_Msk);
  VCAST(port_registers_t, PORTS_ID_2)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P11_Msk| PORT_PDR_P13_Msk | PORT_PDR_P8_Msk);
  /* Enable  NWR1 PD15 Peripheral C PORT3 */
  VCAST(port_registers_t, PORTS_ID_3)->PORT_PDR.w |= (PORT_PDR_P15_Msk);
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[0].w |= (PORT_PDR_P15_Msk);
  VCAST(port_registers_t, PORTS_ID_3)->PORT_ABCDSR[1].w &= ~(PORT_PDR_P15_Msk);
}

</#if>

/* Function:
    void SYS_MEMORY_SMC_Initialize( void )

  Summary:
    Initializes hardware and data for the given instance of the SMC module.

  Description:
    This function initializes the SMC timings according to the external parralel device requirements.

  Returns:
  None.
 */

void SYS_MEMORY_SMC_Initialize( void )
{
    volatile uint32_t config = 0x0;

<#if CONFIG_USE_BSP == false>
    /* Configuration of SMC PORT lines */
    SYS_MEMORY_SMC_SetupPort();
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS0 == true>
    /* Chip Select CS0 Timings */
    /* Setup SMC SETUP register */
    config = 0x01010101; /* Setup register reset value */
    config = SMC_SETUP_NWE_SETUP(SMC_NWE_SETUP_CS0)|SMC_SETUP_NCS_WR_SETUP(SMC_NCS_WR_SETUP_CS0)|SMC_SETUP_NRD_SETUP(SMC_NRD_SETUP_CS0) |SMC_SETUP_NCS_RD_SETUP(SMC_NCS_RD_SETUP_CS0);
    _SMC_REGS->SMC_CS_NUMBER[0].SMC_SETUP.w = (uint32_t)(config << 0);

    /* Setup SMC CYCLE register */
    config = 0x00030003; /* Cycle register reset value */
    config = SMC_CYCLE_NWE_CYCLE(SMC_NWE_CYCLE_CS0) | SMC_CYCLE_NRD_CYCLE(SMC_NRD_CYCLE_CS0);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS0].SMC_CYCLE.w = (uint32_t)config;

    /* Setup SMC_PULSE register */
    config = 0x01010101; /* Pulse register reset value */
    config = SMC_PULSE_NWE_PULSE(SMC_NWE_PULSE_CS0) | SMC_PULSE_NCS_WR_PULSE(SMC_NCS_WR_PULSE_CS0) | SMC_PULSE_NRD_PULSE(SMC_NRD_PULSE_CS0) | SMC_PULSE_NCS_RD_PULSE(SMC_NCS_RD_PULSE_CS0);
    _SMC_REGS->SMC_CS_NUMBER[0].SMC_PULSE.w = (uint32_t)config;

    /* Setup SMC MODE register */
    config = 0x10001003; /* Pulse register reset value */
    config = (( SMC_MODE_READ_MODE_Msk & ((SMC_READ_MODE_CS0) <<  SMC_MODE_READ_MODE_Pos))|(SMC_MODE_WRITE_MODE_Msk & ((SMC_WRITE_MODE_CS0) <<  SMC_MODE_WRITE_MODE_Pos))| SMC_NWAIT_MODE_CS0 | (SMC_MODE_TDF_MODE_Msk & ((SMC_TDF_MODE_CS0) << SMC_MODE_TDF_MODE_Pos)) | SMC_MODE_TDF_CYCLES(SMC_TDF_CYCLES_CS0));

    /* Byte Access Type  setup , used only in 16-bit data bus */
    if (SMC_DATA_BUS_CS0 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS0 == 0x1)
    {
        config |=SMC_DATA_BUS_CS0 |SMC_MODE_BAT_BYTE_WRITE;
    }
    if (SMC_DATA_BUS_CS0 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS0 == 0x0)
    {
        config |=SMC_DATA_BUS_CS0 |SMC_MODE_BAT_BYTE_SELECT;
    }
    if (SMC_DATA_BUS_CS0 ==  SMC_DATA_BUS_WIDTH_8BIT)
    {
        config |=SMC_DATA_BUS_CS0;
    }

    if (SMC_PMEN_CS0 == ASYNC_READ_PAGE_MODE)
    {
        config |= ASYNC_READ_PAGE_MODE | SMC_PS_CS0;
    }
    else
    {
        config |= STD_READ_PAGE_MODE;
    }
    _SMC_REGS->SMC_CS_NUMBER[0].SMC_MODE.w = (uint32_t)config;
    config = 0x0;
    /* Enable Off-chip Memory Scrambling */
    if (SMC_MEM_SCRAMBLING_CS0 == 0x1)
    {
        _SMC_REGS->SMC_OCMS.w |=SMC_OCMS_CS0SE_Msk;
    }
    else // SMC_MEM_SCRAMBLING_CS0 == 0x0
    {
        _SMC_REGS->SMC_OCMS.w &=~(SMC_OCMS_CS0SE_Msk);
    }
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS1 == true>
    /* Chip Select CS1 Timings */
    /* Setup SMC SETUP register */
    config = 0x01010101; /* Setup register reset value */
    config = SMC_SETUP_NWE_SETUP(SMC_NWE_SETUP_CS1)|SMC_SETUP_NCS_WR_SETUP(SMC_NCS_WR_SETUP_CS1)|SMC_SETUP_NRD_SETUP(SMC_NRD_SETUP_CS1) |SMC_SETUP_NCS_RD_SETUP(SMC_NCS_RD_SETUP_CS1);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS1].SMC_SETUP.w = (uint32_t)(config << 0);

    /* Setup SMC CYCLE register */
    config = 0x00030003; /* Cycle register reset value */
    config = SMC_CYCLE_NWE_CYCLE(SMC_NWE_CYCLE_CS1) | SMC_CYCLE_NRD_CYCLE(SMC_NRD_CYCLE_CS1);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS1].SMC_CYCLE.w = (uint32_t)config;

    /* Setup SMC_PULSE register */
    config = 0x01010101; /* Pulse register reset value */
    config = SMC_PULSE_NWE_PULSE(SMC_NWE_PULSE_CS1) | SMC_PULSE_NCS_WR_PULSE(SMC_NCS_WR_PULSE_CS1) | SMC_PULSE_NRD_PULSE(SMC_NRD_PULSE_CS1) | SMC_PULSE_NCS_RD_PULSE(SMC_NCS_RD_PULSE_CS1);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS1].SMC_PULSE.w = (uint32_t)config;

    /* Setup SMC MODE register */
    config = 0x10001003; /* Pulse register reset value */
    config = (( SMC_MODE_READ_MODE_Msk & ((SMC_READ_MODE_CS1) <<  SMC_MODE_READ_MODE_Pos))|(SMC_MODE_WRITE_MODE_Msk & ((SMC_WRITE_MODE_CS1) <<  SMC_MODE_WRITE_MODE_Pos))| SMC_NWAIT_MODE_CS1 | (SMC_MODE_TDF_MODE_Msk & ((SMC_TDF_MODE_CS1) << SMC_MODE_TDF_MODE_Pos)) | SMC_MODE_TDF_CYCLES(SMC_TDF_CYCLES_CS1));

    /* Byte Access Type  setup , used only in 16-bit data bus */
    if (SMC_DATA_BUS_CS1 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS1== 0x1)
    {
        config |=SMC_DATA_BUS_CS1 |SMC_MODE_BAT_BYTE_WRITE;
    }
    if (SMC_DATA_BUS_CS1 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS1== 0x0)
    {
        config |=SMC_DATA_BUS_CS1 |SMC_MODE_BAT_BYTE_SELECT;
    }
    if (SMC_DATA_BUS_CS1 == SMC_DATA_BUS_WIDTH_8BIT)
    {
        config |=SMC_DATA_BUS_CS1;
    }

    if (SMC_PMEN_CS1 == ASYNC_READ_PAGE_MODE)
    {
      config |= ASYNC_READ_PAGE_MODE | SMC_PS_CS1;
    }
    else
    {
      config |= STD_READ_PAGE_MODE;
    }
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS1].SMC_MODE.w = (uint32_t)config;
    config = 0x0;

    if (SMC_MEM_SCRAMBLING_CS1 == 0x1)
    {
        _SMC_REGS->SMC_OCMS.w |=SMC_OCMS_CS1SE_Msk;
    }
    else // SMC_MEM_SCRAMBLING_CS1 == 0x0
    {
      _SMC_REGS->SMC_OCMS.w &=~(SMC_OCMS_CS1SE_Msk);
    }
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS2 == true>
    /* Chip Select CS2 Timings */
    /* Setup SMC SETUP register */
    config = 0x01010101; /* Setup register reset value */
    config = SMC_SETUP_NWE_SETUP(SMC_NWE_SETUP_CS2)|SMC_SETUP_NCS_WR_SETUP(SMC_NCS_WR_SETUP_CS2)|SMC_SETUP_NRD_SETUP(SMC_NRD_SETUP_CS2) |SMC_SETUP_NCS_RD_SETUP(SMC_NCS_RD_SETUP_CS2);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS2].SMC_SETUP.w = (uint32_t)(config << 0);

    /* Setup SMC CYCLE register */
    config = 0x00030003; /* Cycle register reset value */
    config = SMC_CYCLE_NWE_CYCLE(SMC_NWE_CYCLE_CS2) | SMC_CYCLE_NRD_CYCLE(SMC_NRD_CYCLE_CS2);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS2].SMC_CYCLE.w = (uint32_t)config;

    /* Setup SMC_PULSE register */
    config = 0x01010101; /* Pulse register reset value */
    config = SMC_PULSE_NWE_PULSE(SMC_NWE_PULSE_CS2) | SMC_PULSE_NCS_WR_PULSE(SMC_NCS_WR_PULSE_CS2) | SMC_PULSE_NRD_PULSE(SMC_NRD_PULSE_CS2) | SMC_PULSE_NCS_RD_PULSE(SMC_NCS_RD_PULSE_CS2);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS2].SMC_PULSE.w = (uint32_t)config;

    /* Setup SMC MODE register */
    config = 0x10001003; /* Pulse register reset value */
    config = (( SMC_MODE_READ_MODE_Msk & ((SMC_READ_MODE_CS2) <<  SMC_MODE_READ_MODE_Pos))|(SMC_MODE_WRITE_MODE_Msk & ((SMC_WRITE_MODE_CS2) <<  SMC_MODE_WRITE_MODE_Pos))| SMC_NWAIT_MODE_CS2 | (SMC_MODE_TDF_MODE_Msk & ((SMC_TDF_MODE_CS2) << SMC_MODE_TDF_MODE_Pos)) | SMC_MODE_TDF_CYCLES(SMC_TDF_CYCLES_CS2));

    /* Byte Access Type  setup , used only in 16-bit data bus */
    if (SMC_DATA_BUS_CS2 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS2== 0x1)
    {
        config |=SMC_DATA_BUS_CS2 |SMC_MODE_BAT_BYTE_WRITE;
    }
    if (SMC_DATA_BUS_CS2 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS2== 0x0)
    {
        config |=SMC_DATA_BUS_CS2 |SMC_MODE_BAT_BYTE_SELECT;
    }
    if (SMC_DATA_BUS_CS2 == SMC_DATA_BUS_WIDTH_8BIT)
    {
        config |=SMC_DATA_BUS_CS2;
    }

    if (SMC_PMEN_CS2 == ASYNC_READ_PAGE_MODE)
    {
      config |= ASYNC_READ_PAGE_MODE | SMC_PS_CS2;
    }
    else
    {
        config |= STD_READ_PAGE_MODE;
    }
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS2].SMC_MODE.w = (uint32_t)config;
    config = 0x0;
    if (SMC_MEM_SCRAMBLING_CS2 == 0x1)
    {
        _SMC_REGS->SMC_OCMS.w |=SMC_OCMS_CS2SE_Msk;
    }
    else // SMC_MEM_SCRAMBLING_CS2 == 0x0
    {
        _SMC_REGS->SMC_OCMS.w &=~(SMC_OCMS_CS2SE_Msk);
    }
</#if>

<#if CONFIG_SYS_MEMORY_SMC_CS3 == true>
    /* Chip Select CS3 Timings */
    /* Setup SMC SETUP register */
    config = 0x01010101; /* Setup register reset value */
    config = SMC_SETUP_NWE_SETUP(SMC_NWE_SETUP_CS3)|SMC_SETUP_NCS_WR_SETUP(SMC_NCS_WR_SETUP_CS3)|SMC_SETUP_NRD_SETUP(SMC_NRD_SETUP_CS3) |SMC_SETUP_NCS_RD_SETUP(SMC_NCS_RD_SETUP_CS3);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS3].SMC_SETUP.w = (uint32_t)(config << 0);

    /* Setup SMC CYCLE register */
    config = 0x00030003; /* Cycle register reset value */
    config = SMC_CYCLE_NWE_CYCLE(SMC_NWE_CYCLE_CS3) | SMC_CYCLE_NRD_CYCLE(SMC_NRD_CYCLE_CS3);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS3].SMC_CYCLE.w = (uint32_t)config;

    /* Setup SMC_PULSE register */
    config = 0x01010101; /* Pulse register reset value */
    config = SMC_PULSE_NWE_PULSE(SMC_NWE_PULSE_CS3) | SMC_PULSE_NCS_WR_PULSE(SMC_NCS_WR_PULSE_CS3) | SMC_PULSE_NRD_PULSE(SMC_NRD_PULSE_CS3) | SMC_PULSE_NCS_RD_PULSE(SMC_NCS_RD_PULSE_CS3);
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS3].SMC_PULSE.w = (uint32_t)config;

    /* Setup SMC MODE register */
    config = 0x10001003; /* Pulse register reset value */
    config = (( SMC_MODE_READ_MODE_Msk & ((SMC_READ_MODE_CS3) <<  SMC_MODE_READ_MODE_Pos))|(SMC_MODE_WRITE_MODE_Msk & ((SMC_WRITE_MODE_CS3) <<  SMC_MODE_WRITE_MODE_Pos))| SMC_NWAIT_MODE_CS3 | (SMC_MODE_TDF_MODE_Msk & ((SMC_TDF_MODE_CS3) << SMC_MODE_TDF_MODE_Pos)) | SMC_MODE_TDF_CYCLES(SMC_TDF_CYCLES_CS3));

    /* Byte Access Type  setup , used only in 16-bit data bus */
    if (SMC_DATA_BUS_CS3 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS3== 0x1)
    {
        config |=SMC_DATA_BUS_CS3 |SMC_MODE_BAT_BYTE_WRITE;
    }
    if (SMC_DATA_BUS_CS3 == SMC_DATA_BUS_WIDTH_16BIT && SMC_BAT_CS3== 0x0)
    {
        config |=SMC_DATA_BUS_CS3 |SMC_MODE_BAT_BYTE_SELECT;
    }
    if (SMC_DATA_BUS_CS3 == SMC_DATA_BUS_WIDTH_8BIT)   /* SMC_DATA_BUS_CS3 == SMC_DATA_BUS_WIDTH_8BIT */
    {
        config |=SMC_DATA_BUS_CS3;
    }

    if (SMC_PMEN_CS3 == ASYNC_READ_PAGE_MODE)
    {
        config |= ASYNC_READ_PAGE_MODE | SMC_PS_CS3;
    }
    else
    {
        config |= STD_READ_PAGE_MODE;
    }
    _SMC_REGS->SMC_CS_NUMBER[SYS_MEMORY_SMC_CS3].SMC_MODE.w = (uint32_t)config;
    config = 0x0;
    if (SMC_MEM_SCRAMBLING_CS3 == 0x1)
    {
        _SMC_REGS->SMC_OCMS.w |=SMC_OCMS_CS3SE_Msk;
    }
    else // SMC_MEM_SCRAMBLING_CS3 == 0x0
    {
        _SMC_REGS->SMC_OCMS.w &=~(SMC_OCMS_CS3SE_Msk);
    }
</#if>

    /* Enable Write Protection */
    if (SMC_WRITE_PROTECTION == 0x1)
    {
        _SMC_REGS->SMC_WPMR.w = (SMC_WPMR_WPKEY_PASSWD | SMC_WPMR_WPEN_Msk);
    }
    else //Disable Write Protection SMC_WRITE_PROTECTION == 0x0
    {
        _SMC_REGS->SMC_WPMR.w = (SMC_WPMR_WPKEY_PASSWD & ~(SMC_WPMR_WPEN_Msk));
    }

} /* SYS_MEMORY_SMC_Initialize */

</#if>

/*******************************************************************************
 End of File
*/
