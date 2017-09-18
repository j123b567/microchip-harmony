<#--
/*******************************************************************************
  ADC Driver Interrupt Handler Template File

  File Name:
    drv_adc_int.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
<#if CONFIG_DRV_ADC_INTERRUPT_MODE == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_ADC_ISR_VECTOR}, IPL${CONFIG_DRV_ADC_INT_IPL}SOFT) _IntHandlerDrvAdc(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_ADC_INT_IPL}AUTO), vector(${CONFIG_DRV_ADC_ISR_VECTOR}))) IntHandlerDrvAdc_ISR( void );
</#if>
void IntHandlerDrvAdc(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_ADC_ISR_VECTOR}, ipl${CONFIG_DRV_ADC_INT_IPL}AUTO) _IntHandlerDrvAdc(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_ADC_DRIVER_MODE == "DYNAMIC">
    DRV_ADC_Tasks(sysObj.drvAdc);
<#else>
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_ADC_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>


<#if CONFIG_DRV_ADCHS_INTERRUPT_MODE == true>

<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INTERRUPT == true>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX0 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF1_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF1">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX1 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF2_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF2">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX2 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF3_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF3">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX3 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF4_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF4">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX4 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF5_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF5">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INST_IDX5 == true>
    <#assign DRV_ADCHS_DIGITAL_FILTER_VECTOR = "_ADC_DF6_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DF6">
</#if>
</#if>

<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INTERRUPT_EN == true>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX0 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC1_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC1">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX1 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC2_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC2">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX2 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC3_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC3">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX3 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC4_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC4">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX4 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC5_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC5">
</#if>
<#if CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INST_IDX5 == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC6_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC6">
</#if>
</#if>

<#if CONFIG_USE_DRV_ADCHS_CVD == true>
    <#assign DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR = "_ADC_DC1_VECTOR">
    <#assign CONFIG_DRV_ADCHS_INTERRUPT_SOURCE = "INT_SOURCE_ADC_1_DC1">
</#if>

<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INTERRUPT == true>
void __ISR(${DRV_ADCHS_DIGITAL_FILTER_VECTOR}, IPL${CONFIG_DRV_ADCHS_INT_IPL}SOFT) _IntHandlerDrvAdc(void)
<#elseif CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INTERRUPT_EN == true>
void __ISR(${DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR}, IPL${CONFIG_DRV_ADCHS_INT_IPL}SOFT) _IntHandlerDrvAdc(void)
<#else>
void __ISR(${CONFIG_DRV_ADCHS_ISR_VECTOR}, IPL${CONFIG_DRV_ADCHS_INT_IPL}SOFT) _IntHandlerDrvAdc(void)
</#if>
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INTERRUPT == true>
void __attribute__( (interrupt(ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO), vector(${DRV_ADCHS_DIGITAL_FILTER_VECTOR}))) IntHandlerDrvAdc_ISR( void );
<#elseif CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INTERRUPT_EN == true>
void __attribute__( (interrupt(ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO), vector(${DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR}))) IntHandlerDrvAdc_ISR( void );
<#else>
void __attribute__( (interrupt(ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO), vector(${CONFIG_DRV_ADCHS_ISR_VECTOR}))) IntHandlerDrvAdc_ISR( void );
</#if>
</#if>
void IntHandlerDrvAdc(void)
</#if>
<#else>
<#if CONFIG_DRV_ADCHS_DIGITAL_FILTER_INTERRUPT == true>
void __ISR(${DRV_ADCHS_DIGITAL_FILTER_VECTOR}, ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO) _IntHandlerDrvAdc(void)
<#elseif CONFIG_DRV_ADCHS_DIGITAL_COMPARATOR_INTERRUPT_EN == true>
void __ISR(${DRV_ADCHS_DIGITAL_COMPARATOR_VECTOR}, ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO) _IntHandlerDrvAdc(void)
<#else> 
void __ISR(${CONFIG_DRV_ADCHS_ISR_VECTOR}, ipl${CONFIG_DRV_ADCHS_INT_IPL}AUTO) _IntHandlerDrvAdc(void)
</#if>
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_ADCHS_DRIVER_MODE == "DYNAMIC">
    DRV_ADC_Tasks(sysObj.drvAdc);
<#else>
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_ADCHS_INTERRUPT_SOURCE});
</#if>	
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
