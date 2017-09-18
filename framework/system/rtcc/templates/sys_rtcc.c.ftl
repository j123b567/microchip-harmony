<#--
/*******************************************************************************
Copyright (c) 2013-2017 released Microchip Technology Inc.  All rights reserved.

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
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
#include "arch/arm/devices_pic32c.h"
#include "system/rtcc/sys_rtcc.h"
#include "system/clk/sys_clk.h"
<#else>
<#--  PIC32M specific code -->
#include "system_config.h"
#include "system/rtcc/sys_rtcc.h"
#include "peripheral/devcon/plib_devcon.h"
</#if>

// *****************************************************************************
/* helper union for converting to and from BCD Date
  Each nibble:
    [year10][year1][mnth10][mnth1][day10][day1][DOTW] (Day Of The Week).

  Remarks:
    Fill this in manually: date.year10 = 1; or use helper function.
*/

union BCDDateConversion
{
    SYS_RTCC_BCD_DATE bcd;
    struct
    {
        uint32_t DOTW:8;
        uint32_t day1:4;
        uint32_t day10:4;
        uint32_t mnth1:4;
        uint32_t mnth10 :4;
        uint32_t year1:4;
        uint32_t year10:4;
    } parts;
};

// *****************************************************************************
/* helper union for converting to and from BCD time */
/* 4bits each: [hour10][hour1][mins10][mins1][secs10][secs1] */

union BCDConversion
{
    SYS_RTCC_BCD_TIME bcd;
    struct
    {
		uint32_t padding:8;
        uint32_t secs1 :4;
        uint32_t secs10:4;
        uint32_t mins1 :4;
        uint32_t mins10:4;
        uint32_t hour1 :4;
        uint32_t hour10:4;
    } parts;
};

// *****************************************************************************
/* Real Time Clock System Service Object
*/
static SYS_RTCC_OBJECT SysRtccObject;

// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_Start( void )

  Summary:
    Starts the Real Time Clock Calendar.

  Description:
    The function starts the RTCC.

  Precondition:
    None

  Parameters:
    None.

  Returns:
    SYS_RTCC_STATUS type (see above).

  Remarks:
     If the RTCC was running it continues.
*/

SYS_RTCC_STATUS SYS_RTCC_Start( void )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    /* Not implemented on PIC32C */
    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_Enable(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}

// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_Stop( void )

  Summary:
    Stops the Real Time Clock Calendar.

  Description:
    The function stops the RTCC.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_Stop ( void )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    /* Not implemented on PIC32C */
    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_Disable(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}

<#if CONFIG_ARCH_ARM == false>
<#--  PIC32M specific code -->
/*
void DRV_RTCC_ClockOutput ( void )
{
    PLIB_RTCC_ClockOutputEnable(RTCC_PLIB_ID);
<#if CONFIG_DRV_RTCC_OUTPUT_ENABLE == true>
    PLIB_RTCC_OutputSelect(RTCC_PLIB_ID, ${CONFIG_DRV_RTCC_OUTPUT_SELECT});
</#if>
}
*/

static __inline__ bool __attribute__((always_inline)) _SYS_RTCC_ObjectCheck ( SYS_MODULE_OBJ object )
{
    // basic sanity check we're the right object
    return (SYS_RTCC_OBJECT*)object == &SysRtccObject;
}
</#if>


// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_TimeSet(SYS_RTCC_BCD_TIME  time, bool start)

  Summary:
    Sets the Real Time Clock Calendar time.

  Description:
    The function sets the time for the RTCC.

  Precondition:
    None.

  Parameters:
    time    time is in BCD format - see description of SYS_RTCC_BCD_TIME

    start   if true, the RTCC is also started

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_TimeSet ( SYS_RTCC_BCD_TIME time, bool start )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    union BCDConversion bcdTime ;
    bcdTime.bcd = time ;

    /* - Check and stop possible ongoing RTC configuration request */
    if(_RTC_REGS->RTC_CR.w & (RTC_CR_UPDTIM_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDTIM_Msk);
    }

    if(_RTC_REGS->RTC_CR.w & (RTC_CR_UPDCAL_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDCAL_Msk);
    }

    /* - SEC synchronize */
    _RTC_REGS->RTC_SCCR.SECCLR = true ;
    while (_RTC_REGS->RTC_SR.SEC != true );
    /* - request RTC Configuration */
    _RTC_REGS->RTC_CR.w |= (RTC_CR_UPDTIM_Msk);
    /* - Wait for ack */
    while (!(_RTC_REGS->RTC_SR.w & RTC_SR_ACKUPD_Msk));
    /* - Clear ACK flag */
    _RTC_REGS->RTC_SCCR.w |= RTC_SCCR_ACKCLR_Msk;
    /* - Set New Time value */
    _RTC_REGS->RTC_TIMR.w = \
           (RTC_TIMR_HOUR(bcdTime.parts.hour1 | (bcdTime.parts.hour10 << 4)) |\
            RTC_TIMR_MIN(bcdTime.parts.mins1 | (bcdTime.parts.mins10 << 4)) |\
            RTC_TIMR_SEC(bcdTime.parts.secs1 | (bcdTime.parts.secs10 << 4))) ;
    /* - Stop Configuration */
    _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDTIM_Msk);

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_RTCTimeSet(RTCC_PLIB_ID, time);

    if (start)
    {
        SYS_RTCC_Start();
    }

    return SYS_RTCC_STATUS_OK;
</#if>
}


// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_TimeGet(SYS_RTCC_BCD_TIME  *time)

  Summary:
    Gets the Real Time Clock Calendar time.

  Description:
    The function gets the time from the RTCC.

  Precondition:
    None.

  Parameters:
    *time    a pointer to a time type - see description of SYS_RTCC_BCD_TIME


  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_TimeGet ( SYS_RTCC_BCD_TIME *time )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    volatile union BCDConversion bcdTime ;

    /* - Read Date in BCDTIME format from RTC register */
    bcdTime.parts.hour1  = (_RTC_REGS->RTC_TIMR.HOUR) & 0x0F;
    bcdTime.parts.hour10 = ((_RTC_REGS->RTC_TIMR.HOUR) & 0xF0) >> 4;
    bcdTime.parts.mins1  = (_RTC_REGS->RTC_TIMR.MIN) & 0x0F;
    bcdTime.parts.mins10 = ((_RTC_REGS->RTC_TIMR.MIN) & 0xF0) >> 4;
    bcdTime.parts.secs1  = (_RTC_REGS->RTC_TIMR.SEC) & 0x0F;
    bcdTime.parts.secs10 = ((_RTC_REGS->RTC_TIMR.SEC) & 0xF0) >> 4;
    bcdTime.parts.padding = 0;
    *time = bcdTime.bcd;

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    *time = (SYS_RTCC_BCD_TIME)PLIB_RTCC_RTCTimeGet(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_DateSet(SYS_RTCC_BCD_DATE  date)

  Summary:
    Sets the Real Time Clock Calendar date.

  Description:
    The function sets the date for the RTCC in BCD format.

  Precondition:
    None.

  Parameters:
    date    date is in BCD format - see description of SYS_RTCC_BCD_DATE

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_DateSet ( SYS_RTCC_BCD_DATE date )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    union BCDDateConversion bcdDate;
    bcdDate.bcd  = date;

    /* - Check and stop possible ongoing RTC configuration request */
    if (_RTC_REGS->RTC_CR.w & (RTC_CR_UPDTIM_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDTIM_Msk);
    }
    if (_RTC_REGS->RTC_CR.w & (RTC_CR_UPDCAL_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDCAL_Msk);
    }
    /* - SEC synchronize */
    _RTC_REGS->RTC_SCCR.SECCLR = true ;
    while (_RTC_REGS->RTC_SR.SEC != true );
    /* - request RTC Configuration */
    _RTC_REGS->RTC_CR.w |= (RTC_CR_UPDCAL_Msk);
      /* - Wait for ack */
    while (!(_RTC_REGS->RTC_SR.w & RTC_SR_ACKUPD_Msk));
      /* - Clear ACK flag */
    _RTC_REGS->RTC_SCCR.w |= RTC_SCCR_ACKCLR_Msk;
      /* - Set New CALENDAR value */
    _RTC_REGS->RTC_CALR.w = (\
            RTC_CALR_YEAR(bcdDate.parts.year1 | (bcdDate.parts.year10 << 4)) |\
            RTC_CALR_CENT(0x20) |\
            RTC_CALR_MONTH( bcdDate.parts.mnth1 | (bcdDate.parts.mnth10 << 4)) |\
            RTC_CALR_DATE( bcdDate.parts.day1 | (bcdDate.parts.day10 << 4)) |\
            RTC_CALR_DAY( bcdDate.parts.DOTW));
    /* - Stop RTC Configuration */
    _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDCAL_Msk);

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_RTCDateSet(RTCC_PLIB_ID, date);

    return SYS_RTCC_STATUS_OK;
</#if>
}


// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_DateGet(SYS_RTCC_BCD_DATE *date)

  Summary:
    Gets the Real Time Clock Calendar date.

  Description:
    The function gets the date from the RTCC in BCD format.

  Precondition:
    None.

  Parameters:
    *date    a pointer to a date type - see description of SYS_RTCC_BCD_DATE

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_DateGet ( SYS_RTCC_BCD_DATE *date )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    volatile union BCDDateConversion bcdDate;

    /* - Read Date in BCDTIME format from RTC register */
    bcdDate.parts.year1  = (_RTC_REGS->RTC_CALR.YEAR) & 0x0F;
    bcdDate.parts.year10 = ((_RTC_REGS->RTC_CALR.YEAR) & 0xF0) >> 4;
    bcdDate.parts.mnth1  = (_RTC_REGS->RTC_CALR.MONTH) & 0x0F;
    bcdDate.parts.mnth10 = ((_RTC_REGS->RTC_CALR.MONTH) & 0xF0) >> 4;
    bcdDate.parts.day1   = (_RTC_REGS->RTC_CALR.DATE) & 0x0F ;
    bcdDate.parts.day10  = ((_RTC_REGS->RTC_CALR.DATE) & 0xF0) >> 4 ;
    bcdDate.parts.DOTW   = _RTC_REGS->RTC_CALR.DAY;
    *date = bcdDate.bcd;

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    *date = (SYS_RTCC_BCD_DATE)PLIB_RTCC_RTCDateGet(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}


// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmEnable( void )

  Summary:
    Enables RTCC alarm.

  Description:
    The function enables the alarm in the RTCC.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmEnable ( void )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    <#if CONFIG_SYS_PIC32C_RTCC_ALARM_ON_DAY>
    /* - Set Day Alarm mask if required */
    _RTC_REGS->RTC_CALALR.DATEEN = true;
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_ALARM_ON_MONTH>
    /* - Set Month Alarm mask if required */
    _RTC_REGS->RTC_CALALR.MTHEN = true;
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_ALARM_ON_HOUR>
    /* - Set Hour Alarm mask if required */
    _RTC_REGS->RTC_TIMALR.HOUREN = true;
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_ALARM_ON_SEC>
    /* - Set Second Alarm mask if required */
    _RTC_REGS->RTC_TIMALR.SECEN = true;
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_ALARM_ON_MIN>
  /* - Set Minute Alarm mask if required */
    _RTC_REGS->RTC_TIMALR.MINEN = true;
    </#if>

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_AlarmEnable(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}


// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmDisable( void )

  Summary:
    Disables the RTCC alarm.

  Description:
    The function disables the alarm in the RTCC.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmDisable ( void )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    _RTC_REGS->RTC_CALALR.DATEEN = false;
    _RTC_REGS->RTC_CALALR.MTHEN = false;
    _RTC_REGS->RTC_TIMALR.HOUREN = false;
    _RTC_REGS->RTC_TIMALR.SECEN = false;
    _RTC_REGS->RTC_TIMALR.MINEN = false;

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_AlarmDisable(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmTimeSet(SYS_RTCC_BCD_TIME  time, bool enable)

  Summary:
    Sets the Real Time Clock Calendar alarm time.

  Description:
    The function sets the time for the RTCC alarm.

  Precondition:
    None.

  Parameters:
    time    time is in BCD format - see description of SYS_RTCC_BCD_TIME

    enable   if true, the alarm is enabled

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmTimeSet ( SYS_RTCC_BCD_TIME time, bool enable )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    volatile union BCDConversion bcdTime;

    bcdTime.bcd = time;
    /* - Disable Time Alarm */
    _RTC_REGS->RTC_TIMALR.w = (RTC_TIMALR_HOUREN_Msk | RTC_TIMALR_MINEN_Msk | RTC_TIMALR_SECEN_Msk );
    /* - Set New alarm Time */
    _RTC_REGS->RTC_TIMALR.w = (\
            RTC_TIMALR_HOUR( bcdTime.parts.hour1 | (bcdTime.parts.hour10 << 4)) |\
            RTC_TIMALR_MIN( bcdTime.parts.mins1 | (bcdTime.parts.mins10 << 4)) |\
            RTC_TIMALR_SEC( bcdTime.parts.secs1 | (bcdTime.parts.secs10 << 4)) );
    /* - Enable Time Alarm if requested */
    if (enable == true)
    {
        SYS_RTCC_AlarmEnable();
    }

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    PLIB_RTCC_AlarmTimeSet(RTCC_PLIB_ID, time);

    if (enable)
    {
        SYS_RTCC_AlarmEnable();
    }

    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmTimeGet(SYS_RTCC_BCD_TIME *time)

  Summary:
    Gets the Real Time Clock Calendar alarm time.

  Description:
    The function gets the time from the RTCC alarm.

  Precondition:
    None.

  Parameters:
    *time    a pointer to the time type - see description of SYS_RTCC_BCD_TIME

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmTimeGet(SYS_RTCC_BCD_TIME *time)
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    volatile union BCDConversion bcdTime;

    /* - Read time in BCDTIME format from RTC register */
    bcdTime.parts.hour1   = ((_RTC_REGS->RTC_TIMALR.HOUR) & 0x0F);
    bcdTime.parts.hour10  = (((_RTC_REGS->RTC_TIMALR.HOUR) & 0xF0)>>4);
    bcdTime.parts.mins1   = ((_RTC_REGS->RTC_TIMALR.MIN) & 0x0F);
    bcdTime.parts.mins10  = (((_RTC_REGS->RTC_TIMALR.MIN) & 0xF0)>>4);
    bcdTime.parts.secs1   = ((_RTC_REGS->RTC_TIMALR.SEC) & 0x0F);
    bcdTime.parts.secs10  = (((_RTC_REGS->RTC_TIMALR.SEC) & 0xF0)>>4);
    bcdTime.parts.padding = 0;
    *time = bcdTime.bcd;

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    *time = (SYS_RTCC_BCD_TIME)PLIB_RTCC_AlarmTimeGet(RTCC_PLIB_ID);

    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmDateSet(SYS_RTCC_BCD_DATE date)

  Summary:
    Sets the Real Time Clock Calendar alarm date.

  Description:
    The function sets the time for the RTCC alarm. The date for the alarm
    does not include the year. If the year is included it will be ignored.
  Precondition:
    None.

  Parameters:
    date    date is in BCD format - see description of SYS_RTCC_BCD_DATE

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmDateSet ( SYS_RTCC_BCD_DATE date )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    volatile union BCDDateConversion bcdDate;
    volatile bool dateen;
    volatile bool mthen;

    bcdDate.bcd = date;
    /* - Store current Alarm Mask configuration  */
    dateen = _RTC_REGS->RTC_CALALR.DATEEN;
    mthen = _RTC_REGS->RTC_CALALR.MTHEN;

    /* - Disable Date Alarm */
    _RTC_REGS->RTC_CALALR.w &= ~( RTC_CALALR_DATEEN_Msk | RTC_CALALR_MTHEN_Msk );

    /* - Set New Date Time */
    _RTC_REGS->RTC_CALALR.w = (\
        RTC_CALALR_DATE(bcdDate.parts.day1 | ( bcdDate.parts.day10 << 4) ) |\
        RTC_CALALR_MONTH( bcdDate.parts.mnth1 | ( bcdDate.parts.mnth10 << 4 ) ) ) ;

    /* - Restore Alarm Mask configuration  */
    if(dateen == true)
    {
        _RTC_REGS->RTC_CALALR.DATEEN = true;
    }

    if(mthen == true)
    {
        _RTC_REGS->RTC_CALALR.MTHEN = true;
    }

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
  PLIB_RTCC_AlarmDateSet(RTCC_PLIB_ID, date);
    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_STATUS SYS_RTCC_AlarmDateGet(SYS_RTCC_BCD_DATE *date)

  Summary:
    Gets the Real Time Clock Calendar alarm date.

  Description:
    The function gets the time for the RTCC alarm.

  Precondition:
    None.

  Parameters:
    *date    pointer to date type - see description of SYS_RTCC_BCD_DATE

  Returns:
    SYS_RTCC_STATUS type (see above).
*/

SYS_RTCC_STATUS SYS_RTCC_AlarmDateGet ( SYS_RTCC_BCD_DATE *date )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    union BCDDateConversion bcdDate;

    /* - Read Date in BCDTIME format from RTC register */
    bcdDate.parts.year1  = 0;
    bcdDate.parts.year10 = 0;
    bcdDate.parts.mnth1  = ((_RTC_REGS->RTC_CALALR.MONTH) & 0x0F);
    bcdDate.parts.mnth10 = (((_RTC_REGS->RTC_CALALR.MONTH) & 0xF0)>>4);
    bcdDate.parts.day1   = ((_RTC_REGS->RTC_CALALR.DATE) & 0x0F);
    bcdDate.parts.day10  = (((_RTC_REGS->RTC_CALALR.DATE) & 0xF0)>>4);
    bcdDate.parts.DOTW   = 0;
    *date = bcdDate.bcd;

    return SYS_RTCC_STATUS_OK;
<#else>
<#--  PIC32M specific code -->
    *date = (SYS_RTCC_BCD_DATE)(PLIB_RTCC_AlarmDateGet(RTCC_PLIB_ID));

    return SYS_RTCC_STATUS_OK;
</#if>
}



// *****************************************************************************
/* Function:
  SYS_RTCC_ALARM_HANDLE SYS_RTCC_AlarmRegister(SYS_RTCC_ALARM_CALLBACK *callback,
    uintptr_t context );

  Summary:
    Sets the callback function for an alarm.

  Description:
    This function sets the callback function that will be called when the RTCC
    alarm is reached.

  Precondition:
    None.

  Parameters:
    *callback   - a pointer to the function to be called when alarm is reached.
                  Use NULL to Un Register the alarm callback

    context     - a pointer to user defined data to be used when the callback
                  function is called. NULL can be passed in if no data needed.

  Returns:
    SYS_RTCC_ALARM_HANDLE type.
*/

SYS_RTCC_ALARM_HANDLE SYS_RTCC_AlarmRegister ( SYS_RTCC_ALARM_CALLBACK callback, uintptr_t context )
{
    SYS_RTCC_ALARM_HANDLE funcReturn;

    /* - Un-register callback if NULL */
    if (callback == NULL)
    {
        SysRtccObject.callback = NULL;
        SysRtccObject.context = (uintptr_t) NULL;
        SysRtccObject.status = SYS_RTCC_STATUS_OK;
        funcReturn = SYS_RTCC_ALARM_HANDLE_INVALID;
    }
    /* - Save callback and context in local memory */
    SysRtccObject.callback = callback;
    SysRtccObject.context = context;
    SysRtccObject.status = SYS_RTCC_STATUS_OK;
    SysRtccObject.handle = (SYS_RTCC_ALARM_HANDLE)&SysRtccObject;
    funcReturn =  (SYS_RTCC_ALARM_HANDLE)&SysRtccObject;

    return funcReturn;
}

// *****************************************************************************
/* Function:
   SYS_MODULE_OBJ SYS_TMR_Initialize ( void )

  Summary:
    Initializes hardware and data for the instance of the RTCC module and opens
    the specific module instance.

  Description:
    This function initializes hardware for the instance of the RTCC module,
    using the specified hardware initialization data. It also initializes any
    internal data structures.

  Parameters:

    init    - Pointer to a data structure containing any data necessary
            to initialize the driver. This pointer may be null if no
            data is required because static overrides have been
            provided.

  Returns:
    void
*/

<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
SYS_MODULE_OBJ SYS_RTCC_Initialize ( void )
{
    union BCDConversion bcdTime;
    union BCDDateConversion bcdDate;

    /* - Convert Time and Date */
    bcdTime.bcd = 0x${CONFIG_SYS_RTCC_TIME_SET}00;
    bcdDate.bcd = 0x${CONFIG_SYS_RTCC_DATE_SET}0${CONFIG_SYS_RTCC_DAY_SET};

    /* - Disable RTC write protection */
    _PMC_REGS->PMC_WPMR.w = PMC_WPMR_WPKEY_PASSWD;

    /* - Enable RTC peripheral clock */
    SYS_CLK_PeripheralEnable(ID_RTC);

    /* - Check and stop possible ongoing RTC configuration request */
    if (_RTC_REGS->RTC_CR.w & (RTC_CR_UPDTIM_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDTIM_Msk);
    }
    if (_RTC_REGS->RTC_CR.w & (RTC_CR_UPDCAL_Msk))
    {
        _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDCAL_Msk);
    }
    /* - SEC synchronize */
    _RTC_REGS->RTC_SCCR.SECCLR = true ;
    while (_RTC_REGS->RTC_SR.SEC != true );
    /* - request RTC Configuration */
    _RTC_REGS->RTC_CR.w |= (RTC_CR_UPDTIM_Msk|RTC_CR_UPDCAL_Msk);
    /* - Wait for ack */
    while (!(_RTC_REGS->RTC_SR.w & RTC_SR_ACKUPD_Msk));
    /* - Clear ACK flag */
    _RTC_REGS->RTC_SCCR.w |= RTC_SCCR_ACKCLR_Msk;
    /* - Set New Time value */
    _RTC_REGS->RTC_TIMR.w = (\
            RTC_TIMR_HOUR(bcdTime.parts.hour1 | (bcdTime.parts.hour10 << 4)) |\
            RTC_TIMR_MIN(bcdTime.parts.mins1 | (bcdTime.parts.mins10 << 4)) |\
            RTC_TIMR_SEC(bcdTime.parts.secs1 | (bcdTime.parts.secs10 << 4))) ;
    /* - Set New Date value */
    _RTC_REGS->RTC_CALR.w = (\
            RTC_CALR_YEAR(bcdDate.parts.year1 | (bcdDate.parts.year10 << 4)) |\
            RTC_CALR_CENT(0x20) |\
            RTC_CALR_MONTH( bcdDate.parts.mnth1 | (bcdDate.parts.mnth10 << 4)) |\
            RTC_CALR_DATE( bcdDate.parts.day1 | (bcdDate.parts.day10 << 4)) |\
            RTC_CALR_DAY( bcdDate.parts.DOTW));
    /* - Stop Configuration */
    _RTC_REGS->RTC_CR.w &= ~(RTC_CR_UPDTIM_Msk|RTC_CR_UPDCAL_Msk);
    /* - Set Alarm Date and Time value */
    SYS_RTCC_AlarmDateSet(0x00${CONFIG_SYS_RTCC_ALARM_DATE_SET}00);
    SYS_RTCC_AlarmTimeSet(0x${CONFIG_SYS_RTCC_ALARM_TIME_SET}00,true);
<#if CONFIG_SYS_RTCC_OUTPUT0_ENABLE == true>
    /* - Configure Waveform output 0*/
    _RTC_REGS->RTC_MR.OUT1 = RTC_MR_OUT1_${CONFIG_SYS_RTCC_OUTPUT0_SELECT}_Val;
</#if>
<#if CONFIG_SYS_RTCC_OUTPUT1_ENABLE == true>
    /* - Configure Waveform output 1*/
    _RTC_REGS->RTC_MR.OUT1 = RTC_MR_OUT1_${CONFIG_SYS_RTCC_OUTPUT1_SELECT}_Val;
</#if>
<#if CONFIG_SYS_RTCC_INTERRUPT_MODE == true>
    /* - Enable Interrupt at peripheral level */
    _RTC_REGS->RTC_IDR.w = (RTC_IDR_SECDIS_Msk|RTC_IDR_TIMDIS_Msk);
    _RTC_REGS->RTC_IER.w = (RTC_IER_ALREN_Msk);
    <#if CONFIG_SYS_PIC32C_RTCC_CAL_EVENT_USE == true>
    _RTC_REGS->RTC_IER.w = (RTC_IER_CALEN_Msk);
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_TIME_EVENT_USE == true>
    _RTC_REGS->RTC_IER.w = (RTC_IER_TIMEN_Msk);
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_SEC_EVENT_USE == true>
    _RTC_REGS->RTC_IER.w = (RTC_IER_SECEN_Msk);
    </#if>
</#if>
    /* - Enable Alarm */
    SYS_RTCC_AlarmEnable();
<#if CONFIG_SYS_PIC32C_RTCC_CAL_EVENT_USE == true>
    /* - Enable Calendar event */
    _RTC_REGS->RTC_CR.w |= RTC_CR_CALEVSEL(RTC_CR_CALEVSEL_${CONFIG_SYS_PIC32C_RTCC_CAL_EVENT}_Val);
</#if>
<#if CONFIG_SYS_PIC32C_RTCC_TIME_EVENT_USE == true>
    /* - Enable Time event */
    _RTC_REGS->RTC_CR.w |= RTC_CR_TIMEVSEL(RTC_CR_TIMEVSEL_${CONFIG_SYS_PIC32C_RTCC_TIME_EVENT}_Val);
</#if>

    return (SYS_MODULE_OBJ)&SysRtccObject;
}
<#else>

<#--  PIC32M specific code -->
SYS_MODULE_OBJ SYS_RTCC_Initialize ( void )
{
    PLIB_DEVCON_SystemUnlock(DEVCON_ID_0); /* Unlock System */

    /* Initialize RTCC */
    PLIB_RTCC_WriteEnable(RTCC_PLIB_ID); /* Enable writes to RTCC */
    PLIB_RTCC_Disable(RTCC_PLIB_ID); /* Disable clock to RTCC */

    /* wait for clock to stop. Block too long? */
    while (PLIB_RTCC_ClockRunningStatus(RTCC_PLIB_ID)); /* clock disabled? */

    /* initialize the time, date and alarm */
    PLIB_RTCC_RTCTimeSet(RTCC_PLIB_ID, 0x${CONFIG_SYS_RTCC_TIME_SET}00); /* Set RTCC time */
    PLIB_RTCC_RTCDateSet(RTCC_PLIB_ID, 0x${CONFIG_SYS_RTCC_DATE_SET}0${CONFIG_SYS_RTCC_DAY_SET}); /* Set RTCC date */

    PLIB_RTCC_AlarmDisable(RTCC_PLIB_ID); /* Disable alarm */
    while (PLIB_RTCC_AlarmSyncStatusGet(RTCC_PLIB_ID))  ; /* Wait for disable */
    PLIB_RTCC_AlarmTimeSet(RTCC_PLIB_ID, 0x${CONFIG_SYS_RTCC_ALARM_TIME_SET}00);
    PLIB_RTCC_AlarmDateSet(RTCC_PLIB_ID, 0x00${CONFIG_SYS_RTCC_ALARM_DATE_SET}0${CONFIG_SYS_RTCC_ALARM_DAY_SET});

    /* repeat forever or 0-255 times */
    <#if CONFIG_SYS_RTCC_ALARM_REPEAT_FOREVER == true>
    PLIB_RTCC_AlarmChimeEnable(RTCC_PLIB_ID);
    <#else>
    PLIB_RTCC_AlarmChimeDisable(RTCC_PLIB_ID);
    PLIB_RTCC_AlarmRepeatCountSet(RTCC_PLIB_ID, ${CONFIG_SYS_RTCC_ALARM_REPEAT_COUNT});
    </#if>

    /* enum here to select the alarm mask */
    PLIB_RTCC_AlarmMaskModeSelect(RTCC_PLIB_ID, ${CONFIG_SYS_RTCC_ALARM_MASK_CONFIGURATION});

    /* Initialize the output */
    <#if CONFIG_SYS_RTCC_OUTPUT_ENABLE == true>
    /* Select ouput to be placed on output pin */
#if defined (PLIB_RTCC_ExistsOutputSelect)
    {
        PLIB_RTCC_OutputSelect(RTCC_PLIB_ID, ${CONFIG_SYS_RTCC_OUTPUT_SELECT});
    }
#endif
    PLIB_RTCC_ClockOutputEnable(RTCC_PLIB_ID); /* Enable RTCC output */
    <#else>
    PLIB_RTCC_ClockOutputDisable(RTCC_PLIB_ID); /* Disable RTCC output */
    </#if>

    /* Set RTCC clock source (LPRC/SOSC) */
    <#if CONFIG_HAVE_RTCC_WITH_CLOCKSELECT == true>
    PLIB_RTCC_ClockSourceSelect(RTCC_PLIB_ID, ${CONFIG_SYS_RTCC_CLOCK_SOURCE});
    </#if>

    /* Setup RTCC Interrupt */
    <#if CONFIG_SYS_RTCC_INTERRUPT_MODE == true>
    PLIB_INT_SourceEnable(INT_ID_0, ${CONFIG_SYS_RTCC_INT_SOURCE});
    PLIB_INT_VectorPrioritySet(INT_ID_0, ${CONFIG_SYS_RTCC_ISR_VECTOR}, ${CONFIG_SYS_RTCC_INT_PRIORITY});
    PLIB_INT_VectorSubPrioritySet(INT_ID_0,${CONFIG_SYS_RTCC_ISR_VECTOR}, ${CONFIG_SYS_RTCC_INT_SUBPRIORITY});
    SysRtccObject.interruptSource = ${CONFIG_SYS_RTCC_INT_SOURCE};
    <#else>
    SysRtccObject.interruptSource = INT_SOURCE_RTCC;
    </#if>
    /* save for checking alarm state */

    SYS_RTCC_Start();

    return (SYS_MODULE_OBJ)&SysRtccObject;
}
</#if>


// *****************************************************************************
/* Function:
   void SYS_RTCC_Tasks ( SYS_MODULE_OBJ object )

Summary:
Maintains the system RTCC state machine and implements its ISR.

Description:
This routine is used to maintain the system RTCC internal state machine and
implement its ISR for interrupt-driven implementations.

Precondition:
The SYS_RTCC_Initialize function must have been called.

Parameters:
object          - SYS RTCC object returned from SYS_RTCC_Initialize

Returns:
None.
*/

void SYS_RTCC_Tasks ( SYS_MODULE_OBJ object )
{
<#if CONFIG_ARCH_ARM == true>
<#--  PIC32C specific code -->
    uint32_t rtc_status;
    SYS_RTCC_OBJECT *obj = (SYS_RTCC_OBJECT *)object;

    rtc_status = _RTC_REGS->RTC_SR.w;
    /* - Check if Alarm occurs */
    if (rtc_status & RTC_SR_ALARM_Msk)
    {
        /* - Clear Alarm flag */
        _RTC_REGS->RTC_SCCR.ALRCLR = true;
        /* - Execute Alarm callback if registered */
        if ((obj != NULL) && obj->callback)
        {
            /* Note : Callback must be non-blocking */
            obj->callback(obj->handle, obj->context);
        }
    }
  <#if CONFIG_SYS_PIC32C_RTCC_CAL_EVENT_USE == true>
    /* - Check if Calendar event occurs */
    if (rtc_status & RTC_SR_CALEV_Msk)
    {
        /* - Clear Alarm flag */
        _RTC_REGS->RTC_SCCR.CALCLR = true;
        /* - Execute Alarm callback if registered */
        if ((obj != NULL) && obj->callback)
        {
            /* Note : Callback must be non-blocking */
            obj->callback(obj->handle, obj->context);
        }
    }
  </#if>
  <#if CONFIG_SYS_PIC32C_RTCC_TIME_EVENT_USE == true>
    /* - Check if Time event occurs */
    if (rtc_status & RTC_SR_TIMEV_Msk)
    {
        /* - Clear Alarm flag */
        _RTC_REGS->RTC_SCCR.TIMCLR = true;
        /* - Execute Alarm callback if registered */
        if ((obj != NULL) && obj->callback)
        {
            /* Note : Callback must be non-blocking */
            obj->callback(obj->handle, obj->context);
        }
    }
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_SEC_EVENT_USE == true>
    /* - Check if Second event occurs */
    if (rtc_status & RTC_SR_SEC_Msk)
    {
        /* - Clear Alarm flag */
        _RTC_REGS->RTC_SCCR.SECCLR = true;
        /* - Execute Alarm callback if registered */
        if ((obj != NULL) && obj->callback)
        {
            /* Note : Callback must be non-blocking */
            obj->callback(obj->handle, obj->context);
        }
    }
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_TIME_EVENT_USE == true>
    _RTC_REGS->RTC_IER.w = (RTC_IER_TIMEN_Msk);
    </#if>
    <#if CONFIG_SYS_PIC32C_RTCC_SEC_EVENT_USE == true>
    _RTC_REGS->RTC_IER.w = (RTC_IER_SECEN_Msk);
    </#if>
<#else>
<#--  PIC32M specific code -->
    SYS_RTCC_OBJECT *obj = (SYS_RTCC_OBJECT *)object;

    if (PLIB_INT_SourceFlagGet(INT_ID_0, obj->interruptSource))
    {
        PLIB_INT_SourceFlagClear(INT_ID_0, obj->interruptSource);

        /* is there a callback to be made? do it */
        if ((obj != NULL) && obj->callback)
        {
            /* must be non-blocking*/
            obj->callback(obj->handle, obj->context);
        }
    }
</#if>
}


// *****************************************************************************
/* Function:
  uint32_t SYS_RTCC_TimeBCD2Seconds(SYS_RTCC_BCD_TIME time)

  Summary:
    Helper function for time.

  Description:
    This function returns the number of seconds when given a BCD encoded time
    value. (see SYS_RTCC_BCD_TIME typedef above).

  Precondition:
    None.

  Parameters:
    time    - a SYS_RTCC_BCD_TIME value.

  Returns:
    The number of seconds represented by the BCD value.
*/

uint32_t TimeBCD2Seconds ( SYS_RTCC_BCD_TIME TimeInBCD )
{
    /* use a union and make the compiler do the work */
    union BCDConversion conversion = {0};
    conversion.bcd = TimeInBCD;

    return conversion.parts.hour10 * 36000u + conversion.parts.hour1 * 3600 +
           conversion.parts.mins10 * 600 +    conversion.parts.mins1 * 60 +
           conversion.parts.secs10 * 10 +     conversion.parts.secs1;
}



// *****************************************************************************
/* Function:
  SYS_RTCC_BCD_TIME SYS_RTCC_TimeSeconds2BCD(uint32_t seconds)

  Summary:
    Helper function for time.

  Description:
    This function returns the BCD encoded time
    value for the given number of seconds.

  Precondition:
    None.

  Parameters:
    seconds    - number of seconds to convert.

  Returns:
    A SYS_RTCC_BCD_TIME type value in BCD of the number of seconds given.
*/

SYS_RTCC_BCD_TIME TimeSeconds2BCD ( uint32_t seconds )
{
    union BCDConversion conversion = {0};

    /* limit and decompose seconds to BCD */
    seconds %= 360000ul;
    conversion.parts.hour10 = seconds / 36000u;
    seconds %= 36000u;
    conversion.parts.hour1  = seconds / 3600;
    seconds %= 3600;
    conversion.parts.mins10 = seconds / 600;
    seconds %= 600;
    conversion.parts.mins1  = seconds / 60;
    seconds %= 60;
    conversion.parts.secs10 = seconds / 10;
    seconds %= 10;
    conversion.parts.secs1  = seconds;

    return conversion.bcd;
}

<#--
/*******************************************************************************
 End of File
*/
-->
