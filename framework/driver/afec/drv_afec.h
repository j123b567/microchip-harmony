/*******************************************************************************
  AFEC Driver Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_afec.h

  Summary:
    AFEC Driver Definitions Header File

  Description:
    This file contains the interface definitions (Data Types and Functions
    prototype declarations) for the PIC32C AFEC Driver. It must be included in
    any source code file that needs to access to the AFEC Driver functions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_AFEC_H
#define _DRV_AFEC_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {
#endif
 
#include "system_config.h"
#include "system/system.h"
#include "driver/driver_common.h"

// ****************************************************************************
// ****************************************************************************
// Section: Driver Type Defintions 
// ****************************************************************************
// ****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AFEC Driver Index

  Summary:
    AFEC driver index definitions

  Description:
    These constants provide AFEC driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_AFEC_Initialize and DRV_AFEC_Open
    routines to identify the driver instance in use.
*/

#define DRV_AFEC_INDEX_0  0
#define DRV_AFEC_INDEX_1  1
    
// *****************************************************************************
/* Sign mode options

  Summary:
    Defines the sign mode for AFEC.

  Description:
    This enumeration defines different options for the sign of AFEC conversion
    results.

  Remarks:
    Used while initializing the AFEC.
*/

typedef enum
{
    /* Conversion results for Single Ended channel are unsigned and
       differenitial channel are signed. */
    DRV_AFEC_SIGN_MODE_SE_UNSIGN_DF_SIGN,
    
    /* Conversion results for Single Ended channel are signed and
       differenitial channel are unsigned. */
    DRV_AFEC_SIGN_MODE_SE_SIGN_DF_UNSIGN,

    /* Conversion results of all channels are unsigned */
    DRV_AFEC_SIGN_MODE_ALL_UNSIGN,

    /* Conversion result of all channels are signed. */
    DRV_AFEC_SIGN_MODE_ALL_SIGN
    
} DRV_AFEC_SIGN_MODE;

// *****************************************************************************
/* Resolution options

  Summary:
    Defines the resolution of AFEC.

  Description:
    This enumeration defines different options for selecting AFEC conversion
    resolution.

  Remarks:
    Used while initializing the AFEC.
*/

typedef enum
{
    /* This options sets up the AFEC for 12 bit operation */
    DRV_AFEC_RESOLUTION_12_BIT = 0,

    /* This option sets up the AFEC for 13 bit operation */
    DRV_AFEC_RESOLUTION_13_BIT = 2,

    /* This option sets up the AFEC for 14 bit operation */
    DRV_AFEC_RESOLUTION_14_BIT = 3,

    /* This option sets up the AFEC for 15 bit operation */
    DRV_AFEC_RESOLUTION_15_BIT = 4,

    /* This option sets up the AFEC for 16 bit operation */
    DRV_AFEC_RESOLUTION_16_BIT = 5
    
} DRV_AFEC_RESOLUTION;

// *****************************************************************************
/* Hardware trigger sources

  Summary:
    Defines hardware trigger source selection for AFEC

  Description:
    This enumeration defines the different hardware trigger source selections
    for AFEC.

  Remarks:
    Used while initializing the AFEC.
*/

typedef enum
{
    /* This configures the external AFEx_ADTRG pins as the hardware trigger
       sources. */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_AFEC_ADTRG,
    
    /* This configures Timer Counter Channel 0 as the hardware trigger for AFEC0
       and Timer Counter Channel 3 as the hardware trigger for AFEC1 */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_TCC_0_3,
    
    /* This configures Timer Counter Channel 1 as the hardware trigger for AFEC0
       and Timer Counter Channel 4 as the hardware trigger for AFEC1 */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_TCC_1_4,

    /* This configures Timer Counter Channel 2 as the hardware trigger for AFEC0
       and Timer Counter Channel 5 as the hardware trigger for AFEC1 */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_TCC_2_5,

    /* This selects PWM0 Event Line 0 as the hardware trigger for AFEC0 and PWM1
       Event Line 0 as the hardware trigger for AFEC1. */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_PWM_EVENT_LINE_0,

    /* This selects PWM0 Event Line 1 as the hardware trigger for AFEC0 and PWM1
       Event Line 1 as the hardware trigger for AFEC1 */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_PWM_EVENT_LINE_1,

    /* This selects the Analog comparator as the hardware trigger */
    DRV_AFEC_HARDWARE_TRIG_SOURCE_ANALOG_COMPARATOR
    
} DRV_AFEC_HARDWARE_TRIG_SOURCE;

// *****************************************************************************
/* Compare window modes

  Summary:
    Defines different comparison modes available in AFEC.

  Description:
    The AFEC can perform comparison of converted values to predefined thresholds
    and generate events based the thresholds comparison criteria.

  Remarks:
    Used while initializing the AFEC.
*/

typedef enum
{
    /* Event is generated when converted data is lower than window low threshold */
    DRV_AFEC_COMPARE_MODE_LOW,
    
    /* Event is generated when converted data is higher than window high threshold */
    DRV_AFEC_COMPARE_MODE_HIGH,

    /* Event is generated when converted data is in the comparison window */
    DRV_AFEC_COMPARE_MODE_IN,
    
    /* Event is generated when converted data is out the comparison window */
    DRV_AFEC_COMPARE_MODE_OUT
    
} DRV_AFEC_COMPARE_MODE;

// *****************************************************************************
/* AFEC Error types

  Summary:
    Defines different type of AFEC errors.

  Description:
    This enumeration defines different types of AFEC errors. An error status of
    this type is returned in the DRV_AFEC_CONVERSION_COMPLETE_CALLBACK type
    callback function.

  Remarks:
    None.
*/

typedef enum
{
    /* There was no error */
    DRV_AFEC_ERROR_NONE = 0,

    /* An overrun error has occurred. The ADC result register was not read in
       time and the result was over-written. */
    DRV_AFEC_ERROR_OVERRUN = 1

} DRV_AFEC_ERROR;

// *****************************************************************************
/* AFEC Trigger Modes

  Summary:
    Defines different types of AFEC Trigger Modes.

  Description:
    This enumeration defines different types of AFEC Trigger modes. Trigger
    modes are specified during driver intialization in the triggerMode member of
    the DRV_AFEC_INIT data structure.

  Remarks:
    None.
*/

typedef enum
{
    /* The AFEC conversion is triggerred by driver software*/
    DRV_AFEC_TRIGGER_MODE_SOFTWARE = 0,

    /* The AFEC conversion is triggered by a hardware source. The hardware
       source is defined by the trigSource member of DRV_AFEC_INIT data
       structure. The triggerring is controlled by the DRV_AFEC_TriggerStart and
       DRV_AFEC_TriggerStop functions */
    DRV_AFEC_TRIGGER_MODE_HARDWARE = 1,

    /* The AFEC conversion is in free run mode and does not need a trigger. The
       conversion is controllerd by the DRV_AFEC_TriggerStart and
       DRV_AFEC_TriggerStop functions. */
    DRV_AFEC_TRIGGER_MODE_FREERUN = 2

} DRV_AFEC_TRIGGER_MODE;

// *****************************************************************************
/* AFEC result type

  Summary:
    Type for return value of AFEC Client functions

  Description:
    Some AFEC Driver functions return a result an operation result of this type.

  Remarks:
    None.
*/

typedef enum
{
    /* An unknown failure has occurred */
    DRV_AFEC_RESULT_FAILURE = -3,

    /* An input parameter is not valid */
    DRV_AFEC_RESULT_PARAMETER_INVALID = -2,

    /* The input driver handle is not valid */
    DRV_AFEC_RESULT_HANDLE_INVALID = -1,

    /* The operation was successful */
    DRV_AFEC_RESULT_SUCCESS = 0,

} DRV_AFEC_RESULT;

// *****************************************************************************
/* AFEC Conversion Sequence Type

  Summary:
    Defines the available AFEC Conversion Sequence Types.

  Description:
    This enumeration defines the available AFEC Conversion Sequence Types. The
    conversionSequenceType member of DRV_AFEC_INIT data structure should be set
    to any of these conversion sequence types.

  Remarks:
    None.
*/

typedef enum
{
    /* When selected, the AFEC converts channels as per numeric order. For
       example, if channels 0, 6, 4 and 5 are enabled, the AFEC will convert
       channel 0 followed by 4, 5 and 6. */
    DRV_AFEC_CONVERSION_SEQUENCE_TYPE_NUMERIC_ORDER = 0,

    /* When selected, the AFEC converts the channels are per user defined
       sequence. The sequence is defined by the sequence1 and sequence2 members
       of the DRV_AFEC_INIT data structure. */
    DRV_AFEC_CONVERSION_SEQUENCE_TYPE_USER_SEQUENCE = 1

} DRV_AFEC_CONVERSION_SEQUENCE_TYPE;

// *****************************************************************************
/* AFEC startup time

  Summary:
    Type for defining start-up time for AFEC

  Description:
    Configure startup time considering the AFEC clock frequency

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_STARTUP_SUT0,
    DRV_AFEC_STARTUP_SUT8,
    DRV_AFEC_STARTUP_SUT16,
    DRV_AFEC_STARTUP_SUT24,
    DRV_AFEC_STARTUP_SUT64,
    DRV_AFEC_STARTUP_SUT80,
    DRV_AFEC_STARTUP_SUT96,
    DRV_AFEC_STARTUP_SUT112,
    DRV_AFEC_STARTUP_SUT512,
    DRV_AFEC_STARTUP_SUT576,
    DRV_AFEC_STARTUP_SUT640,
    DRV_AFEC_STARTUP_SUT704,
    DRV_AFEC_STARTUP_SUT768,
    DRV_AFEC_STARTUP_SUT832,
    DRV_AFEC_STARTUP_SUT896,
    DRV_AFEC_STARTUP_SUT960

} DRV_AFEC_STARTUP;

// *****************************************************************************
/* AFEC compare type

  Summary:
    Type for comparison operation in AFEC

  Description:
    The compare window can either compare the set thresholds with
    all channels or with a specific channel denoted by  compareAfecChannel

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_COMPARE_SINGLE_CHANNNEL,
    DRV_AFEC_COMPARE_ALL_CHANNNELS,

} DRV_AFEC_COMPARE_TYPE;

// *****************************************************************************
/* AFEC channel numbers

  Summary:
    Type for defining channel number for AFEC

  Description:
    Used at different places where channel number is used.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_CHANNEL_NUMBER0,
    DRV_AFEC_CHANNEL_NUMBER1,
    DRV_AFEC_CHANNEL_NUMBER2,
    DRV_AFEC_CHANNEL_NUMBER3,
    DRV_AFEC_CHANNEL_NUMBER4,
    DRV_AFEC_CHANNEL_NUMBER5,
    DRV_AFEC_CHANNEL_NUMBER6,
    DRV_AFEC_CHANNEL_NUMBER7,
    DRV_AFEC_CHANNEL_NUMBER8,
    DRV_AFEC_CHANNEL_NUMBER9,
    DRV_AFEC_CHANNEL_NUMBER10,
    DRV_AFEC_CHANNEL_NUMBER11,
    DRV_AFEC_NUMBER_OF_CHANNELS
} DRV_AFEC_CHANNEL_NUMBER;

// *****************************************************************************
/* AFEC compare filter configuration

  Summary:
    Used for configuring the compare filer in AFEC

  Description:
    The compare filter can be used for configuring the number of consecutive
    compare events necessary to raise the flag.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_COMPARE_FILTER_VALUE0,
    DRV_AFEC_COMPARE_FILTER_VALUE1,
    DRV_AFEC_COMPARE_FILTER_VALUE2,
    DRV_AFEC_COMPARE_FILTER_VALUE3

} DRV_AFEC_COMPARE_FILTER;

// *****************************************************************************
/* AFEC channel gain configuration

  Summary:
    Used for configuring the channel gain for different channels.

  Description:
    The gain can be 1, 2, or 4. It can be individually set for channels.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_CHANNEL_GAIN1,
    DRV_AFEC_CHANNEL_GAIN2,
    DRV_AFEC_CHANNEL_GAIN4

} DRV_AFEC_CHANNEL_GAIN;

// *****************************************************************************
/* AFEC bias current configuration

  Summary:
    Used for configuring bias current for AFEC.

  Description:
    The bias current can be 1, 2, or 3.

  Remarks:
    None.
*/

typedef enum
{
    DRV_AFEC_BIAS_CURRENT_01 = 1,
    DRV_AFEC_BIAS_CURRENT_10 = 2,
    DRV_AFEC_BIAS_CURRENT_11 = 3

} DRV_AFEC_BIAS_CURRENT;

// *****************************************************************************
/* AFEC Channelset

  Summary:
    ADC Driver channelset that indicates which channels are in a channelset

  Description:
    The AFEC driver uses the concept of a Channel Set to create a group
    containing AFEC channels. A channel set must contain atleast one AFEC
    channel and can contain upto 12 AFEC channels.  There is a one to one
    mapping between a bit position in a channel set entry and the AFEC channel.
    In that, bit 0 of the channel set entry will correspond to AFEC channel CH0,
    bit 1 of the channel set entry will correspond to AFEC Channel CH1 and so
    on. Setting the bit enables the AFEC channel. The AFEC driver will perform a
    conversion on an enabled channel.

    A table (array) of channel sets can be specified for an AFEC driver
    instance. A pointer to this table is supplied to the DRV_AFEC_INIT data
    structure. The index of the channel set in the table also identifies the
    channel set. The channel set table is created automatically if the driver is
    configured through the MPLAB Harmony Configurator (MHC) tool. As an example,
    consider a channel set table that contains two channel sets.  Channel Set 0
    has the value 0x03 and channel set 1 contains the value 0xC0.  Thus, channel
    set 0 enables AFEC channels 0 and 1 and channel set 1 enables channels 6 and
    7. A channel cannot be enabled across multiple channel sets.  One AFEC
    channel can be assigned only to one channel set. The driver clients use the
    channel set index to identify the channel set whose conversion results are
    required.

    The order in which the channels are converted is affected by the choice of
    the conversion sequence (user or hardware). In such a case, it may be more
    meaningful to assign all channels in the conversion sequence to one channel
    set. 

  Remarks:
    None.
*/

typedef uint16_t DRV_AFEC_CHANNEL_SET;

// *****************************************************************************
/* AFEC Driver Sample Buffer Conversion Complete Callback Function Type.

  Summary:
    AFEC Driver Sample Buffer Conversion Complete Callback Function Type.

  Description:
    This data type defines the required function signature of the Conversion
    Complete callback function. The application must register a pointer to a
    Conversion Complete handling function whose function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive call backs from the AFEC Driver.  The driver will invoke
    this function when queued buffer is full and needs to be de-queued. The
    callback function should have be registered using the
    DRV_AFEC_ConversionCompleteCallbackSet function. The description of the
    callback function parameters is given here.

    channelSet - The index of channel set that was processed.

    buffer - A pointer to the application buffer where the results are stored

    error - This value is DRV_AFEC_ERROR_NONE if the conversion was successful
    and there is no error.  This value is DRV_AFEC_ERROR_OVERRUN if there was an
    overrun before this result was processed.

    context - The context that was provided when registering the callback
    function.
    
  Remarks:
    None.
*/

typedef void (*DRV_AFEC_CONVERSION_COMPLETE_CALLBACK)
(
    int channelSet,
    void * buffer,
    DRV_AFEC_ERROR error,
    uintptr_t context
);

// *****************************************************************************
/* AFEC Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the AFEC driver

  Description:
    This structure defines the data required to initialize or reinitialize the
    AFEC driver. A structure of this type should be defined globally and
    initialized in system_init.c The structure of this type should be passed to
    the DRV_AFEC_Initialize function.
    
  Remarks:
    None.
*/

typedef struct
{
    /* Identifies AFEC hardware module ID. */
    AFEC_MODULE_ID afecID;

    /* This specifies the conversion trigger mode. Refer to the description of
       the DRV_AFEC_TRIGGER_MODE enumeration for more details on available
       conversion trigger modes. */
    DRV_AFEC_TRIGGER_MODE triggerMode;
    
    /* This defines the conversion sequence that the AFEC must follow when
       converting channels. Refer to the DRV_AFEC_CONVERSION_SEQUENCE_TYPE
       enumeration for details on available conversion sequence types. */
    DRV_AFEC_CONVERSION_SEQUENCE_TYPE conversionSequenceType;
    
    /* Order of first 8 channels in the user sequence. This is applicable only
       if conversionSequenceType is set
       DRV_AFEC_CONVERSION_SEQUENCE_TYPE_USER_SEQUENCE. */
    uint32_t sequence1;
    
    /* Order of last 4 channels in the user sequence. This is applicable only
       if conversionSequenceType is set
       DRV_AFEC_CONVERSION_SEQUENCE_TYPE_USER_SEQUENCE. */
    uint32_t sequence2;
    
    /* Compare type indicates whether the compare mode operation is performing
       a comparison with all channels or just a single channel. */
    DRV_AFEC_COMPARE_TYPE compareType;
   
    /* Sleepmode enable/disable */
    bool sleepMode;
    
    /* Fastwake mode enable/disable */
    bool fastWake;
    
    /* TAG enable/disable. If this is set to true and driver DMA option is
       enabled, then AFEC conversion result reported by DMA will be a
       combination of the AFEC channel and converison result. */
    bool tagControl;
    
    /* Clock prescaler */
    uint8_t clockPrescaler;
    
    /* Bias current control */
    DRV_AFEC_BIAS_CURRENT biasCurrent;
    
    /* Window comparison mode */
    DRV_AFEC_COMPARE_MODE compareMode;
    
    /* Compare filter */
    DRV_AFEC_COMPARE_FILTER compareFilter;
    
    /* Channel to be compared in Window comparison*/
    uint8_t compareAfecChannel;
    
    /* Comparison Window threshold */
    uint16_t compareThresholdHigh;
    
    /* Comparison Window threshold */
    uint16_t compareThresholdLow;
    
    /* AFEC startup time */
    uint16_t startupTime;
    
    /* AFEC per channel offset */
    uint16_t * channelOffset;
    
    /* Gain selection for all channels, see CGR register */
    DRV_AFEC_CHANNEL_GAIN * channelGain;
    
    /* This should be set to the number of entries contained in the Channel Set
       Table */
    size_t channelSetTableSize;
        
    /* Dual sampling configuration */
    uint16_t dualSamplingChannels;
    
    /* Differential Mode enable */
    uint32_t differentialModeChannels;
    
    /* Hardwarer Trigger source selection if the hardwareTrigger option in this
       data structure is set to true. This is ignored if the hardwareTrigger
       otpion is set to false. */
    DRV_AFEC_HARDWARE_TRIG_SOURCE trigSource;
    
    /* Interrupt Source for the AFEC instance controlled by this driver
       instance. */
    INT_SOURCE interruptSource;
    
    /* This is a pointer to array on channel sets. The number of entries in the
       table should be specified in the channelSetTableSize option on this data
       structure. Refer to the description of the 
       DRV_AFEC_CHANNEL_SET type for details on AFEC Driver Channel Set. */
    DRV_AFEC_CHANNEL_SET * channelSetTable;
    
    /* This option configures the resolution at which the AFEC should operate.
       Refer to the description of the DRV_AFEC_RESOLUTION enumeration for
       available options. */
    DRV_AFEC_RESOLUTION resolution;
    
    /* This option configures the sign of the conversion results. Refer to the
       description of the DRV_AFEC_SIGN_MODE enumeration for available options.
       */ 
    DRV_AFEC_SIGN_MODE signMode;
    
} DRV_AFEC_INIT;

// ****************************************************************************
// ****************************************************************************
// Section: System Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_AFEC_Initialize
    (
        const SYS_MODULE_INDEX index, 
        const SYS_MODULE_INIT * const init
    );

  Summary:
    This function intializes the specified AFEC Driver instance.

  Description:
    This function intializes the specified AFEC Driver instance. The driver
    instance to be intialized is determined by index parameter. Each instance of
    the driver controls and manages one AFEC hardware peripheral. The total
    number of driver instances in the configuration is determined by the
    DRV_AFEC_INSTANCES_NUMBER configuration constant (defined in the
    system_config.h) for this configuration. The intialization function must be
    called once before calling the other driver functions. The init parameter
    should be a pointer to DRV_AFEC_INIT data structure. This structure defines
    the AFEC driver and AFEC peripheral configuration. The intialization
    function will return a valid SYS_MODULE_OBJ object that the system will then
    use to access the driver instance. 

    This function is intended to be called by the Harmomy System Module. It is
    not intended to be called directly by the application. While adding AFEC
    driver support to the application project with MHC, a call to this function
    will be automatically added to SYS_Initialize function. The intialization
    data structure will be created based on MHC and AFEC GUI configuration tool
    selections.

  Precondition:
    The init parameter is setup for meaningful driver operation. The
    DRV_AFEC_INSTANCES_NUMBER configuration constant is configured correctly for
    the required number of driver instances. This is usually less than or equal
    to the number of AFEC instances in the PIC32C device.

  Parameters:
    index  - Identifier for the instance to be initialized

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called before any other AFEC driver routine is called.
    This routine should only be called once during system initialization unless
    DRV_USART_Deinitialize is called to deinitialize the driver instance. This
    routine will NEVER block for hardware access. This function is not thread
    safe. The initialization function is called before the RTOS scheduler has
    started.
*/

SYS_MODULE_OBJ DRV_AFEC_Initialize
(
    const SYS_MODULE_INDEX index, 
    const SYS_MODULE_INIT * const init
);

// *****************************************************************************
/* Function:
    void DRV_AFEC_Deinitialize
    (
        SYS_MODULE_OBJ object
    );

  Summary:
    This function deinitializes the driver.

  Description:
    This function deinitializes the driver. Deinitializing the driver causes the
    driver operation and peripheral operation to stop. The driver state machine
    will stop updating and any on going operations will be halted. Any client
    that have opened the driver will not be able to access the driver and the
    client driver handles will be invalidated. The object parameter is the
    module object of the driver instance to be deinitialized. A deinitialized
    driver must be initialized again (by DRV_AFEC_Initialize function) in order
    for the driver functionality to accessed again. 

    This function is intended to be called by the Harmomy System Module. It is
    not intended to be called directly by the application.
  
  Precondition:
    The driver should have been initialized and object should be a valid driver
    module object. 

  Parameters:
    object - Driver module object returned from the DRV_AFEC_Initialize function
    call.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware. 
*/

void DRV_AFEC_Deinitialize( SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_AFEC_Status
    (
        SYS_MODULE_OBJ object
    );

  Summary:
    This function returns the Harmony module system status.

  Description:
    This function returns the Harmony module system status. The Harmony System
    uses the function return value to understand the current status of the
    driver state machine.

    This function is intended to be called by the Harmony System. It is not
    intended to be called by the application client.
  
  Precondition:
    The driver should have been initialized and object should be a valid driver
    module object. 

  Parameters:
    object - Driver module object returned from the DRV_AFEC_Initialize function
    call.

  Returns:
    SYS_STATUS_DEINITIALIZED - The driver has been deinitialized or is not yet
    initialized.

    SYS_STATUS_BUSY - The driver task state machine is busy with an on-going
    task. An operation request may be declined or queued.

    SYS_STATUS_READY - The driver task state machine is idle. It is ready to
    accept operations.

    SYS_STATUS_ERROR - There has been a un-recoverable error in the driver. The
    driver should be de-initialized and intialized again.

  Example:
    <code>
    </code>

  Remarks:
    The status of the driver state machine although visible only to the Harmony
    system also affects its client API. The application can use the return
    values of the client API to indirectly probe the status of the driver state
    machine.
*/

SYS_STATUS DRV_AFEC_Status( SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    void DRV_AFEC_Tasks
    (
        SYS_MODULE_OBJ object
    );

  Summary:
    This function maintains the state machine of the driver.

  Description:
    This function maintains the state machine of the driver. If the driver is
    configured for interrupt mode operation (DRV_AFEC_INTERRUPT_MODE
    configuration constant in system_config.h is defined as true), then this
    function must be called from the AFEC Interrupt Service Routine. If the
    driver is configured for polled mode operation (DRV_AFEC_INTERRUPT_MODE
    configuration constant in system_config.h is defined as false), then this
    function must be called in the Harmony System in SYS_Tasks function.

    This function is intended to be called by the Harmony System. It is not
    intended to be called by the application client.
  
  Precondition:
    The driver should have been initialized and object should be a valid driver
    module object. 

  Parameters:
    object - Driver module object returned from the DRV_AFEC_Initialize function
    call.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    The driver Tasks routine is designed to be called from one execution thread
    only. It should be not be called different threads in an RTOS application.
*/

void DRV_AFEC_Tasks( SYS_MODULE_OBJ object);

// ****************************************************************************
// ****************************************************************************
// Section: Client Access Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_AFEC_Open
    ( 
        const SYS_MODULE_INDEX index, 
        DRV_IO_INTENT ioIntent
    );

  Summary:
    This function opens the driver and returns a driver handle. 

  Description:
    This function opens the driver and returns a driver handle. The application
    must call this function to obtain the driver handle. This application can
    then use the handle to access driver client functionality. The driver
    instance to be opened is specified by the index parameter. Opening the
    driver makes the calling application the driver's client. The driver can
    have multiple clients i.e multiple applications can call the Open function
    with the same driver module index. In such a case, these multiple clients
    can submit driver requests and the driver processes these requests
    independently.

    Calling this function when the driver is not ready to be opened will cause
    the function to return an invalid driver handle (DRV_HANDLE_INVALID). The
    driver will eventually return a valid handle (a value other than
    DRV_HANDLE_INVALID) when the driver state machine and hence the driver is
    ready to be opened. The application may need to call the Open function more
    than once or repeatedly till the function returns a valid driver handle. The
    application must relinquish control to the Harmony System periodically so
    that the driver state machine is updated.

    The Open function may return an invalid handle if the driver configuration
    is not correct. This can happen if the DRV_AFEC_CLIENTS_NUMBER is not
    configured correctly. The value of this configuration constant should
    accommodate all the possible clients across all the driver instances in the
    project. 

    In an RTOS application, different application tasks can open the driver. The
    AFEC driver is designed to handle one task per handle. A handle obtained in
    one thread should not be used in another thread.
  
  Precondition:
    The driver should have been initialized. 

  Parameters:
    index   - Index of the AFEC driver instance to be opened.

    intent  - In this implementation of the driver, this paramter is ignored. It
    can be set to 0.

  Returns:
    The function returns DRV_HANDLE_INVALID if the driver is not ready to be
    opened. It returns a valid driver handle otherwise.

  Example:
    <code>
    </code>

  Remarks:
    This function is thread safe and can be called in multiple threads in an
    RTOS application. It should not be called from an ISR or an event handler.
*/

DRV_HANDLE DRV_AFEC_Open
( 
    const SYS_MODULE_INDEX index, 
    DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_AFEC_Close( DRV_HANDLE handle );

  Summary:
    Closes an opened-instance of the AFEC driver.

  Description:
    This routine closes an opened-instance of the AFEC driver, invalidating the
    handle. Any operation that was submitted by this client will be terminated.
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_AFEC_Open before the caller may use the driver again.

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    Usually there is no need for the client to verify that the Close operation
    has completed.    
*/

void DRV_AFEC_Close( const DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    DRV_AFEC_RESULT DRV_AFEC_SamplesGet
    (
        DRV_HANDLE handle, 
        int channelSet, 
        void * samples, 
        size_t sizeInBytes
    );

  Summary:
    Performs a Software Triggered Conversion on the specified channel set.

  Description:
    This function performs a Software Triggered Analog to Digital Conversion on
    all the channels contained in the channel set specified by the channelSet
    parameter. The results of the conversion are stored in an interleaved format
    in the buffer pointed to by the samples parameter. The size of the buffer
    should be large enough to accomodate all the results of all the channels in
    the input channel set.

    The channelSet parameter is an index into the channel set table that was
    provided at the time of driver initialization. All channels contained in the
    channel set will be converted. The conversion is performed and results are
    stored based on the AFEC Conversion Sequence configuration.  For example, if
    the AFEC module was configured for Hardware Conversion Sequence and if the
    specified channel set enables AFEC channels 1, 4 and 7, then channel 1 will
    be converted first, followed by channels 4 and 7. The first result in the
    samples buffer will contain result for channel 1, followed by results for
    channels 4 and 7. If the AFEC module was configured for User defined
    Sequence, then the conversions are performed and the results stored in the
    user sequence order. All the channels in the set are converted without user
    intervention.  

    The function will block (will not return) till all the channels in the
    channel set are converted. The function will trigger the Analog to Digital
    conversion using the software trigger method. When called in an RTOS
    application, only one thread can  execute the function. The other threads
    will block. 

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.
    The AFEC module should have been configured for Software Trigger mode i.e
    the triggerMode in DRV_AFEC_INIT should have been set to
    DRV_AFEC_TRIGGER_MODE_SOFTWARE.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

    channelSet - Index of the channel set to be converted. This is the index of
    channel set table provided at the time of driver intialization. 

    samples - pointer to a buffer where the conversion result will be stored.
    The size of this buffer is specified by sizeInBytes.

    sizeInBytes - The size of the buffer. The size of buffer should be large
    enough to accomodate the results of the channels in the channel set and the
    configured conversion resolution (rounded up to the nearest byte).

  Returns:
    DRV_AFEC_RESULT_SUCCESS - The conversion was performed successfully.

    DRV_AFEC_HANDLE_INVALID - The driver handle is not valid.

    DRV_AFEC_RESULT_PARAMETER_INVALID - The channel set index is not valid or
    the sample pointer is null or sizeInBytes is 0.

    DRV_AFEC_RESULT_FAILURE - An unknown failure occurred.

  Example:
    <code>
    </code>

  Remarks:
    This function is thread safe. While this function is executing in a thread,
    other threads calling this function will block. This function should not be
    called in an Interrupt Service Routine or an event handler. 
*/

DRV_AFEC_RESULT DRV_AFEC_SamplesGet
(
    DRV_HANDLE handle, 
    int channelSet, 
    void * samples, 
    size_t sizeInBytes
);

// *****************************************************************************
/* Function:
    DRV_AFEC_RESULT DRV_AFEC_SampleBufferAdd
    (
        DRV_HANDLE handle,
        int channelSet,
        void * buffer,
        size_t sizeInBytes
    };

  Summary:
    Queues a buffer to store conversion results when the AFEC is configured for
    Hardware Triggering mode.

  Description:
    This function queues a buffer to store conversion results when the AFEC is
    configured for Hardware Triggering mode. This function is non blocking. It
    will return after adding the buffer queue. The AFEC driver will store the
    conversion results in the buffer. When sizeInBytes number of bytes have been
    written to the buffer, the driver will call a callback function that was
    registered using the DRV_AFEC_ConversionCompleteCallbackSet function. A
    pointer to the buffer, the channel set that was processed, a status and the
    application defined context is returned in the call back function. The
    buffer is de-queued when the callback function exits.

    A buffer is queued against a channel set. Calling this function with a
    channel set when the driver is already processing a buffer with the same
    channel set will cause the buffer to be queued. The queued buffer is
    processed when the driver has processed all the buffers queued ahead of it.
    The application can queue up multiple buffers, each for the same or
    different channel sets. In a case where a conversion is compelte, but there
    is no buffer available to store the result, the driver will discard the
    result and will return an overflow status the next time a buffer callback
    function is called for the channel set. 

    The Conversion sequence and the order in which results are stored in a
    buffer are controlled by the AFEC driver initialization. In a case where a
    channel set contains more than one channel, results will be stored in an
    interleaved order. The conversion is triggered by a Hardware Trigger which
    is configured at the time of driver intialization. 

    The maximum number of buffers that can be queued across all instances of the
    AFEC driver is defined by the DRV_AFEC_QUEUE_DEPTH_COMBINED configuration
    constant in the system_config.h file for this configuration. Increasing this
    constant increase the number of buffers that can be queued while increasing
    the driver static memory requirement.

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.
    The AFEC module should have been configured for Hardware Trigger mode.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

    channelSet - Index of the channel set to be converted. This is the index of
    channel set table provided at the time of driver intialization. 

    buffer - pointer to a buffer where the conversion result will be stored.
    The size of this buffer is specified by sizeInBytes.

    sizeInBytes - The size of the buffer in bytes. The driver will call the
    callback function when these many bytes have been written to the buffer.

  Returns:
    DRV_AFEC_RESULT_SUCCESS - The conversion was performed successfully.

    DRV_AFEC_RESULT_HANDLE_INVALID - The driver handle is not valid.

    DRV_AFEC_RESULT_QUEUE_FULL - The queue is full. The application should try
    calling again after some time.

    DRV_AFEC_RESULT_PARAMETER_INVALID - The channel set index is not valid or
    the sample pointer is null or sizeInBytes is 0.

    DRV_AFEC_RESULT_FAILURE - An unknown failure occurred.

  Example:
    <code>
    </code>

  Remarks:
    This function is non blocking and is thread safe. It can be called from
    different threads. It should not be called from an Interrupt Service Routine
    or from an event handler.
*/

DRV_AFEC_RESULT DRV_AFEC_SampleBufferAdd
(
    DRV_HANDLE handle,
    int channelSet,
    void * buffer,
    size_t sizeInBytes
);

// *****************************************************************************
/* Function:
    DRV_AFEC_RESULT DRV_AFEC_TriggerStart(DRV_HANDLE handle, int channelSet);

  Summary:
    Starts the AFEC Conversion Hardware Trigger Source.

  Description:
    This function starts the AFEC Conversion Hardware Trigger Source. When
    started, the AFEC module will perform conversions and will store results in
    the queued buffers. The application can choose to queue up buffers using the
    DRV_AFEC_SampleBufferAdd function and call this function to start the
    conversion. In this implementation of the driver, calling this function will
    affect all channel set and all driver clients. That is, the channel set
    parameter is ignored.  

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.
    The AFEC peripheral should have been configured for Hardware triggering.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

    channelSet - In this implementation of this driver, this paramter is
    ignored.

  Returns:
    DRV_AFEC_RESULT_SUCCESS - The operation was completed successfully.

    DRV_AFEC_RESULT_HANDLE_INVALID - The input driver handle is invalid.

    DRV_AFEC_RESULT_FAILURE - An unknown faiure has occurred.

  Example:
    <code>
    </code>

  Remarks:
    In an multi-threaded RTOS application, it is recommended that the
    DRV_AFEC_TriggerStart and DRV_AFEC_TriggerStop functions should be called
    from one control application thread.`
*/

DRV_AFEC_RESULT DRV_AFEC_TriggerStart(DRV_HANDLE handle, int channelSet);

// *****************************************************************************
/* Function:
    DRV_AFEC_RESULT DRV_AFEC_TriggerStop(DRV_HANDLE handle, int channelSet);

  Summary:
    Stops the AFEC Conversion Hardware Trigger Source.

  Description:
    This function stops the AFEC Conversion Hardware Trigger Source. When stops,
    the AFEC module will stop conversions. Buffers that were queued by the
    application will continue to stay in the queue.  In this implementation of
    the driver, calling this function will affect all channel set and all driver
    clients. That is, the channel set parameter is ignored.  

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.
    The AFEC peripheral should have been configured for Hardware triggering.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

    channelSet - In this implementation of this driver, this parameter is
    ignored.

  Returns:
    DRV_AFEC_RESULT_SUCCESS - The operation was completed successfully.

    DRV_AFEC_RESULT_HANDLE_INVALID - The input driver handle is invalid.

    DRV_AFEC_RESULT_FAILURE - An unknown faiure has occurred.

  Example:
    <code>
    </code>

  Remarks:
    In an multi-threaded RTOS application, it is recommended that the
    DRV_AFEC_TriggerStart and DRV_AFEC_TriggerStop functions should be called
    from one control application thread.`
*/

DRV_AFEC_RESULT DRV_AFEC_TriggerStop(DRV_HANDLE handle, int channelSet);

// *****************************************************************************
/* Function:
    void DRV_AFEC_ConversionCompleteCallbackSet
    (
        DRV_HANDLE handle, 
        DRV_AFEC_CONVERSION_COMPLETE_CALLBACK callback,
        uintptr_t context
    );

  Summary:
    This function registers a conversion complete callback function.

  Description:
    This function registers a conversion complete callback function. This
    callback function is called when driver has compeletely processed a buffer
    that was queued using the DRV_AFEC_SampleBufferAdd function. The callback
    function is client specific. The callback function is type should match the
    DRV_AFEC_CONVERSION_COMPLETE_CALLBACK type. The driver will dequeue the
    buffer before calling the callback function. The processing of the next
    queued buffer will start when the callback function returns.

    If the driver is configured for Interrupt Mode operation, this callback
    function is called in an Interrupt context. If the driver is configured for
    Polled mode operatioin, the callback function is called from the SYS_Tasks
    function context.

  Precondition:
    DRV_AFEC_Open must have been called to obtain a valid opened device handle.
    The AFEC peripheral should have been configured for Hardware triggering.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
    routine.

    callback - The application implementation callback function which the driver
    calls when a buffer has been processed.

    context - An application defined context which is returned in the callback
    function.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    Calling this function with an invalid driver handle will result is function
    no operation.
*/

void DRV_AFEC_ConversionCompleteCallbackSet
(
    DRV_HANDLE handle, 
    DRV_AFEC_CONVERSION_COMPLETE_CALLBACK callback,
    uintptr_t context
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_AFEC_H
/*******************************************************************************
 End of File
*/

