/*******************************************************************************
  DAC Driver Definitions Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dac_definitions_pic32c.h

  Summary:
    DAC Driver Definitions Header File

  Description:
    This file will provide enumerations and other dependencies needed by
    DAC driver to manage the DAC module on PIC32C microcontrollers.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _DRV_DAC_DEFINITIONS_PIC32C_H
#define _DRV_DAC_DEFINITIONS_PIC32C_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver DAC Module Index

  Summary:
    DAC driver index definitions

  Description:
    These constants provide DAC driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_DAC_Initialize routines to 
	identify the driver instance in use.
*/

#define DRV_DAC_INDEX_0             0

// *****************************************************************************
/* DAC Output Modes of Operation

  Summary:
    Identifies the output modes of the operation of the DAC module

  Description:
    This data type identifies the Output modes of operation of the DAC module.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum
{
    /* Single-ended analog output. Allows user to configure Two channels
    independently */
    DRV_DAC_OUTPUT_MODE_SINGLE_ENDED,     
            
    /* Differential analog output. All operations are driven by Channel 0 */
    DRV_DAC_OUTPUT_MODE_DIFFERENTIAL   
	      
} DRV_DAC_OUTPUT_MODE;


// *****************************************************************************
/* DAC Channel Modes of Operation

  Summary:
    Identifies the Channel modes of the operation of the DAC module

  Description:
    This data type identifies the Channel modes of operation of the DAC module.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum
{
    /* Conversion starts after Raising edge of trigger to send data to DAC */
    DRV_DAC_OPERATION_TRIGGER_MODE,  
            
    /* Conversion starts as soon as at least one channel is enabled */    
    DRV_DAC_OPERATION_FREE_RUNNING_MODE,
            
    /* Conversion rate is forced by the controller */
	DRV_DAC_OPERATION_MAX_SPEED_MODE	
	
} DRV_DAC_OPERATION_MODE;

// *****************************************************************************
/* DAC Channel Index

  Summary:
    Identifies the Channel index of DAC module.

  Description:
    This data type identifies the Channel index of DAC module.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum
{
    /* Channel 0 Index */
    DRV_DAC_CHANNEL_INDEX_0, 
            
    /* Channel 1 Index */
    DRV_DAC_CHANNEL_INDEX_1         
        
} DRV_DAC_CHANNEL_INDEX;

// *****************************************************************************
/* DAC Channel Events 

  Summary:
    Identifies the possible events that can result during Data conversion.

  Description:
    Identifies the possible events that can result during Data conversion.

  Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that the client registered with the driver by
    calling the DRV_DAC_EventHandlerSet function. */
	
typedef enum
{   
    /* DAC is ready to accept new conversion requests */
	DRV_DAC_EVENT_READY,
    
    /* Previous Data conversion is in progress */
	DRV_DAC_EVENT_PROCESSING,
    
    /* No Event */
	DRV_DAC_EVENT_NONE
        
}DRV_DAC_EVENT;

// *****************************************************************************
/* DAC Input Trigger source selection

  Summary:
    Identifies the Input Trigger when Channel mode is "Trigger Mode"

  Description:
    This data type identifies the Channel modes of operation of the DAC module.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum 
{
    /* External Input Trigger source */
    DRV_DAC_TRIGGER_SOURCE_EXTERNAL_INPUT_DATRG = 0,
    
    /* Trigger source from Timer Counter Channel0 I/O Line A */        
    DRV_DAC_TRIGGER_SOURCE_TC0_TIOA0_EVENT,
    
    /* Trigger source from Timer Counter Channel1 I/O Line A */        
    DRV_DAC_TRIGGER_SOURCE_TC0_TIOA1_EVENT,
    
    /* Trigger source from Timer Counter Channel2 I/O Line A */        
    DRV_DAC_TRIGGER_SOURCE_TC0_TIOA2_EVENT, 
    
    /* Trigger source form PWM0 Event Line 0 */        
    DRV_DAC_TRIGGER_SOURCE_PWM0_EVENT0,
    
    /* Trigger source form PWM0 Event Line 1 */        
    DRV_DAC_TRIGGER_SOURCE_PWM0_EVENT1,
    
    /* Trigger source form PWM1 Event Line 0 */        
    DRV_DAC_TRIGGER_SOURCE_PWM1_EVENT0,
    
    /* Trigger source form PWM1 Event Line 1 */        
    DRV_DAC_TRIGGER_SOURCE_PWM1_EVENT1

} DRV_DAC_TRIGGER_SOURCE;

// *****************************************************************************
/* DAC Oversampling ratio selection

  Summary:
    Identifies the Oversampling ratio when DAC Channel mode is in "Trigger Mode" 

  Description:
    This data type identifies Oversampling ratio.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum 
{
    /* No Oversampling */
    DRV_DAC_OSR_1,
    
    /* Integrated Interpolation Filter with 2x Oversampling Ratio (OSR) */        
	DRV_DAC_OSR_2,
    
    /* Integrated Interpolation Filter with 4x Oversampling Ratio (OSR) */        
	DRV_DAC_OSR_4,
    
    /* Integrated Interpolation Filter with 8x Oversampling Ratio (OSR) */        
	DRV_DAC_OSR_8,
    
    /* Integrated Interpolation Filter with 16x Oversampling Ratio (OSR) */        
	DRV_DAC_OSR_16,
    
    /* Integrated Interpolation Filter with 32x Oversampling Ratio (OSR) */        
	DRV_DAC_OSR_32
    
} DRV_DAC_OVERSAMPLE_RATIO;

// *****************************************************************************
/* DAC Channel Analog output current control selections

  Summary:
    Allows to adapt the slew rate of the analog output

  Description:
    This data type Allows to adapt the slew rate of the analog output.

  Remarks:
    Refer to the specific device data sheet to determine availability.
*/

typedef enum 
{   
    /* Configures to DAC Max speed */
    DRV_DAC_SPEED_1M=3,
    
    /* Configures to DAC to 500 KSps speed */        
	DRV_DAC_SPEED_500K=1,
    
    /* Disables the DAC output buffer and thus minimizes power consumption */
	DRV_DAC_SPEED_BYPASS=0
            
} DRV_DAC_SPEED;	

// *****************************************************************************
/* DAC Driver Channel Initialization data

  Summary:
    Defines the data used to initialize the DAC Driver

  Description:
    This structure is used to define the data used to initialize the DAC Driver.

  Remarks:
    None.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT moduleInit;
    
    /* Identifies the DAC peripheral instance */
    DACC_MODULE_ID dacID; 
    
    /* Identifies DAC channel instance */
	DRV_DAC_CHANNEL_INDEX dacIndex;
    
    /* Peripheral Id */
	uint32_t PID;
    
    /* Identifies DAC output mode */
    DRV_DAC_OUTPUT_MODE outputMode;
    
    /* Identifies DAC operation mode */
    DRV_DAC_OPERATION_MODE operationMode;
    
    /* Identifies slew rate of Analog output */
	DRV_DAC_SPEED dacSpeed;
    
    /* Identifies Trigger source of DAC */
	DRV_DAC_TRIGGER_SOURCE triggerSource;
    
    /* Identifies Oversampling ratio */
	DRV_DAC_OVERSAMPLE_RATIO overSampleRatio;
    
    /* Identifies Interrupt generation */
	bool interruptMode;
    
} DRV_DAC_INIT;

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END


#endif // #ifndef _DRV_DAC_DEFINITIONS_PIC32C_H
/*******************************************************************************
 End of File
*/

