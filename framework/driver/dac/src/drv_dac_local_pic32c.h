/*******************************************************************************
  DAC Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dac_local_pic32c.h

  Summary:
    DAC Driver Local Data Structures

  Description:
    Driver Local Data Structures
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

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

#ifndef _DRV_DAC_LOCAL_H
#define _DRV_DAC_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "driver/dac/drv_dac.h"

// *****************************************************************************
/* DAC Driver Instance Object

  Summary:
    Object used to keep any data required for an instance of the DAC driver.

  Description:
    This object is used to keep track of any data required for an instance of 
    the DAC driver.

  Remarks:
    None.
*/
typedef struct
{
    /* The module index associated with the object */
    dacc_registers_t *moduleId;
    
    /* Identifies DAC output mode */
    DRV_DAC_OUTPUT_MODE outputMode;
    
    /* Flag to indicate interrupt is enabled or not */
    bool interruptMode;
    
    /* The status of the driver */
    SYS_STATUS status;
    
    /* Flag to indicate this object is in use */
    bool inUse;
	
} DRV_DAC_OBJ;

// *****************************************************************************
/* DAC Driver Client Object

  Summary:
    Object used to keep track of a DAC client.

  Description:
    This object is used to keep track of a DAC client.

  Remarks:
    None.
*/
typedef struct
{
    /* The hardware instance object associated with the client */
    DRV_DAC_OBJ * dObj;
    
    /* This flags indicates if the object is in use or is available*/
    bool inUse;
    
    /* The IO intent with which the client is opened */
    DRV_IO_INTENT intent;
    
    /* Client status */
    DRV_CLIENT_STATUS status;
    
    /* Application Context associated with this client */
    uintptr_t context;
    
    /* Identifies DAC channel instance */
    DRV_DAC_CHANNEL_INDEX chId;
    
    /* Identifies the possible events that can result during Data conversion */
	DRV_DAC_EVENT event;
    
    /* Write Data callback function */
	DRV_DAC_EVENT_HANDLE eventHandler;
	 
} DRV_DAC_CH_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Local functions.
// *****************************************************************************
// *****************************************************************************

void _DRV_DAC_HardwareSetup(dacc_registers_t *moduleId, DRV_DAC_INIT * dacInit);

#endif // #ifndef _DRV_DAC_LOCAL_H

/*******************************************************************************
 End of File
*/

