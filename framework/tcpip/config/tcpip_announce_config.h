/*******************************************************************************
  Announce Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_announce_config.h

  Summary:
    Announce configuration file

  Description:
    This file contains the Announce module configuration options
    
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright � 2011 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END
#ifndef _TCPIP_ANNOUNCE_CONFIG_H_
#define _TCPIP_ANNOUNCE_CONFIG_H_


// Maximum size of a payload sent once
// Adjust to your needs
#define TCPIP_ANNOUNCE_MAX_PAYLOAD        (512)

// Type of the broadcast used by the Announce module
// This type enables the network directed broadcast
// If it is not defined or it's 0, the network limited broadcast is used
// The default is nework limited broadcast
#define TCPIP_ANNOUNCE_NETWORK_DIRECTED_BCAST   0

// announce task rate, millseconds
// The default value is 333 milliseconds.
// The lower the rate (higher the frequency) the higher the module priority
// and higher module performance can be obtained
// The value cannot be lower than the TCPIP_STACK_TICK_RATE.
#define TCPIP_ANNOUNCE_TASK_RATE    333


#endif  // _TCPIP_ANNOUNCE_CONFIG_H_
