/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    random_pic32c.c

  Summary:
    Crypto Framework Library source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic
	Framework Library for PIC32 families of Microchip micro-controllers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  random_pic32c.c
Copyright � 2017 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END



#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif
#include "system_config.h"

#include "crypto/src/settings.h"

#include "crypto/src/random.h"

#include "crypto/src/error-crypt.h"

#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */


#define RNG_BYTES_AT_A_TIME 4


int pic32c_InitRng(void)
{
    /* Enable Peripheral clock for TRNG in Power Management Contoller */
    uint32_t PmcBit = 1u << (ID_TRNG - 32);
    if ((_PMC_REGS->PMC_PCSR1.w & PmcBit) != PmcBit)
    {
        /* turn on */
        _PMC_REGS->PMC_PCER1.w = PmcBit;

        /* enable */
        _TRNG_REGS->TRNG_CR.w = TRNG_CR_KEY_PASSWD | TRNG_CR_ENABLE_Msk;

        /* memory barrier */
        __DMB();
    }

    return 0;
}



/* sz is number of bytes */
int pic32c_RNG_GenerateBlock(byte* output, word32 sz)
{
    while (sz)
    {
        /* get 32 bits - BLOCKING */
        while (!_TRNG_REGS->TRNG_ISR.DATRDY)
        {
            /* Wait until data ready. */
        }

        /* get 32 bits */
        uint32_t result = _TRNG_REGS->TRNG_ODATA.ODATA;

        /* stuff the data into the output buffer. Watch for buffer overrun */
        if (sz < RNG_BYTES_AT_A_TIME)
        {
            memcpy((void *)output, (void*)&result, sz);
            sz = 0;
        }
        else
        {
            memcpy((void *)output, (void*)&result, RNG_BYTES_AT_A_TIME);
            sz -= RNG_BYTES_AT_A_TIME;

            /* bump the byte buffer by 4 since we are doing 32 bit randoms */
            output += RNG_BYTES_AT_A_TIME;
        }

        /* memory barrier */
        __DMB();
    }

    return 0;
}



int pic32c_RNG_GenerateByte(byte* b)
{
    /* get 32 bits - BLOCKING */
    while (!_TRNG_REGS->TRNG_ISR.DATRDY)
    {
        /* Wait until data ready. */
    }

    /* cram 32 bits into one byte */
    *b = (byte)_TRNG_REGS->TRNG_ODATA.ODATA;

    /* memory barrier */
    __DMB();

    return 0;
}



int pic32c_FreeRng(void)
{
    /* disable */
    _TRNG_REGS->TRNG_CR.w = TRNG_CR_KEY_PASSWD;

    /* Disable Peripheral Clock to TRNG by writing 1 to bit position */
    /* TRNG >=32 use PCDR1 not PCDR0 and remove 32 bit positions */
    _PMC_REGS->PMC_PCDR1.w = 1u << (ID_TRNG - 32);

    return 0;
}



/* used by random.c */
int pic32c_GenerateSeed(byte* output, word32 sz)
{
    int status;

    /* Make sure power is on, etc */
    status = pic32c_InitRng();
    if (!status)
    {
        status = pic32c_RNG_GenerateBlock(output, sz);
    }

    return status;
}


