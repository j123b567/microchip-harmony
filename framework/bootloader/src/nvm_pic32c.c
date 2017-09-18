/*******************************************************************************
  MPLAB Harmony Bootloader Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    nvm_pic32c.c

  Summary:
    This file contains the source code for handling NVM controllers.

  Description:
    This file contains the source code for the NVM handling functions for
	PIC32C devices.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include "nvm.h"
#include "system_config.h"

typedef struct
{
    uint8_t RecDataLen;
    uint32_t Address;
    uint8_t RecType;
    uint8_t* Data;
    uint8_t CheckSum;
    uint32_t ExtSegAddress;
    uint32_t ExtLinAddress;
}T_HEX_RECORD;

typedef struct {
    uint32_t buff[IFLASH_PAGE_SIZE / 4];
    uint32_t length;
    uint32_t addr;
}T_NVM_DATA;

static T_NVM_DATA nvm_data =
{
    .addr = (uint32_t)APP_FLASH_BASE_ADDRESS
};

static uint32_t *nvm_buff_ptr = nvm_data.buff;

void APP_FlashErase( void )
{
    unsigned int flashAddr = APP_FLASH_BASE_ADDRESS;
    volatile uint16_t page_number;
    SYS_INT_PROCESSOR_STATUS processor_status;
    uint32_t status;
    uint32_t fmr;

    processor_status = SYS_INT_StatusGetAndDisable();

    while (flashAddr < APP_FLASH_END_ADDRESS)
    {
    	/*Calculate the Page number to be passed for FARG register*/
        page_number = (flashAddr - IFLASH_ADDR) / IFLASH_PAGE_SIZE;

    	/*Attach the FRDY interrupt to NVIC for interrupt mode*/
        fmr = _EFC_REGS->EEFC_FMR.w;
    	_EFC_REGS->EEFC_FMR.w = (fmr | EEFC_FMR_FRDY_Msk);

        /* Issue the Flash write operation*/
    	_EFC_REGS->EEFC_FCR.w = (EEFC_FCR_FCMD_EPA|EEFC_FCR_FARG(page_number|0x2)|EEFC_FCR_FKEY_PASSWD);

        /*Wait for the flash operation to complete*/
        do
        {
        	status = _EFC_REGS->EEFC_FSR.w;
        } while ((status & EEFC_FSR_FRDY_Msk) != EEFC_FSR_FRDY_Msk);

        flashAddr += NVM_ERASE_PAGE_SIZE;
    }
    SYS_INT_StatusRestore(processor_status);
}

void APP_NVMWrite(void* address, uint32_t* data)
{
	volatile uint16_t page_number;
	volatile uint32_t fmr;
    SYS_INT_PROCESSOR_STATUS processor_status;
	uint32_t status;

    processor_status = SYS_INT_StatusGetAndDisable();

	/*Calculate the Page number to be passed for FARG register*/
	page_number = ((uint32_t) address - IFLASH_ADDR) / IFLASH_PAGE_SIZE;

	for (uint32_t i = 0; i < IFLASH_PAGE_SIZE; i += 4)
	{
		*((uint32_t *)( IFLASH_ADDR + ( page_number * IFLASH_PAGE_SIZE ) + i )) = *data++;
	}

	/*Attach the FRDY interrupt to NVIC for interrupt mode*/
	fmr = _EFC_REGS->EEFC_FMR.w;
	_EFC_REGS->EEFC_FMR.w = (fmr | EEFC_FMR_FRDY_Msk);

	/* Issue the Flash write operation*/
	_EFC_REGS->EEFC_FCR.w = (EEFC_FCR_FCMD_WP | EEFC_FCR_FARG(page_number)| EEFC_FCR_FKEY_PASSWD);

	/*Wait for the flash operation to complete*/
	do
	{
		status = _EFC_REGS->EEFC_FSR.w;
	} while ((status & EEFC_FSR_FRDY_Msk) != EEFC_FSR_FRDY_Msk);

    SYS_INT_StatusRestore(processor_status);
}

char APP_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen)
{
    static T_HEX_RECORD HexRecordSt;
    uint8_t Checksum = 0;
    uint32_t i;
    uint32_t data_len = 0;

    void* ProgAddress;
    uint32_t nextRecStartPt = 0;

    while(totalLen>=5) // A hex record must be atleast 5 bytes. (1 Data Len byte + 1 rec type byte+ 2 address bytes + 1 crc)
    {
        HexRecord = &HexRecord[nextRecStartPt];
        HexRecordSt.RecDataLen = HexRecord[0];
        HexRecordSt.RecType = HexRecord[3];
        HexRecordSt.Data = &HexRecord[4];

        //Determine next record starting point.
        nextRecStartPt = HexRecordSt.RecDataLen + 5;

        // Decrement total hex record length by length of current record.
        totalLen = totalLen - nextRecStartPt;
 
        // Hex Record checksum check.
        Checksum = 0;
        for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
        {
            Checksum += HexRecord[i];
        }

        if(Checksum != 0)
        {
            return HEX_REC_CRC_ERROR;
        }
        else
        {
            // Hex record checksum OK.
            switch(HexRecordSt.RecType)
            {
                case DATA_RECORD:  //Record Type 00, data record.
                    HexRecordSt.Address = (HexRecord[1]<<8) + HexRecord[2];

                    // Derive the address.
                    HexRecordSt.Address = HexRecordSt.Address + HexRecordSt.ExtLinAddress + HexRecordSt.ExtSegAddress;
                    
                    while(HexRecordSt.RecDataLen) // Loop till all bytes are done.
                    {
                        ProgAddress = (void *)(HexRecordSt.Address);

                        // Make sure we are writing in the desired application memory space.
                        if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS)))
                        {
                            if (nvm_data.length == IFLASH_PAGE_SIZE)
                            {
                                nvm_data.length = 0;
                                APP_NVMWrite((void *)nvm_data.addr, nvm_data.buff);
                                nvm_data.addr += IFLASH_PAGE_SIZE;
                                memset((void *)nvm_data.buff, 0, IFLASH_PAGE_SIZE);
                                nvm_buff_ptr = nvm_data.buff;
                            }

                            data_len = HexRecordSt.RecDataLen;

                            if ((nvm_data.length + HexRecordSt.RecDataLen) > IFLASH_PAGE_SIZE)
                            {
                                data_len = IFLASH_PAGE_SIZE - nvm_data.length;
                            }

                            memcpy(nvm_buff_ptr, HexRecordSt.Data, data_len);
                            nvm_buff_ptr += data_len/4;
                            nvm_data.length += data_len;
                            HexRecordSt.Address += data_len;
                            HexRecordSt.Data += data_len;
                            HexRecordSt.RecDataLen -= data_len;

                        }
                        else    // Out of boundaries. Adjust and move on.
                        {
                            // Increment the address.
                            HexRecordSt.Address += 4;
                            // Increment the data pointer.
                            HexRecordSt.Data += 4;
                            // Decrement data len.
                            if(HexRecordSt.RecDataLen > 3)
                            {
                                HexRecordSt.RecDataLen -= 4;
                            }
                            else
                            {
                                HexRecordSt.RecDataLen = 0;
                            }
                        }
                    }
                    break;

                case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
                    HexRecordSt.ExtSegAddress = (HexRecordSt.Data[0]<<12) + (HexRecordSt.Data[1]<<4);
                    
                    // Reset linear address.
                    HexRecordSt.ExtLinAddress = 0;
                    break;

                case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
                    HexRecordSt.ExtLinAddress = (HexRecordSt.Data[0]<<24) + (HexRecordSt.Data[1]<<16);

                    // Reset segment address.
                    HexRecordSt.ExtSegAddress = 0;
                    break;

                case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
                    /* Program the remaining bytes if any */
                    if(nvm_data.length > 0)
                    {
                        memset(nvm_buff_ptr, 0xFF, (IFLASH_PAGE_SIZE - nvm_data.length));
                        APP_NVMWrite((void *)nvm_data.addr, nvm_data.buff);
                        nvm_data.length = 0;
                        nvm_data.addr += IFLASH_PAGE_SIZE;
                        memset((void *)nvm_data.buff, 0, IFLASH_PAGE_SIZE);
                        nvm_buff_ptr = nvm_data.buff;   
                    }
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;

                case START_LIN_ADRS_RECORD:
                default:
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;
            }

        }
    }

    if ( (HexRecordSt.RecType == DATA_RECORD) || (HexRecordSt.RecType == EXT_SEG_ADRS_RECORD)
            || (HexRecordSt.RecType == EXT_LIN_ADRS_RECORD) || (HexRecordSt.RecType == START_LIN_ADRS_RECORD)
            || (HexRecordSt.RecType == END_OF_FILE_RECORD))
    {
        return HEX_REC_NORMAL;
    }
    else
    {
        return HEX_REC_UNKNOW_TYPE;
    }
}
