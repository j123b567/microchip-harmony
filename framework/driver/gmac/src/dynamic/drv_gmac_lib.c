/*******************************************************************************
  Ethernet Driver Library Source Code

  Summary:
    This file contains the source code for the Ethernet Driver library.

  Description:
    This library provides a low-level abstraction of the Ethernet module
    on Microchip PIC32MX family microcontrollers with a convenient C language
    interface.  It can be used to simplify low-level access to the module
    without the necessity of interacting directly with the module's registers,
    thus hiding differences from one microcontroller variant to another.
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
File Name:      drv_gmac_lib.c
Processor:      PIC32C
Compiler:       Microchip MPLAB 

Copyright � 2008-2009 released Microchip Technology Inc.  All rights
reserved.

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


#include "driver/gmac/src/dynamic/drv_gmac_lib.h"

#if defined(PIC32C_GMAC_ISR_TX) //GMAC with TX interrupt enabled
	#define GMAC_INT_BITS  (GMAC_INT_RX_BITS | GMAC_INT_TX_BITS)
#else
	#define GMAC_INT_BITS  (GMAC_INT_RX_BITS)
#endif


static bool _MacRxPacketAck(TCPIP_MAC_PACKET* pkt,  const void* param);
extern bool SYS_INT_SourceRestore(INT_SOURCE src, int level);

/* TX descriptors for 6 Queues */
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_0[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE0];
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_1[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE1];
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_2[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE2];
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_3[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE3];
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_4[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE4];
__attribute__((__aligned__(8)))  DRV_PIC32CGMAC_HW_TXDCPT sTxDesc_queue_5[TCPIP_GMAC_TX_DESCRIPTORS_COUNT_QUE5];
/* RX descriptors for 6 Queues */
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_0[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE0];
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_1[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE1];
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_2[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE2];
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_3[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE3];
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_4[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE4];
__attribute__((__aligned__(8)))  static DRV_PIC32CGMAC_HW_RXDCPT sRxDesc_queue_5[TCPIP_GMAC_RX_DESCRIPTORS_COUNT_QUE5];

/*  TX descriptors list */
DRV_PIC32CGMAC_HW_TXDCPT *sTxDesc_queue[DRV_GMAC_NUMBER_OF_QUEUES] = 
                                                    {  sTxDesc_queue_0,
                                                        sTxDesc_queue_1,
                                                        sTxDesc_queue_2,
                                                        sTxDesc_queue_3,
                                                        sTxDesc_queue_4,
                                                        sTxDesc_queue_5 
                                                    };
/*  RX descriptors list */
DRV_PIC32CGMAC_HW_RXDCPT *sRxDesc_queue[DRV_GMAC_NUMBER_OF_QUEUES] = 
                                                    {  sRxDesc_queue_0,
                                                        sRxDesc_queue_1,
                                                        sRxDesc_queue_2,
                                                        sRxDesc_queue_3,
                                                        sRxDesc_queue_4,
                                                        sRxDesc_queue_5 
                                                    };
static void _EthMacReset(void)
{
    //TODO  Implement PHY Reset
}


/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibInit
 * Summary: Initialize GMAC peripheral registers
 *****************************************************************************/
void DRV_PIC32CGMAC_LibInit(DRV_GMAC_DRIVER* pMACDrv) 
{		
	//disable Tx
	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_TXEN_Msk;
	//disable Rx
	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_RXEN_Msk;
	
	//disable all GMAC interrupts for QUEUE 0
	_GMAC_REGS->GMAC_IDR.w = GMAC_INT_ALL;
	//disable all GMAC interrupts for QUEUE 1
	_GMAC_REGS->GMAC_IDRPQ[0].w = GMAC_INT_ALL;
	//disable all GMAC interrupts for QUEUE 2
	_GMAC_REGS->GMAC_IDRPQ[1].w = GMAC_INT_ALL;
	
	//Clear statistics register
	_GMAC_REGS->GMAC_NCR.w |=  GMAC_NCR_CLRSTAT_Msk;
	//Clear RX Status
	_GMAC_REGS->GMAC_RSR.w =  GMAC_RSR_RXOVR_Msk | GMAC_RSR_REC_Msk | GMAC_RSR_BNA_Msk  | GMAC_RSR_HNO_Msk;
	//Clear TX Status
	_GMAC_REGS->GMAC_TSR.w = GMAC_TSR_UBR_Msk  | GMAC_TSR_COL_Msk  | GMAC_TSR_RLE_Msk | GMAC_TSR_TXGO_Msk |
																	GMAC_TSR_TFC_Msk  | GMAC_TSR_TXCOMP_Msk  | GMAC_TSR_HRESP_Msk;
									
	//Clear Interrupt status
	_GMAC_REGS->GMAC_ISR.w;
	_GMAC_REGS->GMAC_ISRPQ[0].w ;   
	_GMAC_REGS->GMAC_ISRPQ[1].w ;
	//Set network configurations like speed, full duplex, copy all frames, no broadcast, 
	// pause enable, remove FCS, MDC clock
    uint32_t ncfgr_reg = GMAC_NCFGR_FD_Msk  |(GMAC_NCFGR_DBW_Msk & ((0) << GMAC_NCFGR_DBW_Pos)) | GMAC_NCFGR_CLK_MCK_64  |	GMAC_NCFGR_PEN_Msk  | GMAC_NCFGR_RFCS_Msk;
	_GMAC_REGS->GMAC_NCFGR.w = ncfgr_reg;
	// Set MAC address
	_GMAC_REGS->GMAC_SA[0].GMAC_SAB.w = 	  (pMACDrv->sGmacData.gmacConfig.macAddress.v[3] << 24)
															| (pMACDrv->sGmacData.gmacConfig.macAddress.v[2] << 16)
															| (pMACDrv->sGmacData.gmacConfig.macAddress.v[1] <<  8)
															| (pMACDrv->sGmacData.gmacConfig.macAddress.v[0]);
		
	_GMAC_REGS->GMAC_SA[0].GMAC_SAT.w = 	  (pMACDrv->sGmacData.gmacConfig.macAddress.v[5] <<  8)
															| (pMACDrv->sGmacData.gmacConfig.macAddress.v[4]) ;
	// MII mode config
	_GMAC_REGS->GMAC_UR.w &= ~GMAC_UR_RMII_Msk;
}




/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibInitTransfer
 * Summary :  Configure DMA and interrupts
 *****************************************************************************/
DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibInitTransfer(DRV_GMAC_DRIVER* pMACDrv,GMAC_QUE_LIST queueIdx) 
{
	
	uint16_t wRxDescCnt_temp = pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt;
	uint16_t wTxDescCnt_temp = pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt;
	uint16_t wRxBufferSize_temp = pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize;

	uint32_t wDmaCfg;
	
	if (wRxDescCnt_temp < 1 || wTxDescCnt_temp < 1)
	return DRV_PIC32CGMAC_RES_DESC_CNT_ERR;

	if (!wRxBufferSize_temp || wRxBufferSize_temp > TCPIP_GMAC_RX_MAX_FRAME)
	return DRV_PIC32CGMAC_RES_RX_SIZE_ERR;

    // Setup the interrupts for RX/TX completion (and errors) 
    if(queueIdx)
    {   //for all QUEUEs other than QUEUE 0
        if(queueIdx < DRV_GMAC_NUMBER_OF_QUEUES)
        {
            //dma configuration
            wDmaCfg = (GMAC_RBSRPQ_RBS_Msk & ((wRxBufferSize_temp >> 6) << GMAC_RBSRPQ_RBS_Pos));
            //write dma configuration to register
            _GMAC_REGS->GMAC_RBSRPQ[queueIdx - 1].w = wDmaCfg;
            //enable GMAC interrupts
            _GMAC_REGS->GMAC_IERPQ[queueIdx - 1].w = GMAC_INT_BITS;
        }
        else
        {
           return DRV_PIC32CGMAC_RES_QUEUE_ERR;
        }
    }
    else //for QUEUE 0
    {
        //dma configuration
        wDmaCfg = (GMAC_DCFGR_DRBS_Msk & ((wRxBufferSize_temp >> 6) << GMAC_DCFGR_DRBS_Pos))| (GMAC_DCFGR_RXBMS_Msk & ((3) << GMAC_DCFGR_RXBMS_Pos)) | GMAC_DCFGR_TXPBMS_Msk  | GMAC_DCFGR_FBLDO_INCR4 | GMAC_DCFGR_DDRP_Msk;
        //write dma configuration to register
        _GMAC_REGS->GMAC_DCFGR.w = wDmaCfg;
        //enable GMAC interrupts
        _GMAC_REGS->GMAC_IER.w = GMAC_INT_BITS;
    }
	return DRV_PIC32CGMAC_RES_OK;
	
}

/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibTransferEnable
 * Summary: Enable Rx and Tx of GMAC
 *****************************************************************************/
void DRV_PIC32CGMAC_LibTransferEnable (DRV_GMAC_DRIVER* pMACDrv)
{
	// Enable Rx and Tx, plus the statistics register.
	_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_TXEN_Msk;
	_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_RXEN_Msk;	
	_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_WESTAT_Msk;
	
}


/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibClose
 * Summary: Disable GMAC Rx, Tx and interrupts
 *****************************************************************************/
void DRV_PIC32CGMAC_LibClose(DRV_GMAC_DRIVER * pMACDrv, DRV_PIC32CGMAC_CLOSE_FLAGS cFlags)
{
	// disable Rx, Tx, Eth controller itself

	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_TXEN_Msk;
	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_RXEN_Msk;

	_EthMacReset();	

	_GMAC_REGS->GMAC_ISR.w;
	_GMAC_REGS->GMAC_ISRPQ[0].w;
	_GMAC_REGS->GMAC_ISRPQ[1].w;
}


/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibMACOpen
 * Summary : Open GMAC driver
 *****************************************************************************/
void DRV_PIC32CGMAC_LibMACOpen(DRV_GMAC_DRIVER * pMACDrv, TCPIP_ETH_OPEN_FLAGS oFlags, TCPIP_ETH_PAUSE_TYPE pauseType)
{	
	uint32_t ncfgr;
	
	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_TXEN_Msk;
	_GMAC_REGS->GMAC_NCR.w &= ~GMAC_NCR_RXEN_Msk;
	
	ncfgr = _GMAC_REGS->GMAC_NCFGR.w;
	
	if(oFlags & TCPIP_ETH_OPEN_FDUPLEX)
		ncfgr |= GMAC_NCFGR_FD_Msk ;
	else
		ncfgr &= ~GMAC_NCFGR_FD_Msk ;	
	
	if(oFlags & TCPIP_ETH_OPEN_100)
		ncfgr |= GMAC_NCFGR_SPD_Msk ;
	else
		ncfgr &= ~GMAC_NCFGR_SPD_Msk ;
		
	if(pauseType & TCPIP_ETH_PAUSE_TYPE_EN_RX)
		ncfgr |= GMAC_NCFGR_PEN_Msk ;
	else
		ncfgr &= ~GMAC_NCFGR_PEN_Msk ;		
	
	_GMAC_REGS->GMAC_NCFGR.w = ncfgr;
	
	if(oFlags & TCPIP_ETH_OPEN_RMII)	
	{
		_GMAC_REGS->GMAC_UR.w &= ~GMAC_UR_RMII_Msk;
	}
	else
	{
		_GMAC_REGS->GMAC_UR.w |= GMAC_UR_RMII_Msk;
	}
	
	_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_RXEN_Msk;
	_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_TXEN_Msk;

}



/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibRxBuffersAppend
 * Summary : allocate and add new RX buffers to RX descriptor
 *****************************************************************************/
DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibRxBuffersAppend(DRV_GMAC_DRIVER* pMACDrv, GMAC_QUE_LIST queueIdx, uint16_t start_index, uint16_t nDesc_Cnt) 
{
	TCPIP_MAC_PACKET*   pRxPkt;
	DRV_PIC32CGMAC_RESULT gmacRes = DRV_PIC32CGMAC_RES_OK;
	uint8_t desc_idx = start_index;
	
	while (nDesc_Cnt--)
	{
		if(pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[desc_idx] == 0)
		{
			// allocate the RX buffers
			pRxPkt = 0;
						
			//WORD ALIGN added to make the buffer alignment to 4 bytes
			pRxPkt = (*pMACDrv->sGmacData.pktAllocF)(sizeof(*pRxPkt), pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize   - sizeof(TCPIP_MAC_ETHERNET_HEADER) + WORD_ALIGN, 0);
			if(pRxPkt == NULL)
			{   // failed
				gmacRes = DRV_PIC32CGMAC_RES_OUT_OF_MEMORY;
				break;
			}			
			
			_DRV_GMAC_RxLock(pMACDrv);

			pRxPkt->pDSeg->segLoad = (uint8_t*)(((uint32_t )(pRxPkt->pDSeg->segLoad) + WORD_ALIGN) & GMAC_ADDRESS_MASK);
			/* Reset status value. */
			pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[desc_idx].rx_desc_status.val = 0;
			
			if (desc_idx == pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt - 1)
				pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[desc_idx].rx_desc_buffaddr.val =  ((uint32_t)pRxPkt->pDSeg->segLoad | GMAC_RX_WRAP_BIT);
			else
				pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[desc_idx].rx_desc_buffaddr.val =  ((uint32_t)pRxPkt->pDSeg->segLoad & GMAC_ADDRESS_MASK);
			
			_DRV_GMAC_RxUnlock(pMACDrv);
			

			// set the packet acknowledgement
			pRxPkt->ackFunc = _MacRxPacketAck;
			pRxPkt->ackParam = pMACDrv;
					
			/* Save packet pointer */
			pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[desc_idx] = pRxPkt;
			pRxPkt->next = 0;
			
			GCIRC_INC(desc_idx,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt );
			
			if(gmacRes != DRV_PIC32CGMAC_RES_OK)
			{
				(*pMACDrv->sGmacData.pktFreeF)(pRxPkt);
				break;
			}

		}

	}
	
	return gmacRes;	

} //DRV_PIC32CGMAC_LibRxBuffersAppend

/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibRxInit
 * Summary : initialize Rx Descriptors
 *****************************************************************************/
DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibRxInit(DRV_GMAC_DRIVER* pMACDrv) 
{
	uint8_t queue_idx, desc_idx;
	DRV_PIC32CGMAC_RESULT gmacRes = DRV_PIC32CGMAC_RES_OK;
		
	for(queue_idx=0; queue_idx < DRV_GMAC_NUMBER_OF_QUEUES; queue_idx++)
	{
		for(desc_idx=0; desc_idx < pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queue_idx].nRxDescCnt; desc_idx++)
		{
            pMACDrv->sGmacData.gmac_queue[queue_idx].nRxDescIndex = 0;
			pMACDrv->sGmacData.gmac_queue[queue_idx].pRxPckt[desc_idx] = 0;
			pMACDrv->sGmacData.gmac_queue[queue_idx].pRxDesc[desc_idx].rx_desc_buffaddr.val = 0;
			pMACDrv->sGmacData.gmac_queue[queue_idx].pRxDesc[desc_idx].rx_desc_status.val = 0;
		}
		pMACDrv->sGmacData.gmac_queue[queue_idx].pRxDesc[desc_idx-1].rx_desc_buffaddr.val |= GMAC_RX_WRAP_BIT;
		gmacRes = DRV_PIC32CGMAC_LibRxBuffersAppend(pMACDrv, queue_idx,0,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queue_idx].nRxDescCnt);
		
		if(gmacRes != DRV_PIC32CGMAC_RES_OK)
		{
			break;
		}
		if (!queue_idx)
			_GMAC_REGS->GMAC_RBQB.w = GMAC_RBQB_ADDR_Msk & ((uint32_t)pMACDrv->sGmacData.gmac_queue[queue_idx].pRxDesc);
		else
			_GMAC_REGS->GMAC_RBQBAPQ[queue_idx - 1].w = GMAC_RBQB_ADDR_Msk & ((uint32_t)pMACDrv->sGmacData.gmac_queue[queue_idx].pRxDesc);
	}
	
	return gmacRes;
}//DRV_PIC32CGMAC_LibRxInit

/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibTxInit
 * Summary : initialize TX Descriptors
 *****************************************************************************/
DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibTxInit(DRV_GMAC_DRIVER* pMACDrv) 
{
	uint8_t queue_idx, desc_idx;
	DRV_PIC32CGMAC_RESULT gmacRes = DRV_PIC32CGMAC_RES_OK;
	DRV_PIC32CGMAC_SGL_LIST_NODE*   pNewQueueNode;
	
	for(queue_idx=0; queue_idx < DRV_GMAC_NUMBER_OF_QUEUES; queue_idx++)
	{
		//initialise the 3 different Tx Queues
		DRV_PIC32CGMAC_SingleListInitialize(&pMACDrv->sGmacData.gmac_queue[queue_idx]._TxNewQueue);
		DRV_PIC32CGMAC_SingleListInitialize(&pMACDrv->sGmacData.gmac_queue[queue_idx]._TxStartQueue);
		DRV_PIC32CGMAC_SingleListInitialize(&pMACDrv->sGmacData.gmac_queue[queue_idx]._TxAckQueue);
		for(desc_idx=0; desc_idx < pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queue_idx].nTxDescCnt; desc_idx++)
		{
            pMACDrv->sGmacData.gmac_queue[queue_idx].nTxDescHead = 0;
            pMACDrv->sGmacData.gmac_queue[queue_idx].nTxDescTail = 0;
			pMACDrv->sGmacData.gmac_queue[queue_idx].pTxDesc[desc_idx].tx_desc_buffaddr = 0;
			pMACDrv->sGmacData.gmac_queue[queue_idx].pTxDesc[desc_idx].tx_desc_status.val = GMAC_TX_USED_BIT | GMAC_TX_LAST_BUFFER_BIT;
			
			pNewQueueNode = (*pMACDrv->sGmacData._callocF)(pMACDrv->sGmacData._AllocH,1, sizeof(*pNewQueueNode));
			DRV_PIC32CGMAC_SingleListTailAdd(&pMACDrv->sGmacData.gmac_queue[queue_idx]._TxNewQueue, (DRV_PIC32CGMAC_SGL_LIST_NODE*)pNewQueueNode); // each gmac_queue will have seperate _TxNewQueue
		}
		pMACDrv->sGmacData.gmac_queue[queue_idx].pTxDesc[desc_idx-1].tx_desc_status.val |= GMAC_TX_WRAP_BIT;
				
		if (!queue_idx)
			_GMAC_REGS->GMAC_TBQB.w = GMAC_TBQB_ADDR_Msk & ((uint32_t)pMACDrv->sGmacData.gmac_queue[queue_idx].pTxDesc);
		else
			_GMAC_REGS->GMAC_TBQBAPQ[queue_idx - 1].w = GMAC_TBQB_ADDR_Msk & ((uint32_t)pMACDrv->sGmacData.gmac_queue[queue_idx].pTxDesc);
	}
	
	return gmacRes;
}//DRV_PIC32CGMAC_LibTxInit

/****************************************************************************
 * Function:        _Calculate_Descriptor_Count
 * Summary : calculate number of Tx descriptors needed for a Tx frame packet
 *****************************************************************************/
static inline uint16_t _Calculate_Descriptor_Count (uint16_t numBytes, uint16_t buffSize )
{
	uint16_t nTxDesc = 0;
	nTxDesc = numBytes/buffSize;
	if((numBytes) % buffSize)
	{
		nTxDesc +=1;
	}
	return nTxDesc;
}



/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibTxSendPacket
 * Summary: Add Tx packet to Tx descriptors and trigger Tx Start
 *****************************************************************************/

DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibTxSendPacket(DRV_GMAC_DRIVER * pMACDrv,GMAC_QUE_LIST queueIdx)  
{
	DRV_PIC32CGMAC_RESULT     res;
	DRV_PIC32CGMAC_SGL_LIST_NODE*   txqueue_node;	
	DRV_PIC32CGMAC_HW_TXDCPT *pTxDesc = pMACDrv->sGmacData.gmac_queue[queueIdx].pTxDesc;
	DRV_PIC32CGMAC_PKT_DCPT *pPkt,*pPkt_temp;	
	uint16_t wTxIndex =0, wNewTxHead =0;
	uint16_t nRemByte =0, nLoopCnt =0;
	uint16_t txBufferSize = pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].txBufferSize;
	uint16_t wTxDescCount =pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt;	
	uint32_t nTotalDesc_count = 0; 

	res=DRV_PIC32CGMAC_RES_NO_PACKET;
	
	txqueue_node = DRV_PIC32CGMAC_SingleListHeadRemove(&pMACDrv->sGmacData.gmac_queue[queueIdx]._TxStartQueue);
	
	pPkt_temp = pPkt = (DRV_PIC32CGMAC_PKT_DCPT*)((TCPIP_MAC_PACKET *)txqueue_node->data)->pDSeg;
	
	txqueue_node->startIndex = pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescHead;
	

	//calculate the number of descriptors, even for multi packet frame
	while(pPkt_temp != 0)
	{
		nTotalDesc_count  += _Calculate_Descriptor_Count(pPkt_temp->nBytes,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].txBufferSize);
		pPkt_temp = pPkt_temp->next;
	}
	

	//check for enough number of tx descriptors available
	if(nTotalDesc_count <= (GCIRC_SPACE(pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescHead, pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescTail,
															wTxDescCount)))
	{
		wTxIndex = pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescHead ;
		wNewTxHead = fixed_mod((wTxIndex + nTotalDesc_count),wTxDescCount);
		
		while (pPkt)
		{
			//number of descriptor for each packets
			nLoopCnt = _Calculate_Descriptor_Count(pPkt->nBytes,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].txBufferSize);
			nRemByte = pPkt->nBytes;
			
			//until all the descriptors for this packet is updated
			while(nLoopCnt != 0)
			{
				pTxDesc[wTxIndex].tx_desc_status.val &= (GMAC_TX_WRAP_BIT |GMAC_TX_USED_BIT); //clear all Tx Status except Wrap Bit and Used Bit
				pTxDesc[wTxIndex].tx_desc_buffaddr = (uint32_t)(&((uint8_t *)pPkt->pBuff)[pPkt->nBytes - nRemByte]);	//set the buffer address
				if(nRemByte > txBufferSize)
				{
					pTxDesc[wTxIndex].tx_desc_status.val |= txBufferSize & GMAC_LENGTH_FRAME; //Set Length for each frame
					nRemByte -= txBufferSize;
				}
				else
				{
					pTxDesc[wTxIndex].tx_desc_status.val |= nRemByte & GMAC_LENGTH_FRAME;
					nRemByte = 0;
				}
				GCIRC_INC(wTxIndex,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt); //Increment the index of Tx Desc
				nLoopCnt--;
				
			}
			pPkt = pPkt->next;
		}
		//Set Last Buffer bit for the last descriptor of the packet frame
		GCIRC_DEC(wTxIndex,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt);
		pTxDesc[wTxIndex].tx_desc_status.val |= GMAC_TX_LAST_BUFFER_BIT;
		txqueue_node->endIndex = wTxIndex;
		
		//Clear the Used bit for all descriptors in reverse order to avoid race condition
		nLoopCnt = nTotalDesc_count;
		while(nLoopCnt != 0)
		{
			pTxDesc[wTxIndex].tx_desc_status.val &= ~GMAC_TX_USED_BIT;
			GCIRC_DEC(wTxIndex,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt);
			nLoopCnt--;
		}
		//Update new Tx Head Index
		pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescHead = wNewTxHead;
		
		//memory barrier to ensure all the memories updated before enabling transmission
        __ASM volatile ("dmb");
		//Enable Transmission
		_GMAC_REGS->GMAC_NCR.w |= GMAC_NCR_TSTART_Msk;	
		
		//Add to TxAck Queue
		DRV_PIC32CGMAC_SingleListTailAdd(&pMACDrv->sGmacData.gmac_queue[queueIdx]._TxAckQueue, txqueue_node);
		
		res=DRV_PIC32CGMAC_RES_OK;
		
	}
	else
	{ //not enough descriptors available; add back the packet to TxStartQueue head
		DRV_PIC32CGMAC_SingleListHeadAdd(&pMACDrv->sGmacData.gmac_queue[queueIdx]._TxStartQueue, txqueue_node); 
		res=DRV_PIC32CGMAC_RES_NO_DESCRIPTORS;
	}
	return res;

} //DRV_PIC32CGMAC_LibTxSendPacket



/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibTxAckPacket
 *
 * PreCondition:    None
 *
 * Input:           pMACDrv    - GMAC driver instance
 *                  queueIdx   - GMAc queue index
 *
 * Output:          DRV_PIC32CGMAC_RES_OK - success
 *                  DRV_PIC32CGMAC_RES_DESC_CNT_ERR   - descriptor error
 *
 * Side Effects:    None
 *
 * Overview:        This function acknowledges a packet.
 *                  The supplied packet has to have been completed otherwise the call will fail.
 *             
 *
 * Note:            None
 *****************************************************************************/


DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibTxAckPacket(DRV_GMAC_DRIVER * pMACDrv, GMAC_QUE_LIST queueIdx)  
{
	TCPIP_MAC_PACKET* pPkt;
	DRV_PIC32CGMAC_SGL_LIST_NODE*   pTxAckNode;
	DRV_PIC32CGMAC_RESULT res;
	uint16_t tempIndex = 0;	

	res = DRV_PIC32CGMAC_RES_DESC_CNT_ERR;		
	
	//any packet to be acknowledged in the TxAckQueue	
	while( pMACDrv->sGmacData.gmac_queue[queueIdx]._TxAckQueue.head != 0)	
	{		
		tempIndex = (pMACDrv->sGmacData.gmac_queue[queueIdx]._TxAckQueue.head)->startIndex;	
			
		//Only check the first Descriptor USED bit to see whether the buffer is used by GMAC
		if((pMACDrv->sGmacData.gmac_queue[queueIdx].pTxDesc[tempIndex].tx_desc_status.val) & GMAC_TX_USED_BIT)
		{
			pTxAckNode = DRV_PIC32CGMAC_SingleListHeadRemove(&pMACDrv->sGmacData.gmac_queue[queueIdx]._TxAckQueue); 
			pPkt  = (TCPIP_MAC_PACKET*)(pTxAckNode->data);
			pPkt->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
			pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescTail= pTxAckNode->endIndex;
				
			//increment the tail for the next packet processing
			GCIRC_INC(pMACDrv->sGmacData.gmac_queue[queueIdx].nTxDescTail,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt);
				
			// Tx Callback
			(*pMACDrv->sGmacData.pktAckF)(pPkt, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_MODULE_MAC_PIC32C);				
			pMACDrv->sGmacData._txStat.nTxOkPackets++;
						
			// Set USED bit for all the ack packets; because hardware set USED bit only for first buffer
			while(1)
			{
				pMACDrv->sGmacData.gmac_queue[queueIdx].pTxDesc[tempIndex].tx_desc_status.val |= GMAC_TX_USED_BIT;
				
				if(tempIndex == pTxAckNode->endIndex)
				{					
					break;
				}
				GCIRC_INC(tempIndex,pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nTxDescCnt);
			}
			
			//clean the Tx Node	for reusing		
			_Clear_TxNode(pTxAckNode);
				
			// add the node to Free queue
			DRV_PIC32CGMAC_SingleListTailAdd(&pMACDrv->sGmacData.gmac_queue[queueIdx]._TxNewQueue,pTxAckNode);
			
			res = DRV_PIC32CGMAC_RES_OK;
				
		}
		else
		{	
			//packet in TxAckQueue, but transmission not complete	
			break;
		}		
	}			
	return res;	

} //DRV_PIC32CGMAC_LibTxAckPacket



/****************************************************************************
 * Function:        DRV_PIC32CGMAC_LibRxGetPacket
 * Summary: extract  RX packet from the RX descriptor
 *****************************************************************************/

DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibRxGetPacket(DRV_GMAC_DRIVER * pMACDrv, TCPIP_MAC_PACKET** pRxPkt, int* pnBuffs, DRV_GMAC_RXDCPT_STATUS* pRxStat, GMAC_QUE_LIST queueIdx)  
{	
	volatile DRV_PIC32CGMAC_HW_RXDCPT *pRxDesc = pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc;
	uint16_t nRxDescIndex = pMACDrv->sGmacData.gmac_queue[queueIdx].nRxDescIndex;
	DRV_PIC32CGMAC_PKT_DCPT *pPkt;
	TCPIP_MAC_PACKET* pRxTempPkt;
	DRV_PIC32CGMAC_RESULT     res;
	
	uint16_t startIndex = 0;	
	uint16_t endIndex = 0;
	GMAC_RXFRAME_STATE frameState = GMAC_RX_NO_FRAME_STATE;	
	uint32_t frameSize = 0;
	uint16_t search_count = 0;
	uint16_t frame_count = 0; 
	uint16_t nRxDscCnt =  pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt;
	uint16_t rx_index;
	uint16_t nRx_buffer;
	
	//init return status as NO PACKET	
	res=DRV_PIC32CGMAC_RES_NO_PACKET;
    
    if(_GMAC_REGS->GMAC_RSR.w & GMAC_RSR_BNA_Msk ) //Check for BNA error due to shortage of Rx Buffers
    {
        _GMAC_REGS->GMAC_RSR.w = GMAC_RSR_BNA_Msk ; //Clear Buffer Not Available Flag
    }
	
	if(!pRxPkt)
	{
		return res;
	}
	
	//search the descriptors for valid data frame; search maximum of descriptor count
	while ( search_count < nRxDscCnt)
	{
		//Rx Descriptors with Ownership bit Set?
		if (((uint32_t)(pRxDesc[nRxDescIndex].rx_desc_buffaddr.val) & GMAC_RX_OWNERSHIP_BIT) == GMAC_RX_OWNERSHIP_BIT)
		{
			//look for the first descriptor of data frame
			if(frameState == GMAC_RX_NO_FRAME_STATE)
			{
				// Start of Frame bit set?
				if(GMAC_RX_SOF_BIT == ((uint32_t)(pRxDesc[nRxDescIndex].rx_desc_status.val) & GMAC_RX_SOF_BIT))
				{
					//transition the stat to SOF detected
					frameState = GMAC_RX_SOF_DETECTED_STATE;
					startIndex = nRxDescIndex;
					frame_count = 1; //start counting number of frames from 1
					search_count = 1; // Search maximum total number of descriptor count to find a valid frame 
					
					// End of Frame in same descriptor?	
					if(GMAC_RX_EOF_BIT == ((uint32_t)(pRxDesc[nRxDescIndex].rx_desc_status.val) & GMAC_RX_EOF_BIT))
					{
						//SOF and EOF in same descriptor; transition to EOF detected
						frameState = GMAC_RX_EOF_DETECTED_STATE;
						endIndex = nRxDescIndex;
						break;
					}
				}					
			}
			else if(frameState == GMAC_RX_SOF_DETECTED_STATE)
			{	
				//SOF detected in another descriptor; then it an error			
				if(GMAC_RX_SOF_BIT == ((uint32_t)(pRxDesc[nRxDescIndex].rx_desc_status.val) & GMAC_RX_SOF_BIT))
				{
					rx_index =  fixed_mod((startIndex + frame_count -1),nRxDscCnt);					
					nRx_buffer = frame_count;
					frameState = GMAC_RX_NO_FRAME_STATE;
					
					//clear the above descriptors
					while(nRx_buffer--)
					{
						(*pMACDrv->sGmacData.pktFreeF)(pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index]);
						pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index] = 0; //remove rx packet from rx desc
						pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[rx_index].rx_desc_buffaddr.val &= ~GMAC_ADDRESS_MASK; //clear the buffer address bitfields
						GCIRC_DEC(rx_index, nRxDscCnt); //decrement
						
					}
					
					DRV_PIC32CGMAC_LibRxBuffersAppend(pMACDrv, queueIdx, startIndex, frame_count);
					search_count--; //decrement search count
					GCIRC_DEC(nRxDescIndex, nRxDscCnt); //decrement rx index to restart searching from new SOF frame
				}
				else
				{
					frame_count++; //increment the frame count
					//EOF detected in new descriptor?
					if(GMAC_RX_EOF_BIT == ((uint32_t)(pRxDesc[nRxDescIndex].rx_desc_status.val) & GMAC_RX_EOF_BIT))
					{
						frameState = GMAC_RX_EOF_DETECTED_STATE;
						endIndex = nRxDescIndex;
						break;
					}					
				}				
			}				
		}
		else
		{
			//Ownership bit not set in an intermediate descriptor after an SOF is detected? error condition, free all the used buffers
			if(frameState == GMAC_RX_SOF_DETECTED_STATE)
			{
				rx_index = fixed_mod((startIndex + frame_count -1),nRxDscCnt);	
				nRx_buffer = frame_count;
				frameState = GMAC_RX_NO_FRAME_STATE;
								
				while(nRx_buffer--)
				{
					(*pMACDrv->sGmacData.pktFreeF)(pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index]);					
					pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index] = 0; //remove rx packet from rx desc
					pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[rx_index].rx_desc_buffaddr.val &= ~GMAC_ADDRESS_MASK; //clear the buffer address bit fields
					GCIRC_DEC(rx_index, nRxDscCnt); //decrement 
					
				}								
				DRV_PIC32CGMAC_LibRxBuffersAppend(pMACDrv, queueIdx, startIndex, frame_count);
				search_count--; //decrement search count 
				GCIRC_DEC(nRxDescIndex, nRxDscCnt); //decrement rx index to restart searching from new SOF frame			
			}
		}
		
		GCIRC_INC(nRxDescIndex, nRxDscCnt);
		if ((search_count++) >  nRxDscCnt)
			break;		
		
	}
	//Valid packet detected with SOF and EOF
	if (frameState == GMAC_RX_EOF_DETECTED_STATE)
	{		
		pRxTempPkt = *pRxPkt = pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[startIndex];
		
		// Set the index to end of curent search. This index will be used in next packet search
		GCIRC_INC(nRxDescIndex, pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt);
		pMACDrv->sGmacData.gmac_queue[queueIdx].nRxDescIndex = nRxDescIndex;
		
		*pnBuffs = frame_count; //number of buffers to return
		nRx_buffer = frame_count;		
		rx_index = startIndex;
		
		//calculate the size of frame
		frameSize = (pRxDesc[endIndex].rx_desc_status.val & GMAC_LENGTH_FRAME); // TODO: consider FCS
		
		pPkt = (DRV_PIC32CGMAC_PKT_DCPT *)(*pRxPkt)->pDSeg; //backup of data Segment for later use
		
		while(nRx_buffer--) //process all the packet buffers
		{
			(*pRxPkt)->pDSeg->segLen = (frameSize >= pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize) ?
												pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize :frameSize;
                                           			
			(*pRxPkt)->pDSeg->segSize = pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize;
			
			if(frameSize >= pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize)
				frameSize = frameSize - pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].rxBufferSize;
			else
				frameSize = 0;			
			
			pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index] = 0; //release the rx packets from Gmac queue
			pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[rx_index].rx_desc_buffaddr.val &= ~GMAC_ADDRESS_MASK; //clear the buffer address bitfields
			
			if(frameSize)
			{
				GCIRC_INC(rx_index, pMACDrv->sGmacData.gmacConfig.gmac_queue_config[queueIdx].nRxDescCnt);
				pRxTempPkt->pkt_next = pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index]; //assign pkt_next to indicate that multiple rx packets constitute this data
				pRxTempPkt = pRxTempPkt->pkt_next;
				
				(*pRxPkt)->pDSeg->next = pMACDrv->sGmacData.gmac_queue[queueIdx].pRxPckt[rx_index]->pDSeg;
				(*pRxPkt)->pDSeg = (*pRxPkt)->pDSeg->next;	
			}			
			
		}
		//pass the packet status to upper layer
		*pRxStat = (DRV_GMAC_RXDCPT_STATUS)(pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc[rx_index].rx_desc_status.val);
		
		(*pRxPkt)->pDSeg = (TCPIP_MAC_DATA_SEGMENT*)pPkt; // restore to first data segment
		
		DRV_PIC32CGMAC_LibRxBuffersAppend(pMACDrv, queueIdx, startIndex, frame_count); //allocate new packets in place of used buffers
		res = DRV_PIC32CGMAC_RES_OK;
		
	}
	
	return res;	

} //DRV_PIC32CGMAC_LibRxGetPacket



/*****************************************************************************
 ********************** Generic Single Linked List Manipulation **************
 *****************************************************************************/

// removes the head node
DRV_PIC32CGMAC_SGL_LIST_NODE*  DRV_PIC32CGMAC_SingleListHeadRemove(DRV_PIC32CGMAC_SGL_LIST* pL)  
{
	DRV_PIC32CGMAC_SGL_LIST_NODE* pN = pL->head;
    if(pN)
    {
        if(pL->head == pL->tail)
        {
            pL->head = pL->tail = 0;
        }
        else
        {
            pL->head = pN->next;
        }
        pL->nNodes--;
    }

	return pN;
}

// adds node to tail
void  DRV_PIC32CGMAC_SingleListTailAdd(DRV_PIC32CGMAC_SGL_LIST* pL, DRV_PIC32CGMAC_SGL_LIST_NODE* pN)  
{
	pN->next = 0;
	if(pL->tail == 0)
	{
		pL->head = pL->tail = pN;
	}
	else
	{
		pL->tail->next = pN;
		pL->tail = pN;
	}
    pL->nNodes++;
}

// adds node to tail
void  DRV_PIC32CGMAC_SingleListHeadAdd(DRV_PIC32CGMAC_SGL_LIST* pL, DRV_PIC32CGMAC_SGL_LIST_NODE* pN)  
{
	
	if(pN)
	{
		if(pL->head == 0)
		{
			pL->head = pL->tail = pN;
		}
		else
		{
			pN->next = pL->head;
			pL->head = pN;
			
		}
		pL->nNodes++;
	}

}


void DRV_GMAC_LibDescriptorsPoolAdd (DRV_GMAC_DRIVER * pMACDrv, DRV_GMAC_DCPT_TYPE dType)
{
    GMAC_QUE_LIST queueIdx;
    
    if(dType == DRV_GMAC_DCPT_TYPE_TX)
    {
        for(queueIdx = GMAC_QUE_0; queueIdx < DRV_GMAC_NUMBER_OF_QUEUES; queueIdx++)
        {

            pMACDrv->sGmacData.gmac_queue[queueIdx].pTxDesc = sTxDesc_queue[queueIdx];

        }
    }
    else if(dType == DRV_GMAC_DCPT_TYPE_RX)
    {
        for(queueIdx = GMAC_QUE_0; queueIdx < DRV_GMAC_NUMBER_OF_QUEUES; queueIdx++)
        {

            pMACDrv->sGmacData.gmac_queue[queueIdx].pRxDesc = sRxDesc_queue[queueIdx];

        }
    }
    
}

/****************************************************************************
 * Function:    DRV_PIC32CGMAC_LibRxFilterHash_Calculate
 * Summary :    calculate the hash value for given mac address and set the hash value in GMAC register
 *****************************************************************************/
DRV_PIC32CGMAC_RESULT DRV_PIC32CGMAC_LibRxFilterHash_Calculate(DRV_GMAC_DRIVER* pMACDrv, DRV_GMAC_HASH *hash) 
{
    uint64_t hash_value;        
      
    DRV_GMAC_HASH_INDEX hash_Index;
    DRV_GMAC_MAC_ADDR *mac_addr;

    if(hash->calculate_hash == true) // Calculate hash for given MAC address
    {
        mac_addr = (DRV_GMAC_MAC_ADDR*)(hash->DestMACAddr);

        hash_Index.index = 0;      
        hash_Index.bits.b0 = (mac_addr[5].bits.b0)^(mac_addr[5].bits.b6)^(mac_addr[4].bits.b4)^(mac_addr[3].bits.b2)^(mac_addr[2].bits.b0)^(mac_addr[2].bits.b6)^(mac_addr[1].bits.b4)^(mac_addr[0].bits.b2);
        hash_Index.bits.b1 = (mac_addr[5].bits.b1)^(mac_addr[5].bits.b7)^(mac_addr[4].bits.b5)^(mac_addr[3].bits.b3)^(mac_addr[2].bits.b1)^(mac_addr[2].bits.b7)^(mac_addr[1].bits.b5)^(mac_addr[0].bits.b3);
        hash_Index.bits.b2 = (mac_addr[5].bits.b2)^(mac_addr[4].bits.b0)^(mac_addr[4].bits.b6)^(mac_addr[3].bits.b4)^(mac_addr[2].bits.b2)^(mac_addr[1].bits.b0)^(mac_addr[1].bits.b6)^(mac_addr[0].bits.b4);
        hash_Index.bits.b3 = (mac_addr[5].bits.b3)^(mac_addr[4].bits.b1)^(mac_addr[4].bits.b7)^(mac_addr[3].bits.b5)^(mac_addr[2].bits.b3)^(mac_addr[1].bits.b1)^(mac_addr[1].bits.b7)^(mac_addr[0].bits.b5);
        hash_Index.bits.b4 = (mac_addr[5].bits.b4)^(mac_addr[4].bits.b2)^(mac_addr[3].bits.b0)^(mac_addr[3].bits.b6)^(mac_addr[2].bits.b4)^(mac_addr[1].bits.b2)^(mac_addr[0].bits.b0)^(mac_addr[0].bits.b6);
        hash_Index.bits.b5 = (mac_addr[5].bits.b5)^(mac_addr[4].bits.b3)^(mac_addr[3].bits.b1)^(mac_addr[3].bits.b7)^(mac_addr[2].bits.b5)^(mac_addr[1].bits.b3)^(mac_addr[0].bits.b1)^(mac_addr[0].bits.b7);

        // read the current hash value stored in register
        hash_value = _DRV_GMAC_HashValueGet();
        hash_value  |= (1 << (hash_Index.index));
    }
    else
    {   // Set hash value directly
        hash_value = hash->hash_value;
    }
    _DRV_GMAC_HashValueSet(hash_value);

    return DRV_PIC32CGMAC_RES_OK;
}
/****************************************************************************
 * Function:        _MacRxPacketAck
 * Summary: RX ACK function to free the RX packet
 *****************************************************************************/
static bool _MacRxPacketAck(TCPIP_MAC_PACKET* pRxPkt,  const void* param) 
{
	TCPIP_MAC_PACKET* pCurrPkt;

	DRV_GMAC_DRIVER * pMacDrv = (DRV_GMAC_DRIVER *)param;
	
			
	while(pRxPkt != 0)
	{
		pCurrPkt = pRxPkt->pkt_next;
		(*pMacDrv->sGmacData.pktFreeF)(pRxPkt);
		pRxPkt = pCurrPkt;
	}
					
	return false; 
	
}
