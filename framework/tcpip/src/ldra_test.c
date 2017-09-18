/*******************************************************************************
  -- LDRA test mock file --    
  -- last touch: 2017/03/08

NOTE: This file does NOT contain real working functions!
      Functions in this file only try to exhibit LDRA issues.
*******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <sys/kmem.h>

#include <xc.h>

typedef struct
{
    uint16_t    segLen;
    //
}TCPIP_MAC_SEGMENT;

typedef struct
{
    uint16_t    pktFlags;
    const void* pNetLayer;
    TCPIP_MAC_SEGMENT*  pDSeg;
}TCPIP_MAC_PACKET;

typedef const void* TCPIP_STACK_HEAP_HANDLE;

bool TCPIP_PKT_Initialize(TCPIP_STACK_HEAP_HANDLE heapH)
{

    bool success = false;

    while(heapH != 0)
    {
        TCPIP_MAC_PACKET* allocPtr;

        allocPtr = malloc(sizeof(TCPIP_MAC_PACKET));

        if(allocPtr == 0)
        {
            break;
        }

        // LDRA Violation: no prior declaration for IS_KVA
        if(!IS_KVA(allocPtr))
        {  
            break;
        }
        // success
        success = true;
        break;
    }


    return success;
    
}

typedef const void* TCPIP_HEAP_EXT_OBJ_INSTANCE;

TCPIP_STACK_HEAP_HANDLE TCPIP_HEAP_CreateExternal(const void* pHeapConfig, int* pRes)
{
    // LDRA Violation: Procedure contains UR data flow anomalies. : hInst
    // UR anomaly, variable used before assignment
    TCPIP_HEAP_EXT_OBJ_INSTANCE hInst;
    int  res;
    

    while(true)
    {
        hInst = 0;

        if( pHeapConfig == 0)
        {
            res = -1;
            break;
        }

        hInst = (TCPIP_HEAP_EXT_OBJ_INSTANCE)calloc(1, sizeof(hInst));

        if(hInst == 0)
        {
            res = -2;
            break;
        }

        res = 0;
        break;
    }

    if(pRes)
    {
        *pRes = res;
    }

    // LDRA Violation: Attempt to use uninitialised pointer. : hInst
    return hInst;
    
}

typedef const void* TEST_ENTRY;
typedef TEST_ENTRY*(*TEST_DEL_FUNC)(TEST_ENTRY*);

void TestRemoveEntry(TEST_ENTRY delEntry)
{
}

TEST_ENTRY*   TestLookup(TEST_ENTRY* pOH, TEST_DEL_FUNC delF, int nSlots, int nEntries)
{
    TEST_ENTRY   *pBkt, *pDel;

    pBkt = (TEST_ENTRY*)malloc(100);
    if(pBkt == 0)
    {
        if(nSlots != nEntries)
        {   
            return 0;
        }
        
        if(delF == 0 || (pDel = (*delF)(pOH)) == 0)
        {   
            return 0;
        }

        // LDRA Violation: Attempt to use uninitialised pointer. : pDel
        TestRemoveEntry(pDel);
        pBkt = (TEST_ENTRY*)malloc(100);
        if(pBkt == 0)
        {   
            return 0;
        }
    }

    return pBkt;
}

typedef struct
{
    uint16_t    port;
    uint16_t    flags;  // TCPIP_HELPER_PORT_FLAGS value
}TCPIP_HELPER_PORT_ENTRY;

typedef enum
{
    TCPIP_HELPER_PORT_FLAG_STREAM   = 0x01,     // stream socket port
    TCPIP_HELPER_PORT_FLAG_DGRAM    = 0x02,     // datagram socket port

}TCPIP_HELPER_PORT_FLAGS;

static TCPIP_HELPER_PORT_ENTRY* _TCPIP_Helper_SecurePortEntry(uint16_t port, TCPIP_HELPER_PORT_ENTRY** pFreeEntry)
{

    return 0;
}

bool TCPIP_Helper_SecurePortSet(uint16_t port, bool streamSocket, bool isSecure)
{
    TCPIP_HELPER_PORT_ENTRY* pEntry, *pFree = 0;

    if(port == 0)
    {   // invalid port
        return false;
    }

    TCPIP_HELPER_PORT_FLAGS entryFlags = streamSocket ? TCPIP_HELPER_PORT_FLAG_STREAM : TCPIP_HELPER_PORT_FLAG_DGRAM;

    pEntry = _TCPIP_Helper_SecurePortEntry(port, &pFree);

    if(isSecure)
    {
        if(pEntry == 0)
        {   // not in there already
            // LDRA Violation: Attempt to use uninitialised pointer. : pFree
            if(pFree == 0)
            {   // no more room
                return false;
            }
            pFree->port = port;
            pFree->flags = entryFlags;
        }
        else
        {   // make sure is flagged properly
            pEntry->flags |= entryFlags;
        }
        return true;
    }

    // non secure port: need to remove
    if(pEntry)
    {
        if((pEntry->flags &= ~entryFlags) == 0)
        {   // done, free it
            pEntry->port = 0;
        }
    }
    // else not even in there

    return true;
}



typedef struct
{
}ARP_PACKET;

static TCPIP_MAC_PACKET* pArpPkt = 0;


bool ARPSendIfPkt(void)
{
    TCPIP_MAC_PACKET* pMacPkt;

    if(pArpPkt != 0 && (pArpPkt->pktFlags & 0x01) == 0)
    {
        pMacPkt = pArpPkt;
    }
    else
    {   // packet not available, have to allocate another one
        if((pMacPkt = malloc(sizeof(*pMacPkt))) == 0)
        {
            return false;
        }
        pArpPkt = pMacPkt;   // show we're using this one now
    }



    // LDRA Violation: Attempt to use uninitialised pointer. : pMacPkt
    pMacPkt->pDSeg->segLen = sizeof(ARP_PACKET);
    return true;

}

typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t  v[4];
} TCPIP_IPV4_ADDR;

typedef union
{
    uint32_t Val;
    uint8_t  v[4];
} TCPIP_UINT32_VAL;

typedef union 
{
    uint16_t Val;
    uint8_t v[2];
} TCPIP_UINT16_VAL;


bool TCPIP_Helper_StringToIPAddress(const char* str, TCPIP_IPV4_ADDR* addr)
{
    // LDRA Violation:Procedure contains UR data flow anomalies. : dwVal.v
    // UR anomaly, variable used before assignment.
	TCPIP_UINT32_VAL dwVal;
	uint8_t i, charLen, currentOctet;

	charLen = 0;
	currentOctet = 0;
	dwVal.Val = 0;
    addr->Val = 0;

	while((i = *str++))
	{
		if(currentOctet > 3u)
			break;

		i -= '0';
		

		// Validate the character is a numerical digit or dot, depending on location
		if(charLen == 0u)
		{
			if(i > 9u)
				return false;
		}
		else if(charLen == 3u)
		{
			if(i != (uint8_t)('.' - '0'))
				return false;

			if(dwVal.Val > 0x00020505ul)
				return false;

			addr->v[currentOctet++] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];
			charLen = 0;
			dwVal.Val = 0;
			continue;
		}
		else
		{
			if(i == (uint8_t)('.' - '0'))
			{
				if(dwVal.Val > 0x00020505ul)
					return false;

				addr->v[currentOctet++] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];
				charLen = 0;
				dwVal.Val = 0;
				continue;
			}
			if(i > 9u)
				return false;
		}

		charLen++;
		dwVal.Val <<= 8;
		dwVal.v[0] = i;
	}

	// Make sure the very last character is a valid termination character 
	if(currentOctet != 3 || (i != 0u && i != '/' && i != '\r' && i != '\n' && i != ' ' && i != '\t' && i != ':'))
		return false;

	// Verify and convert the last octet and return the result
	if(dwVal.Val > 0x00020505ul)
		return false;

	addr->v[3] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];

	return true;
}


bool TCPIP_Helper_StringToMACAddress(const char* str, uint8_t macAddr[6])
{
    const char  *beg;
    // LDRA Violation: Procedure contains UR data flow anomalies. : hexDigit.Val
    // UR anomaly, variable used before assignment.
    TCPIP_UINT16_VAL    hexDigit;
    int         ix;
    

    beg = str;
    for(ix=0; ix<6; ix++)
    {
        if(!isxdigit(beg[0]) || !isxdigit(beg[1]))
        {
            return false;
        }

        // found valid byte
        hexDigit.v[0] = beg[1];
        hexDigit.v[1] = beg[0];
        *macAddr++ = (hexDigit.v[0] << 4) | (hexDigit.v[1]);

        // next colon number
        beg += 2;
        if(beg[0] == '\0')
        {
            break;  // done
        }
        else if(beg[0] != ':' && beg[0] != '-')
        {
            return false;   // invalid delimiter
        }
        beg++; // next digit
    }
    
    return ix==5?true:false;    // false if not enough digits    
    
}


static uint32_t arpInitCount = 0;

typedef struct
{
    uint32_t    entrySolvedTmo;
    uint32_t    entryPendingTmo;
    uint32_t    entryRetryTmo;
    uint32_t    ermQuota;
    uint32_t    entryRetries;
    uint32_t    entryGratRetries;
    uint32_t    permQuota;
    uint32_t    retries;
    const void* arpCacheDcpt;
}TCPIP_ARP_MODULE_CONFIG;

typedef struct
{
}OA_HASH_DCPT;

typedef struct
{
    void* head;
}SOME_LIST;


static bool TCPIP_Helper_ProtectedSingleListInitialize(SOME_LIST* pList)
{
    if(pList)
    {
        pList->head = 0;
        return true;
    }
    return false;
}


typedef struct
{
    OA_HASH_DCPT* hashDcpt;
    SOME_LIST   permList;
    SOME_LIST   completeList;
    SOME_LIST   incompleteList;
}ARP_CACHE_DCPT;



static TCPIP_ARP_MODULE_CONFIG arpModData = { 0 };
static ARP_CACHE_DCPT           arpCacheDcpt = { 0 };


bool TCPIP_ARP_Initialize(const TCPIP_ARP_MODULE_CONFIG* arpData)
{
    OA_HASH_DCPT*   hashDcpt;
    ARP_CACHE_DCPT* pArpDcpt;
    size_t          hashMemSize;
    int             ix;
    // LDRA Violation: Procedure contains UR data flow anomalies. : iniRes
    bool            iniRes;


    if(arpInitCount == 0)
    {   // first time we're run
        // check initialization data is provided
        if(arpData == 0)
        {
            return false;
        }


        // parameters initialization
        arpModData.entrySolvedTmo = arpData->entrySolvedTmo;
        arpModData.entryPendingTmo = arpData->entryPendingTmo;
        arpModData.entryRetryTmo = arpData->entryRetryTmo;
        arpModData.permQuota = arpData->permQuota;
        arpModData.retries = arpData->retries;
        arpModData.entryGratRetries =  arpData->entryGratRetries;


        if(arpModData.arpCacheDcpt == 0)
        {
            arpModData.arpCacheDcpt = (ARP_CACHE_DCPT*)calloc(1, sizeof(arpCacheDcpt)); 
            if(arpModData.arpCacheDcpt == 0)
            {   // failed
                return false;
            }

            hashMemSize = 100;
            pArpDcpt = &arpCacheDcpt;
            for(ix = 0; ix < 2; ix++)
            {
                hashDcpt = (OA_HASH_DCPT*)malloc(hashMemSize);

                if(hashDcpt == 0)
                {   // failed
                    return false;
                }

                arpCacheDcpt.hashDcpt = hashDcpt;
                while(true)
                {
                    if((iniRes = TCPIP_Helper_ProtectedSingleListInitialize(&pArpDcpt->permList)) == false)
                    {
                        break;
                    }

                    if((iniRes = TCPIP_Helper_ProtectedSingleListInitialize(&pArpDcpt->completeList)) == false)
                    {
                        break;
                    }

                    iniRes = TCPIP_Helper_ProtectedSingleListInitialize(&pArpDcpt->incompleteList);
                    break;
                }

                if(iniRes == false)
                {
                    return false;
                }

            }

        }
    }

    
    arpInitCount++;

    return true;
}

uint16_t TCPIP_Helper_CalcIPChecksum(uint8_t* buffer, uint16_t count, uint16_t seed)
{
	uint16_t i;
	uint16_t *val;
    // LDRA Violation: Procedure contains UR data flow anomalies. : sum.b
    // UR anomaly, variable used before assignment.
	union
	{
		uint8_t  b[4];
		uint16_t w[2];
		uint32_t dw;
	} sum;

	val = (uint16_t*)buffer;

	// Calculate the sum of all words
	sum.dw = (uint32_t)seed;
    if ((unsigned int)buffer % 2)
    {
        sum.w[0] += (*(uint8_t *)buffer) << 8;
        val = (uint16_t *)(buffer + 1);
        count--;
    }

	i = count >> 1;

	while(i--)
		sum.dw += (uint32_t)*val++;

	// Add in the sum of the remaining byte, if present
	if(count & 0x1)
		sum.dw += (uint32_t)*(uint8_t*)val;

	// Do an end-around carry (one's complement arrithmatic)
	sum.dw = (uint32_t)sum.w[0] + (uint32_t)sum.w[1];

	// Do another end-around carry in case if the prior add 
	// caused a carry out
	sum.w[0] += sum.w[1];

    if ((unsigned int)buffer % 2)
    {
        sum.w[0] = ((uint16_t)sum.b[0] << 8 ) | (uint16_t)sum.b[1];
    }

	// Return the resulting checksum
	return ~sum.w[0];
}


// simple TLV constructor, type and length only
uint8_t* tl_tlvConstruct(uint8_t type, uint16_t tlvLen, uint8_t* pBuff)
{
    if(pBuff)
    {
        // LDRA Violation: Procedure contains UR data flow anomalies. : tl.v
        TCPIP_UINT16_VAL tl;

        tl.Val = type;
        tl.Val <<= 9;
        tl.Val |= tlvLen;

        *pBuff++ = tl.v[1];
        *pBuff++ = tl.v[0];
    }

    return pBuff;
}

typedef struct
{
    uint8_t*    pEDBuff;
    union
    {
        uint16_t    w;
        struct
        {
            uint16_t npv    : 1;
            uint16_t eown   : 1;
            uint16_t sticky : 1;
            uint16_t rx_nack : 1;
            uint16_t kv0 : 1;
            uint16_t reserved : 11;
        };
    }hdr;
}DRV_ETHMAC_DCPT_NODE;

typedef struct
{
    DRV_ETHMAC_DCPT_NODE* head;
    int         nNodes;

}DRV_ETHMAC_DCPT_LIST;

static DRV_ETHMAC_DCPT_LIST* _EnetRxFreePtr = 0;
static DRV_ETHMAC_DCPT_LIST* _EnetRxBusyPtr = 0;


static DRV_ETHMAC_DCPT_NODE* DRV_ETHMAC_LIB_ListRemoveHead(DRV_ETHMAC_DCPT_LIST* pList)
{
    return pList ?  pList->head : 0;
}

static void DRV_ETHMAC_LIB_ListAddTail(DRV_ETHMAC_DCPT_LIST* pList, DRV_ETHMAC_DCPT_NODE* pNode)
{
    pList->head = pNode;
}

static void DRV_ETHMAC_LIB_ListAppendTail(DRV_ETHMAC_DCPT_LIST* pList, DRV_ETHMAC_DCPT_LIST* pNewList)
{
    pNewList->head = pList->head;
}

static bool DRV_ETHMAC_LIB_ListIsEmpty(DRV_ETHMAC_DCPT_LIST* pList)
{
    return pList->head == 0;
}

static DRV_ETHMAC_DCPT_LIST* DRV_ETHMAC_LIB_ListInit(DRV_ETHMAC_DCPT_LIST* pList)
{
    pList->head = 0;
    pList->nNodes = 0;
    return pList;
}

static void _EthAppendBusyList(DRV_ETHMAC_DCPT_LIST* pBusy, DRV_ETHMAC_DCPT_LIST* pNewList, int nNodes)
{
    pBusy->head = pNewList->head;
    pBusy->nNodes += nNodes;
}

int DRV_ETHMAC_LibRxBuffersAppend(void* ppBuff[], int nBuffs, uint16_t rxFlags)
{
    // LDRA Violation: Procedure contains UR data flow anomalies. : pBuff
    void*       pBuff;
    DRV_ETHMAC_DCPT_NODE   *pEDcpt;
    int     res;
    DRV_ETHMAC_DCPT_LIST*   pNewList;
    uint8_t newList [15 + sizeof(DRV_ETHMAC_DCPT_LIST)];

    if(nBuffs==0)
    {
        nBuffs=0x7fffffff;
    }

    pNewList = DRV_ETHMAC_LIB_ListInit((DRV_ETHMAC_DCPT_LIST*)newList);

    res=0;

    // LDRA Violation: Attempt to use uninitialised pointer. : pBuff
    for(pBuff=*ppBuff; pBuff!=0 && nBuffs; pBuff=*(++ppBuff), nBuffs--)
    {
        pEDcpt=DRV_ETHMAC_LIB_ListRemoveHead(_EnetRxFreePtr);
        if(!pEDcpt)
        {   // we've run out of descriptors...
            res=-1;
            break;
        }
        // ok valid descriptor
        // pas it to hw, always use linked descriptors
        pEDcpt->pEDBuff=(uint8_t*)KVA_TO_PA(pBuff);

        pEDcpt->hdr.w = 0x03;   // hw owned

        if((rxFlags&0x01) != 0)
        {
            pEDcpt->hdr.sticky=1;
        }

        if((rxFlags&0x02) != 0)
        {
            pEDcpt->hdr.rx_nack=1;
        }

        if(IS_KVA0(pBuff))
        {
            pEDcpt->hdr.kv0=1;
        }
        else if(!IS_KVA(pBuff))
        {
            res=-2;
            break;
        }

        DRV_ETHMAC_LIB_ListAddTail(pNewList, pEDcpt);
    }

    if(pBuff && nBuffs)
    {   // failed, still buffers in the descriptors, put back the removed nodes
        DRV_ETHMAC_LIB_ListAppendTail(_EnetRxFreePtr, pNewList);
        return res;
    }

    // all's well
    if(!DRV_ETHMAC_LIB_ListIsEmpty(pNewList))
    {
        _EthAppendBusyList(_EnetRxBusyPtr, pNewList, 1);
    }

    return 0;

} //DRV_ETHMAC_LibRxBuffersAppend

typedef struct __attribute__((__packed__))
{
    uint8_t v[6];
} TCPIP_MAC_ADDR;

static void PLIB_ETH_RxFiltersClr(void)
{
}

static void PLIB_ETH_RxFiltersHTSet(uint32_t filt)
{
    ETHHT0 = filt;
}

static void PLIB_ETH_RxFiltersSet(uint32_t filt)
{
    ETHHT1 = filt;
}

int DRV_ETHMAC_PIC32MACRxFilterHashTableEntrySet(TCPIP_MAC_ADDR* DestMACAddr)
{
    volatile unsigned int*    pHTSet;
    uint8_t                      hVal;
    int                       i, j;
    uint8_t                      nullMACAddr[6] =   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    typedef union
    {
        uint8_t Val;
        struct __attribute__((packed))
        {
            uint8_t b0:1;
            uint8_t b1:1;
            uint8_t b2:1;
            uint8_t b3:1;
            uint8_t b4:1;
            uint8_t b5:1;
            uint8_t b6:1;
            uint8_t b7:1;
        } bits;
    }crc_byte;

    // LDRA Violation: Procedure contains UR data flow anomalies. : crc32_val.v.bits.b7
    // UR anomaly, variable used before assignment.
    union
    {
        uint32_t Val;
        crc_byte v[4];
        struct __attribute__((packed))
        {
            uint32_t b0_23: 24;

            uint32_t b24:1;
            uint32_t b25:1;
            uint32_t b26:1;
            uint32_t b27:1;
            uint32_t b28:1;
            uint32_t b29:1;
            uint32_t b30:1;
            uint32_t b31:1;
        } bits;
    } crc32_val;

    crc_byte    macByte;



    crc32_val.Val = 0xFFFFFFFF;


    if( DestMACAddr == 0 || memcmp(DestMACAddr->v, nullMACAddr, sizeof(nullMACAddr))==0 )
    {
        // Disable the Hash Table receive filter and clear the hash table
        PLIB_ETH_RxFiltersClr();
        PLIB_ETH_RxFiltersHTSet(0);
        return 0;
    }


    for(i = 0; i < sizeof(TCPIP_MAC_ADDR); i++)
    {
        uint8_t  crcnext;

        // shift in 8 bits
        for(j = 0; j < 8; j++)
        {
            crcnext = 0;
            if( crc32_val.v[3].bits.b7)
            {
                crcnext = 1;
            }
            macByte.Val = DestMACAddr->v[i];
            crcnext ^= macByte.bits.b0;

            crc32_val.Val <<= 1;
            if(crcnext)
            {
                crc32_val.Val ^= 0x4C11DB7;
            }
            // next bit
            DestMACAddr->v[i] >>= 1;
        }
    }

    pHTSet = (crc32_val.bits.b28)? &ETHHT1SET : &ETHHT0SET;
    hVal = (crc32_val.Val >> 23)&0x1f;
    *pHTSet = 1 << hVal;

    PLIB_ETH_RxFiltersSet(0x11);

    return 0;

}

typedef struct _tag_DRV_ETHMAC_PKT_DCPT
{
    struct _tag_DRV_ETHMAC_PKT_DCPT* next;
    uint8_t*    pBuff;
    uint32_t    nBytes;
}DRV_ETHMAC_PKT_DCPT;

typedef struct
{
    uint32_t    nRxBytes;
    // etc, etc...
}DRV_ETHMAC_PKT_STAT_RX;

int DRV_ETHMAC_LibRxGetPacket(DRV_ETHMAC_PKT_DCPT* pPkt, int* pnBuffs, const DRV_ETHMAC_PKT_STAT_RX** pRxStat)
{
    int     res;

    res = -1;

    if(pPkt)
    {
        pPkt->pBuff=0;
        pPkt->nBytes=0;

        res = 0;
    }

    return res;
}


int DRV_ETHMAC_LibRxGetBuffer(void** ppBuff, const DRV_ETHMAC_PKT_STAT_RX** pRxStat)
{

    int         res;
    // LDRA Violation: Procedure contains UR data flow anomalies. : pktDcpt.nBytes, pktDcpt.pBuff
    DRV_ETHMAC_PKT_DCPT     pktDcpt;
    int             nBuffs;     // buffers per packet

    pktDcpt.next=0;     // create a single buffer packet descriptor;

    res=DRV_ETHMAC_LibRxGetPacket(&pktDcpt, &nBuffs, pRxStat);

    *ppBuff=pktDcpt.pBuff;

    return res;

} //DRV_ETHMAC_LibRxGetBuffer

