/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    aes.c

  Summary:
    Crypto Framework Library source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic
	Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  aes_pic32c.c
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


#ifndef NO_AES
#include <stdint.h>

#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */
#include "aes.h"

#include "crypto/crypto.h"
#include "crypto/src/error-crypt.h"
#include "crypto/src/logging.h"

#undef COMPONENT_TYPEDEF_STYLE
#define COMPONENT_TYPEDEF_STYLE 'N'


/* These should all line up with CMSIS */
/** AES processing mode. */
enum aes_encrypt_mode {
    AES_PIC32C_DECRYPTION = 0, /**< Decryption of data will be performed. */
    AES_PIC32C_ENCRYPTION,     /**< Encryption of data will be performed. */
};

/** AES cryptographic key size. */
enum aes_key_size {
    AES_KEY_SIZE_128 = 0,   /**< AES key size is 128 bits. */
    AES_KEY_SIZE_192,       /**< AES key size is 192 bits. */
    AES_KEY_SIZE_256,       /**< AES key size is 256 bits. */
};

/** AES start mode. */
enum aes_start_mode {
    AES_MANUAL_START = 0,   /**< Manual start mode. */
    AES_AUTO_START,         /**< Auto start mode. */
    AES_IDATAR0_START,      /**< AES_IDATAR0 access only Auto Mode. */
};

/** AES cipher block mode. */
enum aes_opmode {
    AES_ECB_MODE = 0,       /**< Electronic Codebook (ECB). */
    AES_CBC_MODE,           /**< Cipher Block Chaining (CBC). */
    AES_OFB_MODE,           /**< Output Feedback (OFB). */
    AES_CFB_MODE,           /**< Cipher Feedback (CFB). */
    AES_CTR_MODE,           /**< Counter (CTR). */
    AES_GCM_MODE,           /**< Galois Counter Mode (GCM).*/
};

/** AES Cipher FeedBack (CFB) size. */
enum aes_cfb_size {
    AES_CFB_SIZE_128 = 0,   /**< Cipher feedback data size is 128-bit. */
    AES_CFB_SIZE_64,        /**< Cipher feedback data size is 64-bit. */
    AES_CFB_SIZE_32,        /**< Cipher feedback data size is 32-bit. */
    AES_CFB_SIZE_16,        /**< Cipher feedback data size is 16-bit. */
    AES_CFB_SIZE_8,         /**< Cipher feedback data size is 8-bit. */
};

/** AES interrupt source type. */
typedef enum aes_interrupt_source {
    /** Data ready interrupt.*/
    AES_INTERRUPT_DATA_READY = AES_IER_DATRDY_Msk,
    /** Unspecified register access detection interrupt.*/
    AES_INTERRUPT_UNSPECIFIED_REGISTER_ACCESS = AES_IER_URAD_Msk,

    AES_INTERRUPT_TAG_READY = AES_IER_TAGRDY_Msk,
} aes_interrupt_source_t;


#define AES_INTERRUPT_SOURCE_NUM 3


/** AES interrupt callback function type. */
typedef void (*aes_callback_t)(void);


/* AES configuration */
struct aes_config
{
    /** AES data mode (decryption or encryption). */
    enum aes_encrypt_mode encrypt_mode;

    /** AES key size. */
    enum aes_key_size key_size;

    /** Start mode. */
    enum aes_start_mode start_mode;

    /** AES block cipher operation mode.*/
    enum aes_opmode opmode;

    /** Cipher feedback data size. */
    enum aes_cfb_size cfb_size;

    /** Last output data mode enable/disable. */
    int lod;

    /** Galois Counter Mode (GCM) automatic tag generation enable/disable */
    int gtag_en;

    /** Processing delay parameter. */
    uint32_t processing_delay;
} ;


/* our local configuration */
static struct aes_config aes_configuration;



/* helper to compose the configuration for the AES */
static void AesConfigure(struct aes_config *const p_cfg)
{
    uint32_t ul_mode = 0;

    /* Set processing mode */
    if (p_cfg->encrypt_mode == AES_PIC32C_ENCRYPTION)
    {
        /* 0 = decrypt., 1 = encrypt */
        ul_mode |= AES_MR_CIPHER_Msk;
    }

    /* Active dual buffer in DMA mode */
    if (p_cfg->start_mode == AES_IDATAR0_START)
    {
        ul_mode |= AES_MR_DUALBUFF_ACTIVE;
    }

    /* Set start mode */
    ul_mode |= (p_cfg->start_mode << AES_MR_SMOD_Pos);

    /* Set key size */
    ul_mode |= (p_cfg->key_size << AES_MR_KEYSIZE_Pos);

    /* Set Confidentiality mode */
    ul_mode |= (p_cfg->opmode << AES_MR_OPMOD_Pos);

    /* Set CFB size */
    ul_mode |= (p_cfg->cfb_size << AES_MR_CFBS_Pos);

    /* Last Output Data */
    if (p_cfg->lod)
    {
        ul_mode |= AES_MR_LOD_Msk;
    }

    if ((p_cfg->opmode == AES_GCM_MODE) && (p_cfg->gtag_en))
    {
        ul_mode |= AES_MR_GTAGEN_Msk;
    }

    ul_mode |= AES_MR_PROCDLY(p_cfg->processing_delay);

    ul_mode |= AES_MR_CKEY_PASSWD;

    _AES_REGS->AES_MR.w = ul_mode;
}



/* key length enters as number of bytes but we do words so / by 4 */
static void AesWriteKey(const word32 *AesKey, int key_length)
{
    if (AesKey)
    {
        uint8_t i;

        key_length /= 4;
        for (i = 0; i < key_length; i++)
        {
            _AES_REGS->AES_KEYWR[i].KEYW = *AesKey++;
        }
    }
}


static void AesWriteIV(word32 *iv)
{
    /* put AES IV into place */
    _AES_REGS->AES_IVR[0].w = iv[0];
    _AES_REGS->AES_IVR[1].w = iv[1];
    _AES_REGS->AES_IVR[2].w = iv[2];
    _AES_REGS->AES_IVR[3].w = iv[3];
}



/* Start actions */
int wc_AesSetKey(Aes* aes, const byte* userKey, word32 keylen, const byte* iv, int dir)
{
    /* valid key length = 16, 24, 32 */
    if (!((keylen == 16) || (keylen == 24) || (keylen == 32)))
    {
        return BAD_FUNC_ARG;
    }

    /* Enable clock for AES */
    uint32_t AesPmcBit = 1u << (ID_AES - 32);
    if ((_PMC_REGS->PMC_PCSR1.w & (AesPmcBit)) != (AesPmcBit))
    {
        _PMC_REGS->PMC_PCER1.w = AesPmcBit;

        /* software reset */
        _AES_REGS->AES_CR.w = AES_CR_SWRST_Msk;

    }

    /* save the particulars about the key and IV */
    memcpy(aes->key_ce, userKey, keylen);
    aes->keylen = keylen;
    wc_AesSetIV(aes, iv);

    return 0;
}




#if defined(WOLFSSL_AES_DIRECT) || defined(WOLFSSL_AES_COUNTER)

/* AES-CTR and AES-DIRECT need to use this for key setup, no aesni yet */
int wc_AesSetKeyDirect(Aes* aes,
                        const byte* userKey,
                        word32 keylen,
                        const byte* iv,
                        int dir)
{
    /* pass this on */
    return wc_AesSetKey(aes, userKey, keylen, iv, dir);
}

#endif /* WOLFSSL_AES_DIRECT || WOLFSSL_AES_COUNTER */



/* wc_AesSetIV is shared between software and hardware */
int wc_AesSetIV(Aes* aes, const byte* iv)
{
    if (iv)
    {
        memcpy((void*)aes->iv_ce, (void*)iv, AES_BLOCK_SIZE);
    }
    else
    {
        memset((void*)aes->iv_ce, 0, AES_BLOCK_SIZE);
    }

    return 0;
}



#if defined(WOLFSSL_AES_DIRECT)

/* AES-DIRECT */
void wc_AesEncryptDirect(Aes* aes, byte* out, const byte* in)
{
    /* one block is 4 32bit words, 16 bytes */
    wc_AesCbcEncrypt(aes, out, in, AES_BLOCK_SIZE);
}



/* Allow direct access to one block decrypt */
void wc_AesDecryptDirect(Aes* aes, byte* out, const byte* in)
{
    /* one block is 4 32bit words, 16 bytes */
    wc_AesCbcDecrypt(aes, out, in, AES_BLOCK_SIZE);
}

#endif  /* AES DIRECT */




/* AES-CBC */
#ifdef HAVE_AES_CBC


int wc_AesCbcEncrypt(Aes* aes, byte* out, const byte* in, word32 sz)
{
    /* set all the fields needed to set-up the AES engine */
    aes_configuration.lod = 0;
    aes_configuration.gtag_en = 0;
    aes_configuration.processing_delay = 0;
    aes_configuration.opmode = AES_CBC_MODE;
    aes_configuration.start_mode = AES_AUTO_START;
    aes_configuration.cfb_size = AES_CFB_SIZE_128;
    aes_configuration.encrypt_mode = AES_PIC32C_ENCRYPTION;

    /* keylen is in bytes from call - CMSIS is enum */
    switch (aes->keylen)
    {
        default:
        case 16: aes_configuration.key_size = AES_KEY_SIZE_128; break;
        case 24: aes_configuration.key_size = AES_KEY_SIZE_192; break;
        case 32: aes_configuration.key_size = AES_KEY_SIZE_256; break;
    }

    AesConfigure(&aes_configuration);
    AesWriteKey(aes->key_ce, aes->keylen);
    AesWriteIV(aes->iv_ce);


    /* set up pointers for input and output buffers */
    const uint32_t *inptr = (const uint32_t *)in;
    uint32_t *outptr = (uint32_t *)out;

    uint32_t block;   /* 16 bytes = 4 32bit block size */
    for (block = 0; block < sz; block += 16)
    {
        /* Write the data to be ciphered to the input data registers. */
        _AES_REGS->AES_IDATAR[0].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[1].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[2].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[3].IDATA = *inptr++;

        /* Note the blocking here - state machine this? */
        while (!(_AES_REGS->AES_ISR.w & AES_ISR_DATRDY_Msk))  ;

        /* encrypt complete - read out the data */
        *outptr++ = _AES_REGS->AES_ODATAR[0].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[1].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[2].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[3].ODATA;
    }

    /* Last IV equals last cipher text */
    wc_AesSetIV(aes, out + sz - 16);

    return 0;
}



#ifdef HAVE_AES_DECRYPT
int wc_AesCbcDecrypt(Aes* aes, byte* out, const byte* in, word32 sz)
{
    aes_configuration.encrypt_mode = AES_PIC32C_DECRYPTION;
    aes_configuration.key_size = AES_KEY_SIZE_128;
    aes_configuration.start_mode = AES_AUTO_START;
    aes_configuration.opmode = AES_CBC_MODE;
    aes_configuration.cfb_size = AES_CFB_SIZE_128;
    aes_configuration.lod = 0;
    aes_configuration.gtag_en = 0;
    aes_configuration.processing_delay = 0;

    AesConfigure(&aes_configuration);
    AesWriteKey(aes->key_ce, aes->keylen);
    AesWriteIV(aes->iv_ce);

    /* set up pointers for input and output buffers */
    const uint32_t *inptr = (const uint32_t *)in;
    uint32_t *outptr = (uint32_t *)out;

    uint32_t block;   /* 16 bytes = 4 32bit block size */
    for (block = 0; block < sz; block += 16)
    {
        /* Write the data to be ciphered to the input data registers. */
        _AES_REGS->AES_IDATAR[0].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[1].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[2].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[3].IDATA = *inptr++;

        /* Note the blocking here - state machine this? */
        while (!(_AES_REGS->AES_ISR.w & AES_ISR_DATRDY_Msk))  ;

        /* encrypt complete - read out the data */
        *outptr++ = _AES_REGS->AES_ODATAR[0].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[1].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[2].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[3].ODATA;
    }

    /* Last IV equals last cipher text */
    wc_AesSetIV(aes, in + sz - 16);

    return 0;
}
#endif /* HAVE_AES_DECRYPT */


#endif /* HAVE_AES_CBC */



/* AES-CTR */
#ifdef WOLFSSL_AES_COUNTER

int wc_AesCtrEncrypt(Aes* aes, byte* out, const byte* in, word32 sz)
{
    aes_configuration.encrypt_mode = AES_PIC32C_ENCRYPTION;
    aes_configuration.key_size = AES_KEY_SIZE_128;
    aes_configuration.start_mode = AES_AUTO_START;
    aes_configuration.opmode = AES_CTR_MODE;
    aes_configuration.cfb_size = AES_CFB_SIZE_128;
    aes_configuration.lod = 0;
    aes_configuration.gtag_en = 0;
    aes_configuration.processing_delay = 0;

    AesConfigure(&aes_configuration);
    AesWriteKey(aes->key_ce, aes->keylen);
    AesWriteIV(aes->iv_ce);

    /* set up pointers for input and output buffers */
    const uint32_t *inptr = (const uint32_t *)in;
    uint32_t *outptr = (uint32_t *)out;

    uint32_t block;   /* 16 bytes = 4 32bit block size */
    for (block = 0; block < sz; block += 16)
    {
        /* Write the data to be ciphered to the input data registers. */
        _AES_REGS->AES_IDATAR[0].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[1].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[2].IDATA = *inptr++;
        _AES_REGS->AES_IDATAR[3].IDATA = *inptr++;

        /* Note the blocking here - state machine this? */
        while (!(_AES_REGS->AES_ISR.DATRDY))  ;

        /* encrypt complete - read out the data */
        *outptr++ = _AES_REGS->AES_ODATAR[0].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[1].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[2].ODATA;
        *outptr++ = _AES_REGS->AES_ODATAR[3].ODATA;
    }
    
    return 0;
}


#endif /* WOLFSSL_AES_COUNTER */



#ifdef HAVE_AESCCM

#ifdef WOLFSSL_PIC32MZ_CRYPT
    #error "PIC32MZ doesn't currently support AES-CCM mode"

#endif
#endif /* HAVE_AESCCM */


#endif /* NO_AES */

