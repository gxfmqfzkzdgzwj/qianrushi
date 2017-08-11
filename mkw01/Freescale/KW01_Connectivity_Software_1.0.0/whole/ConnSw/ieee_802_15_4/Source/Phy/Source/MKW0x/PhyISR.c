/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyISR.c
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "EmbeddedTypes.h"
#include "FunctionLib.h"
#include "SX123xDrv.h"
#include "PhyInterface.h"
#include "Phy.h"
#include "PhyPib.h"
#include "PhyExtended.h"
#include "PhyISR.h"
#include "PhyTime.h"
//#include "Gpio_IrqAdapter.h"

#include "fsl_device_registers.h"
//#include "fsl_os_abstraction.h"
#include "portmacro.h"   
#ifdef gSmacSupported
#include "board.h"

#define SetContinuousDrivePin() (GPIO_PSOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) |= (1<<APP_DIO2_DATA_DRIVE_PIN))
#define ClearContinuousDrivePin() (GPIO_PCOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) |= (1<<APP_DIO2_DATA_DRIVE_PIN))
#define SetContinuousDrivePinValue(x) (                                            \
                                        GPIO_PDOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) =  \
                                        (GPIO_PDOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) & \
                                          (~(1 << APP_DIO2_DATA_DRIVE_PIN))) |     \
                                            (x << (APP_DIO2_DATA_DRIVE_PIN))       \
                                       )
#endif
/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define mPhySFDEventRxTimeout_c          40 /* MAC symbols */
#define mPhyPayloadReadyRxTimeout_c     100 /* MAC symbols */

#define ADDRMODE_TBL_SRC_DST_LEN(src, dst) ((dst) | ((src)<<4))

#define gPhySecHeaderMinimumLength_c    5

const uint8_t frmCtrlTable[16] = { ADDRMODE_TBL_SRC_DST_LEN( 0, 0),
		ADDRMODE_TBL_SRC_DST_LEN( 0, 0), ADDRMODE_TBL_SRC_DST_LEN( 4, 0),
		ADDRMODE_TBL_SRC_DST_LEN( 10, 0), ADDRMODE_TBL_SRC_DST_LEN( 0, 0),
		ADDRMODE_TBL_SRC_DST_LEN( 0, 0), ADDRMODE_TBL_SRC_DST_LEN( 0, 0),
		ADDRMODE_TBL_SRC_DST_LEN( 0, 0), ADDRMODE_TBL_SRC_DST_LEN( 0, 4),
		ADDRMODE_TBL_SRC_DST_LEN( 0, 0), ADDRMODE_TBL_SRC_DST_LEN( 4, 4),
		ADDRMODE_TBL_SRC_DST_LEN( 10, 4), ADDRMODE_TBL_SRC_DST_LEN( 0, 10),
		ADDRMODE_TBL_SRC_DST_LEN( 0, 0), ADDRMODE_TBL_SRC_DST_LEN( 4, 10),
		ADDRMODE_TBL_SRC_DST_LEN( 10, 10) };

/* A table to determine the length of the ASH given the KeyIdMode */
const uint8_t phyKeyIdModeToAshLengthTable[] = {gPhySecHeaderMinimumLength_c,      /* Security Control[1] + Frame Counter[4] = 5 */
                                                gPhySecHeaderMinimumLength_c + 1,  /* + KeyIndex[1] */
                                                gPhySecHeaderMinimumLength_c + 5,  /* + macPanId[2] + macShortAddress[2] + KeyIndex[1] */
                                                gPhySecHeaderMinimumLength_c + 9,  /* + macExtendedAddress[8] + KeyIndex[1] */
                                                };
/* LookUp Table for Data Whitening */
const uint8_t lookUpTable[gMaxPHYPacketSize_c] = {0xF0, 0x0E, 0xCD, 0xF6, 0xC2, 0x19, 0x12, 0x75,
                                                  0x3D, 0xE9, 0x1C, 0xB8, 0xCB, 0x2B, 0x05, 0xAA,
                                                  0xBE, 0x16, 0xEC, 0xB6, 0x06, 0xDD, 0xC7, 0xB3,
                                                  0xAC, 0x63, 0xD1, 0x5F, 0x1A, 0x65, 0x0C, 0x98,
                                                  0xA9, 0xC9, 0x6F, 0x49, 0xF6, 0xD3, 0x0A, 0x45,
                                                  0x6E, 0x7A, 0xC3, 0x2A, 0x27, 0x8C, 0x10, 0x20,
                                                  0x62, 0xE2, 0x6A, 0xE3, 0x48, 0xC5, 0xE6, 0xF3,
                                                  0x68, 0xA7, 0x04, 0x99, 0x8B, 0xEF, 0xC1, 0x7F,
                                                  0x78, 0x87, 0x66, 0x7B, 0xE1, 0x0C, 0x89, 0xBA,
                                                  0x9E, 0x74, 0x0E, 0xDC, 0xE5, 0x95, 0x02, 0x55,
                                                  0x5F, 0x0B, 0x76, 0x5B, 0x83, 0xEE, 0xE3, 0x59,
                                                  0xD6, 0xB1, 0xE8, 0x2F, 0x8D, 0x32, 0x06, 0xCC,
                                                  0xD4, 0xE4, 0xB7, 0x24, 0xFB, 0x69, 0x85, 0x22,
                                                  0x37, 0xBD, 0x61, 0x95, 0x13, 0x46, 0x08, 0x10,
                                                  0x31, 0x71, 0xB5, 0x71, 0xA4, 0x62, 0xF3, 0x79,
                                                  0xB4, 0x53, 0x82, 0xCC, 0xC5, 0xF7, 0xE0, 0x3F,
                                                  0xBC, 0x43, 0xB3, 0xBD, 0x70, 0x86, 0x44, 0x5D,
                                                  0x4F, 0x3A, 0x07, 0xEE, 0xF2, 0x4A, 0x81, 0xAA,
                                                  0xAF, 0x05, 0xBB, 0xAD, 0x41, 0xF7, 0xF1, 0x2C,
                                                  0xEB, 0x58, 0xF4, 0x97, 0x46, 0x19, 0x03, 0x66,
                                                  0x6A, 0xF2, 0x5B, 0x92, 0xFD, 0xB4, 0x42, 0x91,
                                                  0x9B, 0xDE, 0xB0, 0xCA, 0x09, 0x23, 0x04, 0x88,
                                                  0x98, 0xB8, 0xDA, 0x38, 0x52, 0xB1, 0xF9, 0x3C,
                                                  0xDA, 0x29, 0x41, 0xE6, 0xE2, 0x7B, 0xF0, 0x1F,
                                                  0xDE, 0xA1, 0xD9, 0x5E, 0x38, 0x43, 0xA2, 0xAE, 
                                                  0x27, 0x9D, 0x03, 0x77, 0x79, 0xA5, 0x40, 0xD5,
                                                  0xD7, 0x82, 0xDD, 0xD6, 0xA0, 0xFB, 0x78, 0x96,
                                                  0x75, 0x2C, 0xFA, 0x4B, 0xA3, 0x8C, 0x01, 0x33,
                                                  0x35, 0xF9, 0x2D, 0xC9, 0x7E, 0x5A, 0xA1, 0xC8,
                                                  0x4D, 0x6F, 0x58, 0xE5, 0x84, 0x11, 0x02, 0x44,
                                                  0x4C, 0x5C, 0x6D, 0x1C, 0xA9, 0xD8, 0x7C, 0x1E,
                                                  0xED, 0x94, 0x20, 0x73, 0xF1, 0x3D};
#ifdef gSmacSupported
#define gPrbs9BufferLength_c (65)
uint8_t 	u8Prbs9Buffer[gPrbs9BufferLength_c] = 
{
  0x42,
  0xff,0xc1,0xfb,0xe8,0x4c,0x90,0x72,0x8b,0xe7,0xb3,0x51,0x89,0x63,0xab,0x23,0x23,  
  0x02,0x84,0x18,0x72,0xaa,0x61,0x2f,0x3b,0x51,0xa8,0xe5,0x37,0x49,0xfb,0xc9,0xca,
  0x0c,0x18,0x53,0x2c,0xfd,0x45,0xe3,0x9a,0xe6,0xf1,0x5d,0xb0,0xb6,0x1b,0xb4,0xbe,
  0x2a,0x50,0xea,0xe9,0x0e,0x9c,0x4b,0x5e,0x57,0x24,0xcc,0xa1,0xb7,0x59,0xb8,0x87
};
uint8_t PN9BitIndex;
uint8_t PN9ByteIndex;
#endif

/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
static volatile phyRxParams_t *mpRxParams = NULL;
static volatile pdDataReq_t   *mpTxParams = NULL;

static phyTime_t    mStartClock;
static uint8_t      mPhyState = gIdle_c;
static uint8_t      mPhyLastRxLQI = 0;
static uint8_t      mPhyLastRxRssi = 0;
static uint8_t      mDstLen;
static uint8_t      mSrcLen;
static uint8_t      mFrameType;
static uint8_t      frameControlLow;
static uint8_t      frameControlHigh;
static uint8_t      mSeqNumber;
static bool_t       mAckWaitOngoing = FALSE;

static void Phy_SetRxOnUntilNextEvent(uint32_t duration);

/*****************************************************************************
 *                               PUBLIC VARIABLES                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have global      *
 * (project) scope.                                                          *
 * These variables / constants can be accessed outside this module.          *
 * These variables / constants shall be preceded by the 'extern' keyword in  *
 * the interface header.                                                     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
void (* gpfPendingRxEvent)(void);
void (* gpfPendingTxEvent)(void);

extern void (*gpfPhyTimeNotify)(void);

extern Phy_PhyLocalStruct_t phyLocal;
extern uint8_t              gTurnaroundTimeTable[];
extern uint8_t              rxData[];
extern pdDataReq_t          ackDataReq;
extern uint8_t              ackBuffer[3];
#if gSnifferCRCEnabled_d
extern bool_t               crcValid;
#endif

/*****************************************************************************
 *                             PUBLIC FUNCTIONS                              *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyIsrPassRxParams()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyIsrPassRxParams(volatile phyRxParams_t * pRxParam)
{  
  mpRxParams = pRxParam; 
}

/*---------------------------------------------------------------------------
 * Name: PhyPassTxParams()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyPassTxParams(pdDataReq_t *pTxParam)
{
  uint8_t fifoThreshold = gFifoThreshold_c;

  mpTxParams = pTxParam;
  
  /* PHY internal Tx params */
  mpTxParams->macDataIndex = 0;    
  mpTxParams->phyHeader.frameLength = pTxParam->psduLength + 2; // ADD FCS Length
  mpTxParams->phyHeader.fcsType = 0x01;                         // FCS Length 2 bytes    
  mpTxParams->fillFifoBlockLength = gFillFifoBlockLength_c;
  
  // Save sequence number to be checked with the one received in ACK
  if( pTxParam->ackRequired != gPhyNoAckRqd_c )
  {
    mSeqNumber = mpTxParams->pPsdu[2];
    mAckWaitOngoing = TRUE;
  }
  
  if (mpTxParams->phyHeader.frameLength < gFillFifoBlockLength_c)
  {
    mpTxParams->fillFifoBlockLength = mpTxParams->phyHeader.frameLength;
    fifoThreshold = mpTxParams->phyHeader.frameLength - 1;
  }
  
  return fifoThreshold;
}

/*****************************************************************************
 *                                PRIVATE FUNCTIONS                          *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyTxHandleDummyEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxHandleDummyEvent(void)
{
  ;  
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_FillFifo()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_FillFifo(void)
{
  uint8_t   tempBuffer[gFillFifoBlockLength_c];
  uint32_t  blockIndex;
  uint8_t   blockLength = mpTxParams->fillFifoBlockLength;
  uint8_t   tempIndex;
  uint8_t  *blockPtr;
  
  tempIndex = mpTxParams->macDataIndex;
  blockPtr = (uint8_t *)mpTxParams->pPsdu + tempIndex;
    
  if(mpTxParams->phyHeader.frameLength <= gFillFifoBlockLength_c)
  {
    blockLength = mpTxParams->phyHeader.frameLength;
  }

  for(blockIndex = 0; blockIndex < blockLength; blockIndex++)
  {
    Phy_CrcAddByte(*blockPtr);
    if (gPhyPib.mPIBphyFSKScramblePSDU == TRUE)
    {
      /* Whitening TX data */
      tempBuffer[blockIndex] = *blockPtr ^ lookUpTable[mpTxParams->macDataIndex++];
    }
    else
    {
      tempBuffer[blockIndex] = *blockPtr;
    }
    
    blockPtr++;
  }
  
  XCVRDrv_WriteBytesToFifoLSB(tempBuffer, blockLength);
 
  mpTxParams->phyHeader.frameLength -= blockLength;
  
  if(mpTxParams->phyHeader.frameLength == 0)    
  {
    blockPtr = (uint8_t *)&phyLocal.fcs;
   
    for(blockIndex = 0; blockIndex < 2; blockIndex++)
    {
      if (gPhyPib.mPIBphyFSKScramblePSDU == TRUE)
      {
        /* Whitening TX data */
        tempBuffer[blockIndex] = *blockPtr ^ lookUpTable[mpTxParams->macDataIndex++];
      }
      else
      {
        tempBuffer[blockIndex] = *blockPtr;
      }
      
      blockPtr++;
    }
  
    XCVRDrv_WriteBytesToFifoLSB(tempBuffer, 2);
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyTxPacketSentEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxPacketSentEvent(void)
{    
  XCVRDrv_IRQ_DIO0_Disable(); 
     
  gpfPendingTxEvent = PhyTxHandleDummyEvent; 
  
  PhySetState(gIdle_c);
  
  Phy_SetOperationModeFast(OpMode_StandBy);      
  
  // If ACK or Enhanced ACK required enter RX
  if ((phyLocal.flags.rxAckRqd) || 
      (phyLocal.flags.rxEnhAckRqd))    
  {
    PhyTimeDisableEventTimeout();  
   
    if( phyLocal.flags.tschEnabled )
    {
        PhyTimeReadClockTicks(&mStartClock);
        mStartClock += TIME_US_TO_TICKS(gPhyPib.mPIBphyTsRxAckDelayUs);
        phyLocal.startTime = mStartClock;
        mStartClock -= Phy_XCVRRxWarmupTime();
        PhyTimeSetEventTrigger(mStartClock);
        
        mStartClock = phyLocal.startTime + 
                      TIME_US_TO_TICKS(gPhyPib.mPIBphyTsAckWaitUs);
        PhyTimeSetEventTimeout(&mStartClock);
    }
    else
    {
        // Start Rx with 3 symbols earlier to make sure no ACK is missed
        PhyTimeReadClock(&mStartClock);
        mStartClock += gTurnaroundTimeTable[gPhyPib.mPIBphyCurrentMode] - 3;  
        
        if( phyLocal.flags.rxAckRqd )
        {
            Phy_SetSequenceTiming(mStartClock, 
                                  gPhyPib.mPIBphyAckWaitDuration, 
                                  gRX_c, 
                                  0);
        }
        else
        {
            Phy_SetSequenceTiming(mStartClock, 
                                  gPhyPib.mPIBphyAckWaitDuration + (50 * gPhySymbolsPerOctet_c), 
                                  gRX_c, 
                                  0);
        }    
    }      

    (void)PhyPlmeRxRequest(gPhyUnslottedMode_c, (phyRxParams_t *) &phyLocal.rxParams);              
    
    phyLocal.flags.rxAckRqd = FALSE;
    phyLocal.flags.rxEnhAckRqd = FALSE; 
  }
  else
  {
    if (phyLocal.flags.autoAck)
    {      
      phyLocal.flags.autoAck = FALSE;
      Radio_Phy_PdDataIndication(0);
      
      /* Check if there was an ACK wait ongoing */
      if( mAckWaitOngoing )
      {
        mAckWaitOngoing = FALSE;
        Radio_Phy_TimeRxTimeoutIndication(0);
      }
    }
    else        
    {
      Radio_Phy_PdDataConfirm(0, FALSE);       
    }
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyTxHandleFifoLevelEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxHandleFifoLevelEvent(void)
{  
  if(mpTxParams->phyHeader.frameLength > 0)
  {
    XCVRDrv_FillFifo();
  }    
  else
  {        
    gpfPendingTxEvent = PhyTxPacketSentEvent;     
    XCVRDrv_IRQ_DIO1_Disable();
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleDummyEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleDummyEvent(void)
{
  ;
}

/*---------------------------------------------------------------------------
 * Name: PhyRxHandlePayloadReadyEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandlePayloadReadyEvent(void)
{
  uint64_t    tempAddress; 
  uint32_t    i;
  uint16_t    tempPanId;
  uint16_t    tempChecksum;
  phyStatus_t status;
  uint8_t     tempAuxSecOffsetId;

  gPhyPib.mPIBTxAckFP = 0;
  
  /* Clear XCVR Fifo, should any extra bytes be received */
  Phy_ClearFifo();

  /* CRC check */
  if( ( rxData[mpRxParams->macDataIndex-2] != (phyLocal.fcs & 0xFF) ) ||
      ( rxData[mpRxParams->macDataIndex-1] != (phyLocal.fcs >> 8) ) )
  {
#if gSnifferCRCEnabled_d    
    crcValid = FALSE;
#else
    Phy_RxHandleFilterFail();
    return;
#endif    
  }
#if gSnifferCRCEnabled_d
  else
  {
    crcValid = TRUE;
  }
#endif
  
  if( mFrameType == gFrameTypeAck_c ) 
  {
    mAckWaitOngoing = FALSE;
    
    if(((frameControlHigh & gFrameVersionMask_c) >> gFrameVersionShift_c) == gFrameVersion4ge_c)
    {
      Radio_Phy_PdDataIndication(0);
      return;
    }
    
    if( frameControlLow & gFramePendingFlag_c )
    {
      Radio_Phy_PdDataConfirm(0, TRUE);      
    }
    else
    {
      Radio_Phy_PdDataConfirm(0, FALSE);      
    }        
  }
  else      
  {        
    if( mFrameType == gFrameTypeCommand_c )  // Check Frame Type for MAC Command
    {
      if( frameControlLow & gSecurityEnableFlag_c )
      {
        tempAuxSecOffsetId = (rxData[mpRxParams->headerLength] & 0x18) >> 3;
        mpRxParams->headerLength += phyKeyIdModeToAshLengthTable[tempAuxSecOffsetId];
      }
      
      if( rxData[mpRxParams->headerLength] == 0x04 )   // Poll Command ID
      {            
        if( frameControlLow & gIntraPanFlag_c )
        {
          FLib_MemCpy( &tempAddress, &rxData[gAddrFieldsStartPos_c + mDstLen], mSrcLen);
          FLib_MemCpy( &tempPanId, &rxData[gAddrFieldsStartPos_c], sizeof(uint16_t) );
        }
        else
        {
          FLib_MemCpy( &tempAddress, &rxData[gAddrFieldsStartPos_c + mDstLen + 2], mSrcLen - 2 );
          FLib_MemCpy( &tempPanId, &rxData[gAddrFieldsStartPos_c + mDstLen], sizeof(uint16_t) );
        }
        
        tempChecksum = Phy_IndirectQueueChecksum((mSrcLen < 8), tempAddress, tempPanId);
        
        for (i=0; i < gPhyIndirectQueueSize_c; i++)
        {            
          if (( tempChecksum == phyLocal.phyIndirectQueue[i] ) && 
              ( phyLocal.phyUnavailableQueuePos & (1 << i)) )
          {
            gPhyPib.mPIBTxAckFP = 1;
            break;
          }
        }
      }
    }           
    
    // Send AUTO ACK only if CSL/TSCH is not activated. 
    if( (frameControlLow & gAckReqFlag_c) && 
        (FALSE == phyLocal.flags.cslRxEnabled) &&
        (FALSE == phyLocal.flags.tschEnabled) )
    {
      PhyTimeDisableEventTimeout();      
      
      // Generate ACK packet
      ackBuffer[0] = gFrameTypeAck_c | (gPhyPib.mPIBTxAckFP << 4);
      ackBuffer[1] = 0;
      ackBuffer[2] = rxData[gSeqNumberPos_c];
      
      phyLocal.flags.autoAck = TRUE;
      
      PhyTimeReadClock(&mStartClock);
      mStartClock += gTurnaroundTimeTable[gPhyPib.mPIBphyCurrentMode]; 
      Phy_SetSequenceTiming(mStartClock, 
                            Phy_TxLength(3 + 4), // mAckFrame.psduLength + 4 bytes for FCS and PHR
                            gTX_c,
                            0);      
      
      status = PhyPdDataRequest(&ackDataReq, 
                                &phyLocal.rxParams, 
                                &phyLocal.txParams);
      
      if( gPhySuccess_c != status )
      {        
        PhyTimeDisableEventTrigger();
        PhyTimeDisableEventTimeout();
      }
    }
    else
    {
      Radio_Phy_PdDataIndication(0);
      
      /* Check if there was an ACK wait ongoing */
      if( mAckWaitOngoing )
      {
        mAckWaitOngoing = FALSE;
        Radio_Phy_TimeRxTimeoutIndication(0);
      }
    }
  } 
}

/*---------------------------------------------------------------------------
 * Name: Phy_RxFrameFilter
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RxFrameFilter(void)
{
    uint8_t dstsrc, dst, src;
    uint8_t macDataIndex = mpRxParams->macDataIndex;
    
    // FrameControl LSB received, check frame types
    if( macDataIndex == 2 )
    {
        frameControlLow = rxData[gFrameControlLsbPos_c];
        mFrameType = (uint8_t) ((frameControlLow >> gFrameTypeShift_c) & gFrameTypeMask_c);

        /* Check for multipurpose wake-up frame */
        if( mFrameType == gFrameTypeMultipurpose_c )
        {
            /* Short frame control */
            if( !(frameControlLow & gMppLongFrameControlMask_c) )
            {
                /* Drop multipurpose frame with any other addressing mode than short */
                if( gAddrMode16BitAddr_c != ((frameControlLow & gMppFrameDstAddrModeMask_c ) >> gMppDstAddrModeShift_c ) )
                {
                    Phy_RxHandleFilterFail();
            		return;
                }
                else
                {
                    mDstLen = 4;
                    mSrcLen = 0;

                    /* 1 fc, 1 seq no, 4 dst */
                    mpRxParams->headerLength = 6;
                }
            }
            else
            {
                /* Drop multipurpose frame with long frame control format */
                Phy_RxHandleFilterFail();
                return;
            }
        }
        else      
        {            
            frameControlHigh = rxData[gFrameControlMsbPos_c];                                    

            // Check if reserved frame type
            if((mFrameType != gFrameTypeBeacon_c) && (mFrameType != gFrameTypeData_c) && 
               (mFrameType != gFrameTypeAck_c) && (mFrameType != gFrameTypeCommand_c)) 
            {
                Phy_RxHandleFilterFail();
                return;
            }         

            if ((mFrameType == gFrameTypeAck_c) && 
                !(frameControlHigh & (gFrameVersion4ge_c << gFrameVersionShift_c)))
            {          
                // Check if ACK frame length != 5
                if(mpRxParams->psduLength != gMacMinHeaderLengthAck_c)
                {
                    Phy_RxHandleFilterFail();
                    return;
                }
          
                mpRxParams->headerLength = gMacMinHeaderLengthAck_c - 2; 
                return;
            }         

            //If security enabled with frame version 2003 drop the packet
            if(frameControlLow & gSecurityEnableFlag_c)
            {
                if(!(frameControlHigh & gFrameVersionMask_c))
                {
                    Phy_RxHandleFilterFail();
                    return;
                }
            }

            //drop packets with frame version set on reserved
            if(((frameControlHigh & gFrameVersionMask_c) >> gFrameVersionShift_c) == 0x30) 
            {
                Phy_RxHandleFilterFail();
                return;
            }

            // Calculate source and destionation field lengths
            dst = rxData[gFrameControlMsbPos_c] & 0x0C;  // dest addr mode
            src = (rxData[gFrameControlMsbPos_c] >> 6) & 0x03;  // src addr mode
            dstsrc = (dst | src);   // dstsrc is a4 bit variable organised with dst in 2MSBs and src in 2LSBs
            // Decode addr mode and set dest and src length accordingly. Abort on illegal frame types.

            mDstLen = frmCtrlTable[dstsrc] & 0x0F;
            mSrcLen = frmCtrlTable[dstsrc] >> 4;

            if(mDstLen == 0 && mSrcLen == 0)
            {
                Phy_RxHandleFilterFail();
                return;
            }
            if((mDstLen != 0 && mSrcLen == 0) && (frameControlLow & gIntraPanFlag_c))
            {
                Phy_RxHandleFilterFail();        
                return;
            }                  
            if((mDstLen != 0 && mSrcLen != 0) && (frameControlLow & gIntraPanFlag_c))
            {
                mSrcLen -= 2;
            }

            mpRxParams->headerLength = gAddrFieldsStartPos_c + mDstLen + mSrcLen;   
        }
    } 
    else if( macDataIndex == mpRxParams->headerLength )
    {
        // if the addressing field was received
        if (mFrameType == gFrameTypeAck_c)
        {
            if( (!mAckWaitOngoing) || (mSeqNumber != rxData[gSeqNumberPos_c]) )
            {
                Phy_RxHandleFilterFail();         
                return;                
            }                             
        }
        else 
        {
            if(mDstLen != 0)
            {
                Phy_IsrRxFilterDest();
            }
        }
    } 
}

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleFifoLevelEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleFifoLevelEvent(void)
{
    uint8_t readData[gRxFifoBlockLength_c];
    uint8_t macDataIndex;
    uint8_t blockLen = mpRxParams->fifoBlockLen;
    
    mpRxParams->phyHeader.frameLength -= blockLen;
    
    /* Check if these bytes were the last part of the frame */
    if( 0 == mpRxParams->phyHeader.frameLength )
    {      
#ifdef gListenSupported_c
        if(phyLocal.flags.rxIsListen)
        {
          Phy_SetOperationModeFast((OpMode_StandBy | OpMode_Listen_Abort));
        }
#endif
        /* Set transceiver in standby */
        Phy_SetOperationModeFast(OpMode_StandBy);

        /* Disable FifoLevel */
        XCVRDrv_IRQ_DIO1_Disable();

        PhyTimeDisableRxTimeout();
        PhySetState(gIdle_c);
        gpfPendingRxEvent = PhyRxHandleDummyEvent;
    }
    
    XCVRDrv_ReadBytesFromFifoLSB(readData, mpRxParams->fifoBlockLen);
    
    /* Get mpRxParams->fifoBlockLen bytes from FIFO */
    for( uint32_t i=0; i<mpRxParams->fifoBlockLen; i++ )
    {    
        /* Dewhitening received data */
        if( mpRxParams->phyHeader.dataWhitening )
        {
            readData[i] = readData[i] ^ lookUpTable[mpRxParams->macDataIndex];
        }
    
        rxData[mpRxParams->macDataIndex++] = readData[i];
    
        macDataIndex = mpRxParams->macDataIndex;
        
        if( macDataIndex < (mpRxParams->psduLength - 1) )
        {
            Phy_CrcAddByte(readData[i]);
        }
        
        /* Filter frame if FC or MAC header was received */
        if( !phyLocal.flags.promiscuous )
        { 
            if( ( macDataIndex == 2 ) ||
                ( macDataIndex == mpRxParams->headerLength) )
            {
                Phy_RxFrameFilter();
                
                if( phyLocal.flags.filterFail )
                {
                    phyLocal.flags.filterFail = 0;
                    XCVRDrv_IRQ_DIO1_Clear();
                    return;
                }
            }
        }
    }
    
    /* Clear FifoLevel after reading from FIFO */  
    XCVRDrv_IRQ_DIO1_Clear();

    /* If there are more bytes to receive */
    if( mpRxParams->phyHeader.frameLength )
    {
        /* Compute next block size */                  
        if( (gRxFifoBlockLength_c << 1) <= mpRxParams->phyHeader.frameLength )
        {
            mpRxParams->fifoBlockLen = gRxFifoBlockLength_c;
        }
        else
        {
            mpRxParams->fifoBlockLen = mpRxParams->phyHeader.frameLength;
        }

        /* Set FIFO threshold to the next of the remaining payload */
        Phy_SetFifoThreshold(mpRxParams->fifoBlockLen - 1);    
    }
    else
    {        
        /* All frame received, continue processing */
        PhyRxHandlePayloadReadyEvent();
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyRxHandlePHREvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandlePHREvent(void)
{   
    /* Disable Timeout set at sync address */
    PhyTimeDisableRxTimeout();

    XCVRDrv_ReadBytesFromFifoLSB((uint8_t*)&mpRxParams->phyHeader.byteAccess[0], 1);

    /* Frame length is sent MSB first */
    mpRxParams->phyHeader.byteAccess[1] = XCVRDrv_ReadFifo();
    
    /* Clear FifoLevel after reading from FIFO */
    XCVRDrv_IRQ_DIO1_Clear();

    mpRxParams->psduLength = (uint16_t)mpRxParams->phyHeader.frameLength;
          
    /* Filter packet length */
    if((mpRxParams->psduLength > gMaxPHYPacketSize_c) || 
       (mpRxParams->psduLength < gMinPHYPacketSize_c))
    {
        Phy_RxHandleFilterFail();   
        return;
    }
        
    gpfPendingRxEvent = PhyRxHandleFifoLevelEvent;
    
    /* Check to set FIFO threshold at 4, so that minimum packet size of 5 can be received */                
    if( gRxFifoBlockLength_c <= mpRxParams->phyHeader.frameLength )
    {
        mpRxParams->fifoBlockLen = gRxFifoBlockLength_c;
    }
    else
    {
        mpRxParams->fifoBlockLen = mpRxParams->phyHeader.frameLength;
    }
    
    Phy_SetFifoThreshold(mpRxParams->fifoBlockLen - 1);

    /* Set Rx timeout for actual packet length */
    Phy_SetRxOnUntilNextEvent(Phy_RxLength(mpRxParams->psduLength) + mPhyPayloadReadyRxTimeout_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleSyncAddresEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleSyncAddresEvent(void)
{
    // Disable timeout set at RxReady
    PhyTimeDisableRxTimeout();

    /* Disable SyncAddress */
    XCVRDrv_IRQ_DIO0_Disable();

    /* Timestamp will be saved now in us and converted to symbols when passed to MAC */
    PhyTimeReadClockTicks((phyTime_t*)&mpRxParams->timeStamp);

    /* RSSI read now and converted to LQI when passed to MAC */
    mpRxParams->linkQuality = Phy_GetRssi();

    gpfPendingRxEvent = PhyRxHandlePHREvent;

    /* Initialize RX params */
    mpRxParams->macDataIndex = 0;
    phyLocal.fcs = 0;

    /* Set timeout if the reception fails */
    Phy_SetRxOnUntilNextEvent(Phy_RxLength(gMaxPHYPacketSize_c + gPhyMRFSKPHRLength_c));
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetRxOnTimeout
 * Description: The function is used for Rx timeout for RxReady/SFD/PHR events
 * Parameters: - duration in MAC symbols
 * Return: -
 *---------------------------------------------------------------------------*/
static void Phy_SetRxOnUntilNextEvent(uint32_t duration)
{
  phyTime_t eventTime;
  phyTime_t durationTicks = duration;
  
  /* Convert from MAC symbols to ticks */
  Phy_TimeMultiplicator(&durationTicks);  
  PhyTimeReadClockTicks(&eventTime);
  eventTime += durationTicks;

  /* Set Rx timeout should next event not occur */
  PhyTimeSetRxTimeout(&eventTime);
      
  /* Check to extend Rx On duration set in event timeout */
  if( PhyTimeGetEventTimeout() < eventTime ) 
  {
    PhyTimeSetEventTimeout(&eventTime);
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventTimeout()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventTimeout(void)
{ 
#ifdef gListenSupported_c
  if(phyLocal.flags.rxIsListen)
  {
    Phy_SetOperationModeFast((OpMode_StandBy | OpMode_Listen_Abort));
  }
#endif
  Phy_SetOperationModeFast(OpMode_StandBy);    
  
  switch(mPhyState)
  {        
    case gTX_c:    
      gpfPendingTxEvent = PhyTxHandleDummyEvent;
      XCVRDrv_IRQ_DIO0_Disable();
      XCVRDrv_IRQ_DIO1_Disable();
      break;    
    case gRX_c:      
      gpfPendingRxEvent = PhyRxHandleDummyEvent;
      XCVRDrv_IRQ_DIO0_Disable();      
      XCVRDrv_IRQ_DIO1_Disable();
      XCVRDrv_IRQ_DIO4_Disable();
      Radio_Phy_TimeRxTimeoutIndication(0);
      mAckWaitOngoing = FALSE;
      break;  
  }
  
  PhySetState(gIdle_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventTrigger()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventTrigger(void)
{
    switch(mPhyState)
    {
    case gTX_c:
        Phy_SetOperationModeFast(OpMode_Transmitter);
        break;    
    case gCCA_c:
    case gED_c:  
        Phy_SetOperationModeFast(OpMode_Receiver);
        break;
    case gRX_c:              
#ifdef gListenSupported_c
        if(phyLocal.flags.rxIsListen)
        {
          Phy_SetOperationModeFast((OpMode_StandBy | OpMode_Listen_On)); 
        }
        else
#endif
        {
          Phy_SetOperationModeFast(OpMode_Receiver);     
        }
        break;   
    default:    
        break;    
    }  
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventWaitTimeout()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventWaitTimeout(void)
{  
  if( gpfPhyTimeNotify )
  {
    gpfPhyTimeNotify();
  } 
}

/*---------------------------------------------------------------------------
 * Name: Phy_RxRestart()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RxRestart(void)
{
  PhyPlmeForceTrxOffRequest();
    
  if( phyLocal.flags.idleRx )
  {
    Radio_Phy_DummyEvent(0);
  }
  else
  {
    phyLocal.startTime = gPhySeqStartAsap_c;
    (void)PhyPlmeRxRequest(gPhyUnslottedMode_c, (phyRxParams_t *)&phyLocal.rxParams);
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventRxTimeout()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventRxTimeout(void)
{
  if( mPhyState == gRX_c )    
  {
    Phy_RxRestart();
  }
  else
  {
    phyLocal.flags.ccaComplete = TRUE;
  }
}

/*---------------------------------------------------------------------------
 * Name: Phy_ISR()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PORTC_PORTD_IRQHandler()
{  
  if( XCVRDrv_IRQ_DIO0_Detected() || XCVRDrv_IRQ_DIO1_Detected() )
  {    
    XCVRHandlerInterrupt();   
  }
  else if( XCVRDrv_IRQ_DIO4_Detected())
  {
    CCAPinHandler_Interrupt();
  }
}

/*---------------------------------------------------------------------------
 * Name: XCVRHandlerInterrupt()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRHandlerInterrupt(void)
{
  XCVRDrv_IRQ_DIO0_Clear();
  XCVRDrv_IRQ_DIO1_Clear();

  switch(mPhyState)
  {
    case gTX_c:
      gpfPendingTxEvent();
      break;    
    case gRX_c:    
      gpfPendingRxEvent();
      break;
#ifdef gSmacSupported
    case gPN9_c:
      SetContinuousDrivePinValue( ((u8Prbs9Buffer[PN9ByteIndex] >> PN9BitIndex) & 0x01) );
      PN9ByteIndex = (PN9ByteIndex + (((PN9BitIndex + 1) & 0x08) >> 3)) & (gPrbs9BufferLength_c - 2);
      PN9BitIndex = (PN9BitIndex + 1) & 0x07;
      break;
#endif
    default:
       break;
  }
}

/*---------------------------------------------------------------------------
 * Name: CCAPinHandler_Interrupt()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void CCAPinHandler_Interrupt(void)
{
  uint32_t rssiAcc = 0;
  uint16_t ccaMeasurementsCnt = 0;
  uint8_t  energyLeveldB = 0;
  
  XCVRDrv_IRQ_DIO4_Clear();
  XCVRDrv_IRQ_DIO4_Disable();
  
  if( (mPhyState == gCCA_c) || 
      (mPhyState == gED_c) )
  {     
    do
    {
      rssiAcc += Phy_GetRssi();
      ccaMeasurementsCnt++;
    } while( phyLocal.flags.ccaComplete != TRUE );   
    
    energyLeveldB = (uint8_t)(rssiAcc / ccaMeasurementsCnt);       
    
    PhySetState(gIdle_c);
    //Put transceiver in Stand By mode
    Phy_SetOperationModeFast(OpMode_StandBy);
    
    energyLeveldB = (energyLeveldB >> 1);  // [dBm] 
#if CT_Feature_Calibration
    //this means there was an overflow
    if(energyLeveldB + (int8_t)(gPhyPib.mPIBAdditionalEDOffset) < 128)
    {
      energyLeveldB += (int8_t)(gPhyPib.mPIBAdditionalEDOffset);
    }
    
#endif
    if(phyLocal.channelParams.ccaParam == gCcaCCA_MODE1_c)
    {       
      if(energyLeveldB > gPhyPib.pPIBphyRfConstants->ccaThreshold)
      {
        if (phyLocal.flags.ccaBfrTX == TRUE)    // Channel IDLE and CCA before TX. Send Data request
        {
          phyLocal.flags.ccaBfrTX = FALSE;
          
          if( phyLocal.flags.tschEnabled )
          {
            mStartClock = phyLocal.startTime + 
                          TIME_US_TO_TICKS(gPhyPib.mPIBphyTsCcaUs) +
                          TIME_US_TO_TICKS(gPhyPib.mPIBphyTsRxTxUs);
          }
          else
          {
            PhyTimeReadClockTicks(&mStartClock);
            mStartClock += 1500; /* aTurnaroundTime is constant - 1000 us */          
          }
          
          mStartClock -= Phy_XCVRTxWarmupTime();
          PhyTimeSetEventTrigger(mStartClock);
          
          //CCA is complete, Tx can be prepared
          PhyPrepareTx((pdDataReq_t*)mpTxParams);
        }
				
        else
        {
          Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, 0);
        }
      }
      else
      {
        Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, 0);
      }
    }
    else
    { 
      Radio_Phy_PlmeEdConfirm(energyLeveldB, 0);      
    }
        
    /* Restore AFC state disabled at CCA/ED start */
    if( gPhyPib.mPIBAfcEnabled )
    {
      Phy_EnableAfcAuto(TRUE);
    }   
  }
  else if (mPhyState == gRX_c) 
  {        
    /* Enable transceiver interrupts for Sync Address and FifoLevel */
    XCVRDrv_IRQ_DIO0_Enable(0x09); /* SyncAddress rising */
    XCVRDrv_IRQ_DIO1_Enable(0x0C); /* FifoLevel logic 1 */
    
    /* Set timeout if SyncAddress event is not received */
    Phy_SetRxOnUntilNextEvent(gAfcRxTimeout_c + mPhySFDEventRxTimeout_c);
  }
}

/*---------------------------------------------------------------------------
 * Name: Phy_GetEnergyLevel
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_GetEnergyLevel(uint8_t energyLevel)
{
#define edThreshold (gPhyPib.pPIBphyRfConstants->ccaThreshold) 
    /* Compare level with receiver reference sensitivity */
    if( energyLevel >= edThreshold )
    {
        energyLevel = 0x00;
    }
    /* Compare level with 64 dbM over the reference sensitivity */
    else if( energyLevel <= (edThreshold - 64) )
    {
        energyLevel = 0xFF;
    }
    else
    {
        energyLevel = ( edThreshold - energyLevel );
        
        /* Expand the 64 possible values to 0x00 - 0xFF range */
        energyLevel <<= 2;
    }
    
    return energyLevel;
}

uint8_t Phy_LqiConvert(uint8_t rssiValue)
{  
  mPhyLastRxRssi = rssiValue;
  
  /* LQI min RSSI level when AGC is enabled is -115 dBm */
  if( rssiValue >= 90 )
  {
    rssiValue = 0x00;
  }
  else if( rssiValue <= 26 )
  {
    rssiValue = 0xFF;
  }
  else
  {
    rssiValue = (90 - rssiValue);

    /* Expand the 64 possible values to 0x00 - 0xFF range */
    rssiValue <<= 2;
  }
  
  mPhyLastRxLQI = rssiValue;
  
  return rssiValue;
}

/*---------------------------------------------------------------------------
 * Name: PhyGetLastRxLqiValue
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyGetLastRxLqiValue(void)
{
  return mPhyLastRxLQI;
}

/*---------------------------------------------------------------------------
 * Name: PhyGetLastRxRssiValue
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyGetLastRxRssiValue(void)
{
  return mPhyLastRxRssi;
}

/*---------------------------------------------------------------------------
 * Name: PHY_InstallIsr()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
/*void PHY_InstallIsr(void)
{   
    GpioInstallIsr(Phy_ISR, gGpioIsrPrioHigh_c, gPhy_IRQ_Priority_c, kGpioXcvrDio0);
    GpioInstallIsr(Phy_ISR, gGpioIsrPrioHigh_c, gPhy_IRQ_Priority_c, kGpioXcvrDio1);
    GpioInstallIsr(Phy_ISR, gGpioIsrPrioHigh_c, gPhy_IRQ_Priority_c, kGpioXcvrDio4);
}*/

/*---------------------------------------------------------------------------
 * Name: PhyGetState
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyPpGetState(void)
{
  uint8_t phyState;

  portENTER_CRITICAL();
  phyState = mPhyState;
  portEXIT_CRITICAL();
  
  return phyState;
}

/*---------------------------------------------------------------------------
 * Name: PhySetState()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhySetState(uint8_t phyState)
{ 
  portENTER_CRITICAL();
  mPhyState = phyState;
 portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: Phy_RxHandleFilterFail
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RxHandleFilterFail(void)
{  
  phyLocal.flags.filterFail = 1;
  Phy_RxRestart(); 
}
          
/*---------------------------------------------------------------------------
 * Name: Phy_IsrRxFilterDest
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_IsrRxFilterDest(void) 
{	
  uint8_t addrField = gAddrFieldsStartPos_c;
  
  /* Check for multipurpose wake-up frame */
  if( mFrameType == gFrameTypeMultipurpose_c )
  {
    /* Short frame control and sequence number present */
    //addrField = 2;
    /* Per standard, wake-up frame should be received with any dest addr */
    return;
  }
  else if( (((frameControlHigh & gFrameVersionMask_c) >> gFrameVersionShift_c) == gFrameVersion4ge_c) )
  {
    /* Check for seq number suppression, if frame is 4g/e */
    if( frameControlHigh & gSNSuppresionFlag_c )
    {
      addrField = gAddrFieldsStartPos_c - 1;
    }
  }
  
  /* Check own PAN ID */
  if ((phyLocal.macPanID[0] != rxData[addrField]) || 
      (phyLocal.macPanID[1] != rxData[addrField + 1])) 
  {
    /* Check for broadcast PAN ID */
    if ((0xFF != rxData[addrField]) || 
        (0xFF != rxData[addrField + 1])) 
    {
      Phy_RxHandleFilterFail();      
      return;
    }
  }

  /* Check destination for no, short or extended address */
  if( mDstLen == 0 )
  {
    if( gFrameTypeBeacon_c != mFrameType )
    {
      /* Check source PAN ID if PANC */
      if( !phyLocal.flags.panCordntr || 
          ((phyLocal.macPanID[0] != rxData[addrField ]) || 
           (phyLocal.macPanID[1] != rxData[addrField + 1])) ) 
      {
        Phy_RxHandleFilterFail();				
        return;
      }    
    }
  }
  else if( mDstLen == 4 )
  {
    /* Check own short address */
	if( (phyLocal.macShortAddress[0] != rxData[addrField + gPanIdLength_c]) || 
        (phyLocal.macShortAddress[1] != rxData[addrField + gPanIdLength_c + 1]) ) 
    {
      /* Check broadcast address */
	  if( (0xFF != rxData[addrField + gPanIdLength_c]) || 
          (0xFF != rxData[addrField + gPanIdLength_c + 1]) )
      {
        Phy_RxHandleFilterFail();
        return;
	  }
      /* Drop broadcast frames with ACK request flag set, 
         except short frame control of wakeup frames */
      else if( ( frameControlLow & gAckReqFlag_c ) &&
               ( mFrameType != gFrameTypeMultipurpose_c ) )
      {
        Phy_RxHandleFilterFail();
        return;
      }
	}
  } 
  else 
  {
    /* Check own extended address */
    if( FALSE == FLib_MemCmp(phyLocal.macLongAddress, 
                             &rxData[addrField + gPanIdLength_c],
                             8) )
    {
      Phy_RxHandleFilterFail();
      return;
    }
  }
}   
      
/*---------------------------------------------------------------------------
 * Name: Phy_CrcAddByte
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_CrcAddByte(uint8_t data)
{
#define crc (phyLocal.fcs)
  crc = crc ^ data;

  for( uint32_t i=0; i<8; i++ )    
  {     
    if( crc & 0x0001 )
    {      
	  crc = (crc >> 1) ^ 0x8408;
    }
	else
    {
	  crc = crc >> 1;
    }
  }      
}

/*---------------------------------------------------------------------------
 * Name: Phy_XCVRTxWarmupTime
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_XCVRTxWarmupTime(void)
{
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        return gTs_Tr_50_c;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        return gTs_Tr_50_c;
    case gPhyMode2_c:
        return gTs_Tr_100_c;        
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        return gTs_Tr_50_c;
    case gPhyMode2_c:
        return gTs_Tr_150_c;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        return gTs_Tr_50_c;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        return gTs_Tr_100_c;       
    case gPhyMode3ARIB_c:
        return gTs_Tr_200_c;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        return gTs_Tr_1_c;
    case gPhyMode2_c:
        return gTs_Tr_20_c;
    case gPhyMode3_c:
        return gTs_Tr_100_c;
#endif    
    default:    
        return 0;     
    }
}

/*---------------------------------------------------------------------------
 * Name: Phy_XCVRRxWarmupTime
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_XCVRRxWarmupTime(void)
{
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        return gTs_Re_50_c;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        return gTs_Re_50_c;
    case gPhyMode2_c:
        return gTs_Re_100_c;        
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        return gTs_Re_50_c;
    case gPhyMode2_c:
        return gTs_Re_150_c;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        return gTs_Re_50_c;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        return gTs_Re_100_c;       
    case gPhyMode3ARIB_c:
        return gTs_Re_200_c;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        return gTs_Re_1_c;
    case gPhyMode2_c:
        return gTs_Re_20_c;
    case gPhyMode3_c:
        return gTs_Re_100_c;        
#endif    
    default:    
        return 0;     
    }
}
