/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyPlmeData.c
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
#include "Phy.h"
#include "PhyExtended.h"
#include "PhyPib.h"
#include "PhyISR.h"
#include "PhyTime.h"
#include "SX123xDrv.h"
  
#include "fsl_device_registers.h"
//#include "fsl_os_abstraction.h"

 /*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

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
extern Phy_PhyLocalStruct_t     phyLocal;

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
 * Name: PhyPdDataRequest()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPdDataRequest(pdDataReq_t *pTxPacket, volatile phyRxParams_t *pRxParams, 
                             volatile phyTxParams_t *pTxParams)    
{
  uint8_t fifoThreshold = gFifoThreshold_c;
  
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pTxPacket)
  {
    return gPhyInvalidParameter_c;
  }
  
  // if CCA required
  if ((pTxPacket->CCABeforeTx == gPhyCCAMode2_c) ||
      (pTxPacket->CCABeforeTx == gPhyCCAMode3_c) || 
      (pTxPacket->CCABeforeTx == gPhyEnergyDetectMode_c))
  {
    //...cannot perform other types than MODE1
    return gPhyInvalidParameter_c;
  }
  // cannot have packets shorter than gPhyMinDataLength_c 
  if(pTxPacket->psduLength < gMinPHYPacketSize_c - 2)   // substract FCS length
  {
    return gPhyInvalidParameter_c;
  }
  // cannot have packets longer than gPhyMaxDataLength_c
  if(pTxPacket->psduLength > gMaxPHYPacketSize_c - 2)  // substract FCS length
  {
    return gPhyInvalidParameter_c;
  }  
#endif // PHY_PARAMETERS_VALIDATION  
  
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
   
  if(pTxPacket->ackRequired == gPhyRxAckRqd_c)
  {
    PhyIsrPassRxParams(pRxParams);    
    phyLocal.flags.rxAckRqd = TRUE;        
  }
  else if (pTxPacket->ackRequired == gPhyEnhancedAckReq)
  { 
    PhyIsrPassRxParams(pRxParams);  
    phyLocal.flags.rxEnhAckRqd = TRUE;
  }
  else
  { 
    PhyIsrPassRxParams(NULL);
    phyLocal.flags.rxAckRqd = FALSE;   
    phyLocal.flags.rxEnhAckRqd = FALSE;   
  }    
    
  /* FCS is initialized here and computed on the fly */
  phyLocal.fcs = 0;
  
  pTxPacket->phyHeader.mask = 0;
  pTxPacket->phyHeader.dataWhitening = gPhyPib.mPIBphyFSKScramblePSDU;

  /* pass TX PHY parameters to the transceiver ISR */
  fifoThreshold = PhyPassTxParams(pTxPacket);

  /* Set FIFO threshold */
  Phy_SetFifoThreshold(fifoThreshold);
  
  // perform CCA before TX if required
  if(pTxPacket->CCABeforeTx != gPhyNoCCABeforeTx_c)
  {
    phyLocal.flags.ccaBfrTX = TRUE;         

    PhyPlmeCcaEdRequest(gPhyCCAMode1_c, gPhyContCcaDisabled);    
  }
  else
  {  
    PhyPrepareTx(pTxPacket);
    
    if(phyLocal.startTime == gPhySeqStartAsap_c)
    {
      PhyTimeDisableEventTrigger();
      /* Put transceiver in TX mode */     
      Phy_SetOperationModeFast(OpMode_Transmitter);
    }
  }
  
  return gPhySuccess_c;  
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeRxRequest()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPlmeRxRequest(phySlottedMode_t phyRxMode, phyRxParams_t *pRxParams)
{  
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pRxParams)
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION
  
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  
  //configure RSSI threshold
  Phy_SetRssiThreshold(gPhyPib.mPIBRssiThreshold);

  //pass RX PHY parameters to the transceiver ISR
  PhyIsrPassRxParams(pRxParams);

  //put PHY in RX state
  PhySetState(gRX_c);
  
  XCVRDrv_ConfigureDIOPins((DIO0_RxSyncAddress | DIO1_RxFifoLevel | DIO2_RxLowBat | DIO3_RxFifoFull),(DIO4_RxRxReady | DIO5_RxClkOut));
  
  /* Set FIFO threshold */
  Phy_SetFifoThreshold(gPhyMRFSKPHRLength_c-1);
  
  gpfPendingRxEvent = PhyRxHandleSyncAddresEvent;

  /* Enable RxReady as interrupt (rising edge) */
  XCVRDrv_IRQ_DIO4_Enable(0x09);
  
  if(phyLocal.startTime == gPhySeqStartAsap_c)
  {
    PhyTimeDisableEventTrigger();
#ifdef gListenSupported_c
    if(phyLocal.flags.rxIsListen)
    {
      Phy_SetOperationModeFast((OpMode_StandBy | OpMode_Listen_On)); 
    }
    else
#endif
    {
	  //Put transceiver in RX mode
      Phy_SetOperationModeFast(OpMode_Receiver);     
    }
  }  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeCcaEdRequest()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPlmeCcaEdRequest(phyCCAType_t ccaParam, phyContCCAMode_t cccaMode)
{
#ifdef PHY_PARAMETERS_VALIDATION
  // only CCA mode1 or gCcaED_c 
  if((gPhyCCAMode1_c != ccaParam) && (gPhyEnergyDetectMode_c != ccaParam))
  {
    return gPhyInvalidParameter_c;
  }
  // cannot perform Continuous CCA using ED type
  if((ccaParam == gPhyEnergyDetectMode_c) && (cccaMode == gPhyContCcaEnabled))
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION  
  
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }

  //configure RSSI threshold to -127.5 dBm
  Phy_SetRssiThreshold(0xFF); 
  
  phyLocal.channelParams.energyLeveldB = 0;  
  phyLocal.channelParams.ccaParam = ccaParam;       
  
  //put PHY in CCA or ED state
  if(gPhyCCAMode1_c == ccaParam)
  {    
    // normal CCA (not continuous)
    PhySetState(gCCA_c);
  }
  else
  {
    // ED sequence
    PhySetState(gED_c);
  }
  
  /* This flag will be cleared on Rx Timeout handler */
  phyLocal.flags.ccaComplete = FALSE;
  
  XCVRDrv_ConfigureDIOPins((DIO0_RxRssi | DIO1_RxFifoNotEmpty | DIO2_RxLowBat | DIO3_RxFifoFull),(DIO4_RxRxReady | DIO5_RxClkOut));

   /* Enable RxReady as interrupt (rising edge) */
  XCVRDrv_IRQ_DIO4_Enable(0x09);

  /* AFC has to be disabled for CCA/ED */
  if( gPhyPib.mPIBAfcEnabled )
  {
    Phy_EnableAfcAuto(FALSE);
  } 
  
  phyTime_t ccaEndTime;
  phyTime_t ccaDuration;
 
  if( phyLocal.flags.tschEnabled )
  {
    ccaDuration = TIME_US_TO_TICKS(gPhyPib.mPIBphyTsCcaUs);
  }
  else
  {
    /* Convert CCA duration from MAC symbols to ticks */
    ccaDuration = gPhyPib.mPIBphyCCADuration;
    Phy_TimeMultiplicator(&ccaDuration);
  }
  
  if( phyLocal.startTime == gPhySeqStartAsap_c )
  {
    PhyTimeReadClockTicks(&ccaEndTime);
    ccaEndTime += Phy_XCVRRxWarmupTime();
  }
  else
  {
    ccaEndTime = phyLocal.startTime;
  }
  
  ccaEndTime += ccaDuration;
  PhyTimeSetRxTimeout(&ccaEndTime);

  if(phyLocal.startTime == gPhySeqStartAsap_c)
  {
    PhyTimeDisableEventTrigger();       
    //Put transceiver in RX mode
    Phy_SetOperationModeFast(OpMode_Receiver);
  }

  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPrepareTx()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPrepareTx(pdDataReq_t *pTxPacket)
{
  /* Write PHR in FIFO */
  XCVRDrv_WriteFifoLSB(pTxPacket->phyHeader.byteAccess[0]);
    
  /* Frame length is sent MSB first */
  XCVRDrv_WriteFifo(pTxPacket->phyHeader.byteAccess[1]);
  
  /* Remove FCS len and use it to compute remaining length */
  pTxPacket->phyHeader.frameLength -= 2;    
    
  /* Pre-fill FIFO */
  XCVRDrv_FillFifo();
    
  PhySetState(gTX_c); 
    
  gpfPendingTxEvent = PhyTxHandleFifoLevelEvent;
  
  XCVRDrv_ConfigureDIOPins((DIO0_TxPacketSent | DIO1_TxFifoLevel | DIO2_TxLowBat | DIO3_TxLowBat), (DIO4_TxLowBat | DIO5_TxClkOut));  
    
  /* Enable PacketSent and FifoLevel */
  XCVRDrv_IRQ_DIO0_Enable(0x09); /* PacketSent - Rising */
  XCVRDrv_IRQ_DIO1_Enable(0x0A); /* FifoLevel - Falling */
}

/*---------------------------------------------------------------------------
 * Name: PhyAbort()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyAbort(void)
{  
  if( gIdle_c != PhyPpGetState() )
  {
    XCVRDrv_IRQ_DIO0_Disable();
    XCVRDrv_IRQ_DIO1_Disable();
    XCVRDrv_IRQ_DIO4_Disable();

    gpfPendingRxEvent = PhyRxHandleDummyEvent;    
    gpfPendingTxEvent = PhyTxHandleDummyEvent;
    
    PhySetState(gIdle_c);
    
#ifdef gListenSupported_c
    if(phyLocal.flags.rxIsListen)
    {
      Phy_SetOperationModeFast((OpMode_StandBy | OpMode_Listen_Abort));
    }
#endif
    //Put transceiver in Stand-By mode
    Phy_SetOperationModeFast(OpMode_StandBy);
    
    //clear the transceiver FIFO
    Phy_ClearFifo();
  }
}
