/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyISR.h
* PHY ISR Functions
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

#ifndef _PHY_ISR_H_
#define _PHY_ISR_H_

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define gPhy_IRQ_Port_c         PORTC_PORTD_IRQn
#define gPhy_IRQ_Priority_c     (0x80)
   
/// \note MUST REMAIN UNCHANGED:
#define gFillFifoBlockLength_c    8
#define gFifoThreshold_c         (gFillFifoBlockLength_c - 1) 
#define gRxFifoBlockLength_c      5

/* StandBy - Tx warmup times */
#define gTs_Tr_1_c                      400 /* 1.2kbps */
#define gTs_Tr_20_c                     204 /*  20kbps */   
#define gTs_Tr_50_c                     220 /*  50kbps */
#define gTs_Tr_100_c                    192 /* 100kbps */
#define gTs_Tr_150_c                    220 /* 150kbps */
#define gTs_Tr_200_c                    220 /* 200kbps */ //TODO

/* StandBy - Rx warmup times */
#define gTs_Re_1_c                     4332 /* 1.2kbps */ /* Mode Ready */  
#define gTs_Re_20_c                     622 /*  20kbps */ /* Mode Ready */  
#define gTs_Re_50_c                     668 /*  50kbps */
#define gTs_Re_100_c                    412 /* 100kbps */
#define gTs_Re_150_c                    296 /* 150kbps */
#define gTs_Re_200_c                    296 /* 200kbps */ //TODO

#define gPanIdLength_c          (2)

// Frame Control Field (Lsb) masks:
#define gFrameTypeMask_c        (0x07)  //                    00000111
#define gSecurityEnableFlag_c   (0x08)  // (1<<3)   // 0x08 - 00001000
#define gFramePendingFlag_c     (0x10)  // (1<<4)   // 0x10 - 00010000 or ((uint8_t) (1<<4))
#define gAckReqFlag_c           (0x20)  // (1<<5)   // 0x20 - 00100000
#define gIntraPanFlag_c         (0x40)  // (1<<6)   // 0x40 - 01000000 - MAC 2006 PanIdCompression field
#define gCCA2Flag_c             (0x80)  // (1<<7)   // 0x80 - 10000000 - freescale internal use of the reserved use bit, cleared in PHY -> data.c

#define gFrameTypeShift_c       (0)   
   
// Frame Control Field (Msb) masks :

// Address mode indentifiers. Used for both network and MAC interfaces
#define gAddrModeNoAddr_c       (0)
#define gAddrMode16BitAddr_c    (2)
#define gAddrMode64BitAddr_c    (3)
  
#define gDstAddrModeMask_c      (0x0C)  //        00001100
#define gSrcAddrModeMask_c      (0xC0)  //        11000000

#define gAddrModeMask_c         (0x3)
#define gDstModeShift_c         (2)
#define gSrcModeShift_c         (6)

#define gFrameVersionMask_c     (0x30)  //        00110000
#define gFrameVersionShift_c    (4) 

// Frame version identifiers
#define gFrameVersion2003_c      (0x00)
#define gFrameVersion2006_2011_c (0x01)
#define gFrameVersion4ge_c       (0x02)

// Frame Types
#define gFrameTypeBeacon_c       (0x00)
#define gFrameTypeData_c         (0x01)
#define gFrameTypeAck_c          (0x02)
#define gFrameTypeCommand_c      (0x03)

#define gIeListPresentFlag_c     (0x02) // (1<<1) // 0x02 - 00000010 - IE list present fiels is one if IEs are contained in the frame
#define gSNSuppresionFlag_c      (0x01) // (1<<0) // 0x01 - 00000001 - Sequence number suppersion flag

#define gMacMinHeaderLengthAck_c  (5) /* Smallest packet (ACK) excluding FCS */
#define gMacMinHeaderLength_c     (9) /* Smallest packet (ACK) excluding FCS */

#define gFrameTypeMultipurpose_c        (0x05)

#define gMppLongFrameControlMask_c      (0x08)
#define gMppFrameDstAddrModeMask_c      (0x30)
#define gMppFrameSrcAddrModeMask_c      (0xC0)
#define gMppDstAddrModeShift_c          (0x04)
#define gMppSrcAddrModeShift_c          (0x06)

#define Phy_TxLength(PacketLength)       ((uint32_t)( gPhySHRDuration_c + (PacketLength) * gPhySymbolsPerOctet_c))
#define Phy_RxLength(PacketLength)       ((uint32_t)( (PacketLength) * gPhySymbolsPerOctet_c))
   
typedef struct hdrGenericHeaderType_tag {
  uint8_t frameControlLsb;
  uint8_t frameControlMsb;
  uint8_t seqNr;
  uint8_t addrFieldsStart;
} hdrGenericHeaderType_t; 

// Generic Header Format
// For accessing fixed location fields in (MPDU/SPDU) rx/txData in rx/txPacket_t:
#define gFrameControlLsbPos_c   (GetRelAddr(hdrGenericHeaderType_t, frameControlLsb)) // Position in rx/txData in rx/txPacket_t
#define gFrameControlMsbPos_c   (GetRelAddr(hdrGenericHeaderType_t, frameControlMsb)) // Position in rx/txData in rx/txPacket_t
#define gSeqNumberPos_c         (GetRelAddr(hdrGenericHeaderType_t, seqNr))           // Position in rx/txData in rx/txPacket_t
#define gAddrFieldsStartPos_c   (GetRelAddr(hdrGenericHeaderType_t, addrFieldsStart)) // Position in rx/txData in rx/txPacket_t

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
extern void (* gpfPendingRxEvent)(void);
extern void (* gpfPendingTxEvent)(void);

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyTxHandleDummyEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxHandleDummyEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_FillFifo
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_FillFifo
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTxPacketSentEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxPacketSentEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTxHandleFifoLevelEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTxHandleFifoLevelEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleDummyEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleDummyEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyRxHandlePayloadReadyEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandlePayloadReadyEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleFifoLevelEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleFifoLevelEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyRxHandlePHREvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandlePHREvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyRxHandleSyncAddresEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyRxHandleSyncAddresEvent
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRHandlerInterrupt
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRHandlerInterrupt
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_ISR 
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_ISR
(
  void
);


/*---------------------------------------------------------------------------
 * Name: CCAPinHandler_Interrupt
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void CCAPinHandler_Interrupt
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_RxHandleFilterFail
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RxHandleFilterFail
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_IsrRxFilterDest
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_IsrRxFilterDest
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_CrcAddByte
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_CrcAddByte
(
  uint8_t data
);

/*---------------------------------------------------------------------------
 * Name: Phy_RxFrameFilter
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RxFrameFilter
(
  void
);

/*****************************************************************************
 *                        PUBLIC FUNCTIONS PROTOTYPES                        *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyTime_ISR
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTime_ISR
(
  void
);	

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventTimeout
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventTrigger
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventTrigger
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventWaitTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventWaitTimeout
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeHandleEventRxTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeHandleEventRxTimeout
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_XCVRTxWarmupTime
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_XCVRTxWarmupTime
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_XCVRRxWarmupTime
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_XCVRRxWarmupTime
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_LqiConvert
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_LqiConvert
(
  uint8_t rssiValue
);

#endif /* _PHY_ISR_H_ */
