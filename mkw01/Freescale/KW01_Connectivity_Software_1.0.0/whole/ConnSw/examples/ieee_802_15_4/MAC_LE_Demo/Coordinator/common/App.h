/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file App.h
* This header file is for MAC LE Demo Coordinator application.
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

#ifndef _APP_H_
#define _APP_H_


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */

/* Enable for CSL demo */
#ifndef mDemoCsl_d
#define mDemoCsl_d 1
#endif

/* Enable for RIT demo */
#ifndef mDemoRit_d
#define mDemoRit_d 0
#endif

/* CSL PIBs */
#if mDemoCsl_d
    /* value expressed in units of 10 MAC symbols - 1.5 seconds */
    #define gMPibCslMaxPeriodValue_c 0x1D4C
#endif

/* RIT PIBs */
#if mDemoRit_d
    #define gMPibRitTxWaitDurationValue_c    0x4E2 // 24 sec 
    #define gMPibRitDataWaitDurationValue_c  0x34  //  1 sec
    #define gMPibRitPeriodValue_c            0x410 // 20 sec
#endif

#define mMacExtendedAddress_c    (0x1111111111111111)

/* Set the Coordinator short address */ 
#define mDefaultValueOfShortAddress_c     0xC000

/* Set the Coordinator PanID */ 
#define mDefaultValueOfPanId_c            0xBEEF

/* Set the End Device short address */ 
#define mDefaultValueOfEDShortAddr_c      0xD000

#define mDefaultValueOfLogicalChannel_c 0

/* Packet payload size */
#define mDataTxMsduLen_c 243

/* Maximum number of outstanding packets */
#define mDefaultValueOfMaxPendingDataPackets_c 1

/* Data Tx interval in ms */
#define gDataTxIntervalMs_c             10000

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/* The various states of the applications state machines. */
enum {
  stateInit,
  stateStartCoordinator,
  stateStartCoordinatorWaitConfirm,
  stateListen
};

/* Events that are passed to the application task. 
   Are defined as byte masks to make possible 
   send multiple events to the task */

#define gAppEvtDummyEvent_c            (1 << 0)
#define gAppEvtMessageFromMLME_c       (1 << 1)
#define gAppEvtMessageFromMCPS_c       (1 << 2)
#define gAppEvtStartCoordinator_c      (1 << 3)

/* Error codes */
enum {
  errorNoError,
  errorWrongConfirm,
  errorNotSuccessful,
  errorNoMessage,
  errorAllocFailed,
  errorInvalidParameter,
  errorNoScanResults
};

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
#ifdef __cplusplus
    extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/**********************************************************************************/
#endif /* _MAPP_H_ */
