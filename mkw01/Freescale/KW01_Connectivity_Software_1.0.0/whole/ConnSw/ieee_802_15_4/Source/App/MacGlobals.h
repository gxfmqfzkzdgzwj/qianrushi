/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacGlobals.h
* This is the header file for the MacGlobals.c
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


#ifndef _MAC_GLOBALS_H_
#define _MAC_GLOBALS_H_

#include "MacInterface.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/*! *****************************/
/*** MAC Security Tables sizes **/
/****************************** */
#if gMacSecurityEnable_d
#ifndef gNumKeyTableEntries_c
#define gNumKeyTableEntries_c                       2
#endif

#ifndef gNumKeyIdLookupListEntries_c
#define gNumKeyIdLookupListEntries_c                2
#endif

#ifndef gMAC2011_d
    #ifndef gNumKeyDeviceListEntries_c
    #define gNumKeyDeviceListEntries_c              2
    #endif
#else /* gMAC2011_d */
    #ifndef gNumDeviceDescriptorHandleListEntries_c
    #define gNumDeviceDescriptorHandleListEntries_c 2
    #endif
#endif /* gMAC2011_d */

#ifndef gNumKeyUsageListEntries_c
#define gNumKeyUsageListEntries_c                   2
#endif

#ifndef gNumDeviceTableEntries_c
#define gNumDeviceTableEntries_c                    2
#endif

/* Do not change the following macro definition!
 */
#ifndef gNumDeviceAddrTableEntries_c
    #ifndef gMAC2011_d
        #if (gNumDeviceTableEntries_c > gNumKeyDeviceListEntries_c)
        #define gNumDeviceAddrTableEntries_c (gNumKeyDeviceListEntries_c + 2)
        #else
        #define gNumDeviceAddrTableEntries_c (gNumDeviceTableEntries_c)
        #endif
    #else
        #if (gNumDeviceTableEntries_c > gNumDeviceDescriptorHandleListEntries_c)
        #define gNumDeviceAddrTableEntries_c (gNumDeviceDescriptorHandleListEntries_c + 2)
        #else
        #define gNumDeviceAddrTableEntries_c (gNumDeviceTableEntries_c)
        #endif
    #endif
#endif

#ifndef gNumSecurityLevelTableEntries_c
#define gNumSecurityLevelTableEntries_c             2
#endif
#endif /* gMacSecurityEnable_d */

/*! ****************************************/
/*** MAC LE Tables sizes and transactions **/
/***************************************** */
#if gCslSupport_d
#ifndef gMacCslTableSize_c
#define gMacCslTableSize_c          (10)
#endif

#ifndef gMacCslMaxSequencesCnt_c
#define gMacCslMaxSequencesCnt_c    (12)
#endif
#endif /* gCslSupport_d */

#if gRitSupport_d
#ifndef gMacRitTableSize_c
#define gMacRitTableSize_c          (10)
#endif

#ifndef gMacRitMaxSequencesCnt_c
#define gMacRitMaxSequencesCnt_c    (12)
#endif
#endif /* gRitSupport_d */

/*! *************************/
/*** MAC TSCH Tables sizes **/
/************************** */
#if gTschSupport_d
#ifndef gMacHoppingSequenceTableEntries_c
#define gMacHoppingSequenceTableEntries_c   (128)
#endif

#ifndef gMacSlotframeTableEntries_c
#define gMacSlotframeTableEntries_c         (2)
#endif

#ifndef gMacLinkTableEntries_c
#define gMacLinkTableEntries_c              (10)
#endif

#ifndef gMacTschNeighborTableEntries_c
#define gMacTschNeighborTableEntries_c      (10)
#endif

#ifndef gMacTschMaxPanCoordSync_c
#define gMacTschMaxPanCoordSync_c           (1)       
#endif

#ifndef gMacTschMaxTransactions_c
#define gMacTschMaxTransactions_c           (10)       
#endif
#endif /* gTschSupport_d */

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern uint8_t gMacData[gMacInstancesCnt_c][gMacInternalDataSize_c];
extern uint8_t gMacMaxIndirectTransactions;
extern const uint8_t gMacNoOfInstances;

/************************************************************************************
*************************************************************************************
* Public functions prototypes
*************************************************************************************
************************************************************************************/

#endif