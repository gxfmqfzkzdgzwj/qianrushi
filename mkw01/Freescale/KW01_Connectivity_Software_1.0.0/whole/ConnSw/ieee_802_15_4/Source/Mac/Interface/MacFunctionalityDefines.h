/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacFunctionalityDefines.h
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

#ifndef _MAC_FUNCTIONALITY_DEFINES_H_
#define _MAC_FUNCTIONALITY_DEFINES_H_

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  MAC features:
 * ==============
 *  06         2006 security
 *  11         2011 security
 *  g          Used with g PHY
 *  e          features followed by listing (LE/TSCH/DSME)
 *  LE         4e low energy features (CSL, RIT)
 *  TSCH       4e Time Slotted Channel Hopping
 *  DSME       4e Deterministic and Synchronous Multi-channel Extension
 *  BE         Beacon order !=15 (for code size reduction purposes)
 *  GTS        GTS support (for code size reduction purposes)
 *  ZP         ZigBee PRO customizations
 *  M0         Cortex M0+
 *  M4         Cortex M4
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define gMacFeatureSet_06M4_d        0
#define gMacFeatureSet_11M4_d        1
#define gMacFeatureSet_ZPM4_d        2
#define gMacFeatureSet_06BEM4_d      3
#define gMacFeatureSet_06BEGTSM4_d   4

#define gMacFeatureSet_06gM0_d       5
#define gMacFeatureSet_11gM0_d       6
#define gMacFeatureSet_06eLEgM0_d    7
#define gMacFeatureSet_11eLEgM0_d    8
#define gMacFeatureSet_06eTSCHgM0_d  9
#define gMacFeatureSet_11eTSCHgM0_d  10

#define gMacFeatureSet_06M0_d        11
#define gMacFeatureSet_11M0_d        12
#define gMacFeatureSet_ZPM0_d        13
#define gMacFeatureSet_06BEM0_d      14
#define gMacFeatureSet_06BEGTSM0_d   15

#define gMacFeatureSet_THR_M4_d      16
#define gMacFeatureSet_THR_M0_d      17
#define gMacFeatureSet_THRRFD_M4_d   18
#define gMacFeatureSet_THRRFD_M0_d   19

/* Default MAC feature set */
#ifndef gMacFeatureSet_d
#define gMacFeatureSet_d gMacFeatureSet_06M4_d
#endif

#if (gMacFeatureSet_d == gMacFeatureSet_06M4_d) || (gMacFeatureSet_d == gMacFeatureSet_06M0_d)
  #define gMacInternalDataSize_c      360 /* [bytes] */
  #define gMacSecurityEnable_d        (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_11M4_d) || (gMacFeatureSet_d == gMacFeatureSet_11M0_d)
  #define gMacInternalDataSize_c      336 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMAC2011_d                  (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_ZPM4_d) || (gMacFeatureSet_d == gMacFeatureSet_ZPM0_d)
  #define gMacInternalDataSize_c      376 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gZPRO_d                     (1)
  #define gMacUsePackedStructs_d      (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_THR_M4_d) || (gMacFeatureSet_d == gMacFeatureSet_THR_M0_d)
  #define gMacInternalDataSize_c      344 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMacThread_d                (1)
  #define gMacUseAssociation_d        (0)
  #define gMacUseOrphanScan_d         (0)
  #define gMacUsePromiscuous_d        (0)
  #define gMacUseRxEnableRequest_d    (0)
  #define gMacPanIdConflictDetect_d   (0)

#elif (gMacFeatureSet_d == gMacFeatureSet_THRRFD_M4_d) || (gMacFeatureSet_d == gMacFeatureSet_THRRFD_M0_d)
  #define gMacInternalDataSize_c      264 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMacThread_d                (1)
  #define gMacUseAssociation_d        (0)
  #define gMacUseOrphanScan_d         (0)
  #define gMacUsePromiscuous_d        (0)
  #define gMacUseRxEnableRequest_d    (0)
  #define gMacUseMlmeStart_d          (0)
  #define gMacPanIdConflictDetect_d   (0)
  #define gMacCoordinatorCapability_d (0)

#elif (gMacFeatureSet_d == gMacFeatureSet_06BEM4_d) || (gMacFeatureSet_d == gMacFeatureSet_06BEM0_d)
  #define gMacInternalDataSize_c      560 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gBeaconEnabledSupport_d     (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_06BEGTSM4_d) || (gMacFeatureSet_d == gMacFeatureSet_06BEGTSM0_d)
  #define gMacInternalDataSize_c      600 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gBeaconEnabledSupport_d     (1)
  #define gGtsSupport_d               (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_06gM0_d)
  #define gMacInternalDataSize_c      464 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  
#elif (gMacFeatureSet_d == gMacFeatureSet_06eLEgM0_d)
  #define gMacInternalDataSize_c      568 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gCslSupport_d               (1)
  #define gRitSupport_d               (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_06eTSCHgM0_d)
  #define gMacInternalDataSize_c      552 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gTschSupport_d              (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_11gM0_d)
  #define gMacInternalDataSize_c      448 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMAC2011_d                  (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_11eLEgM0_d)
  #define gMacInternalDataSize_c      560 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMAC2011_d                  (1)
  #define gCslSupport_d               (1)
  #define gRitSupport_d               (1)

#elif (gMacFeatureSet_d == gMacFeatureSet_11eTSCHgM0_d)
  #define gMacInternalDataSize_c      544 /* [bytes] */
  #define gMacSecurityEnable_d        (1)
  #define gMAC2011_d                  (1)
  #define gTschSupport_d              (1)

#else
  #error Unsupported MAC Feature Set

#endif

/* Set default values */
#ifndef gBeaconEnabledSupport_d
#define gBeaconEnabledSupport_d     (0)
#endif

#ifndef gGtsSupport_d
#define gGtsSupport_d               (0)
#endif

#ifndef gCslSupport_d
#define gCslSupport_d               (0)
#endif

#ifndef gRitSupport_d
#define gRitSupport_d               (0)
#endif

#ifndef gTschSupport_d
#define gTschSupport_d              (0)
#endif

#ifndef gMacSecurityEnable_d
#define gMacSecurityEnable_d        (0)
#endif

#ifndef gMacUseChannelPage_d
#define gMacUseChannelPage_d        (0)
#endif

#ifndef gMacUsePackedStructs_d
#define gMacUsePackedStructs_d      (0)
#endif

#ifndef gMacUseAssociation_d
#define gMacUseAssociation_d        (1)
#endif

#ifndef gMacUseOrphanScan_d
#define gMacUseOrphanScan_d         (1)
#endif

#ifndef gMacUseRxEnableRequest_d
#define gMacUseRxEnableRequest_d    (1)
#endif

#ifndef gMacUsePromiscuous_d
#define gMacUsePromiscuous_d        (1)
#endif 

#ifndef gMacUseMlmeStart_d
#define gMacUseMlmeStart_d          (1)
#endif

#ifndef gMacPanIdConflictDetect_d
#define gMacPanIdConflictDetect_d   (1)
#endif

#ifndef gMacCoordinatorCapability_d
#define gMacCoordinatorCapability_d (1)
#endif

#endif /* _MAC_FUNCTIONALITY_DEFINES_H_ */
