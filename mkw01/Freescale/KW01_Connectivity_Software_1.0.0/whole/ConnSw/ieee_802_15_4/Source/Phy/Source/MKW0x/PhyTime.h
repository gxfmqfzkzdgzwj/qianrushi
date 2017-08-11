/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyTime.h
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

#ifndef _PHY_TIME_H_
#define _PHY_TIME_H_

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "EmbeddedTypes.h"

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#ifndef gPhyTime_TPM_instance_c
#define gPhyTime_TPM_instance_c     0
#endif

#define gPhyTime_IRQ_Priority_c     (0x40)

#define gPhyEventTriggerCh_c        (0x00)
#define gPhyEventTimeoutCh_c        (0x01)
#define gPhyWaitTimeoutCh_c         (0x02)
#define gPhyRxTimeoutCh_c           (0x03)
   
// TMR clock 24MHz - 1 tick = (1 / (24000000 / 16)) = 0.666667 us
#define TIME_MULTIPLICATOR_x5(time)  { (time) = ( ( (time) >> 1 ) * 15 ); }  // 5      us symbol
#define TIME_MULTIPLICATOR_x6(time)  { (time) = ( (time) * 10 ); }           // 6.(6)  us symbol   
#define TIME_MULTIPLICATOR_x10(time) { (time) = ( (time) * 15 ); }           // 10     us symbol
#define TIME_MULTIPLICATOR_x20(time) { (time) = ( (time) * 30 ); }           // 20     us symbol
#define TIME_MULTIPLICATOR_x50(time) { (time) = ( (time) * 75 ); }           // 50     us symbol
#define TIME_MULTIPLICATOR_x83(time) { (time) = ( (time) * 1250 ); }         //833     us symbol

#define TIME_DIVIDER_by5(time)  { (time) = ( ( (time) << 1 ) / 15 ); }
#define TIME_DIVIDER_by6(time)  { (time) = ( (time) / 10 ); }
#define TIME_DIVIDER_by10(time) { (time) = ( (time) / 15 ); }
#define TIME_DIVIDER_by20(time) { (time) = ( (time) / 30 ); }
#define TIME_DIVIDER_by50(time) { (time) = ( (time) / 75 ); }
#define TIME_DIVIDER_by83(time) { (time) = ( (time) / 1250 ); }

#define TIME_US_TO_TICKS(time) ( ( (time) * 3 ) >> 1 )
#define TIME_TICKS_TO_US(time) ( ( (time) << 1 ) / 3 )

/*****************************************************************************
 *                             PUBLIC TYPE DEFINITIONS                       *
 *---------------------------------------------------------------------------* 
 *****************************************************************************/        

typedef union phyEvent64BitTimer_tag
{
  uint64_t longAccess;
  uint16_t halfWordAccess[4];   
} phyEvent64BitTimer_t;

/*****************************************************************************
 *                        PUBLIC FUNCTIONS PROTOTYPES                        *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
 *-------------l--------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyTimerInit
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerInit
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimerStart
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerStart
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimerStop
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerStop
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventTrigger_Handler
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventTrigger_Handler
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventTimeout_Handler
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventTimeout_Handler
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventWaitTimeout_Handler
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventWaitTimeout_Handler
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventRxTimeout_Handler
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventRxTimeout_Handler
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_TimeMultiplicator
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_TimeMultiplicator
(
  phyTime_t *time
);

/*---------------------------------------------------------------------------
 * Name: Phy_TimeDivider
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_TimeDivider
(
  phyTime_t *time
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeReadClockTicks
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeReadClockTicks
(
  phyTime_t *pRetClk
);

/*---------------------------------------------------------------------------
 * Name: PhyTimeSyncClockTicks
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSyncClockTicks
(
phyTime_t ticks
);
#endif /* _PHY_TIME_H_ */

