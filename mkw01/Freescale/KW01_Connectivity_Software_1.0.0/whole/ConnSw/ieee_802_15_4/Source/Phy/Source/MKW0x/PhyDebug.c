/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyDebug.c
*  PHY debug and logging functions
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
#include "SX123xDrv.h"
#include "Phy.h"
#include "Phydebug.h"

#ifdef MAC_PHY_DEBUG

/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
uint16_t nDebugIndex = 0, nDebugSize = DEBUG_LOG_ENTRIES * 4;
uint8_t  nDebugStorage[DEBUG_LOG_ENTRIES * 4];


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
 * Name: PhyDebugLogTime()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyDebugLogTime(uint8_t item)
{
  phyTime_t time;
  nDebugStorage[nDebugIndex + 0] = item;  
  
  PhyTimeReadClock(&time);
  
  nDebugStorage[nDebugIndex + 3] = (uint8_t) (time >> 0);
  nDebugStorage[nDebugIndex + 2] = (uint8_t) (time >> 8);
  nDebugStorage[nDebugIndex + 1] = (uint8_t) (time >> 16);
  
  nDebugIndex += 4;
  if(nDebugIndex >= nDebugSize)
  {
    nDebugIndex = 0;
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyDebugLogParam1()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyDebugLogParam1(uint8_t item, uint8_t param1)
{
  nDebugStorage[nDebugIndex + 0] = item;
  nDebugStorage[nDebugIndex + 1] = param1;
  nDebugStorage[nDebugIndex + 2] = 0;
  nDebugStorage[nDebugIndex + 3] = 0;

  nDebugIndex += 4;
  if(nDebugIndex >= nDebugSize)
  {
    nDebugIndex = 0;
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyDebugLogParam2()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyDebugLogParam2(uint8_t item, uint8_t param1, uint8_t param2)
{
  nDebugStorage[nDebugIndex + 0] = item;
  nDebugStorage[nDebugIndex + 1] = param1;
  nDebugStorage[nDebugIndex + 2] = param2;
  nDebugStorage[nDebugIndex + 3] = 0;

  nDebugIndex += 4;
  if(nDebugIndex >= nDebugSize)
  {
    nDebugIndex = 0;
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyDebugLogParam3()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyDebugLogParam3(uint8_t item, uint8_t param1, uint8_t param2, uint8_t param3)
{
  nDebugStorage[nDebugIndex + 0] = item;
  nDebugStorage[nDebugIndex + 1] = param1;
  nDebugStorage[nDebugIndex + 2] = param2;
  nDebugStorage[nDebugIndex + 3] = param3;

  nDebugIndex += 4;
  if(nDebugIndex >= nDebugSize)
  {
    nDebugIndex = 0;
  }
}


#endif /* MAC_PHY_DEBUG */

