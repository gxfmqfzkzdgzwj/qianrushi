/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyPacketProcessor.c
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
#include "PhyExtended.h"
#include "PhyPib.h"
#include "PhyTime.h"
#include "PhyISR.h"

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

// Address mode indentifiers. Used for both network and MAC interfaces
#define gPhyAddrModeNoAddr_c        (0)
#define gPhyAddrModeInvalid_c       (1)
#define gPhyAddrMode16BitAddr_c     (2)
#define gPhyAddrMode64BitAddr_c     (3)

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
extern Phy_PhyLocalStruct_t      phyLocal;

const uint8_t gPhyIndirectQueueSize_c = gPhyIndirectQueueSize4g_c;

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                                PRIVATE FUNCTIONS                          *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

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
 * Name: PhyGetRandomNo
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyGetRandomNo(uint32_t *pRandomNo)
{
  uint8_t *p = (uint8_t*)pRandomNo;
  uint8_t  i = 4;
  
  //configure RSSI threshold to -127.5 dBm
  Phy_SetRssiThreshold(0xFF); 

  while(i--)
  {
    Phy_SetOperationModeFast(OpMode_Receiver);
  
    //wait for RSSI sample to complete
    while( !(XCVRDrv_ReadRegister(XCVR_Reg_IrqFlags1) & IrqFlags1_RssiReady) );
    
    *p++ = Phy_GetRssi();
    
    Phy_SetOperationModeFast(OpMode_StandBy);
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyPpSetPromiscuous
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPpSetPromiscuous(bool_t mode)
{
  phyLocal.flags.promiscuous = mode;
}

/*---------------------------------------------------------------------------
 * Name: PhySetActivePromiscuous()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhySetActivePromiscuous(bool_t state)
{
  /* if Prom is set */
  if( TRUE == state )
  {
    if( phyLocal.flags.promiscuous == TRUE )
    {
       /* Disable Promiscuous mode */
       phyLocal.flags.promiscuous = FALSE;

       /* Enable Active Promiscuous mode */
       phyLocal.flags.activePromiscuous = TRUE;
     }
  }
  else
  {
    if( phyLocal.flags.activePromiscuous == TRUE)
    {
      /* Disable Active Promiscuous mode */
      phyLocal.flags.activePromiscuous = FALSE;

      /* Enable Promiscuous mode */
      phyLocal.flags.promiscuous = TRUE;
    }
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyGetActivePromiscuous()
 * Description: - returns the state of ActivePromiscuous feature (Enabled/Disabled)
 * Parameters: -
 * Return: - TRUE/FALSE
 *---------------------------------------------------------------------------*/
bool_t PhyGetActivePromiscuous(void)
{
  return phyLocal.flags.activePromiscuous;
}

/*---------------------------------------------------------------------------
 * Name: PhyPpSetPanId
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPpSetPanId(uint8_t *pPanId, uint8_t pan)
{
  (void)pan;
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pPanId)
  {
    return gPhyInvalidParameters_c;   
  }
#endif  // PHY_PARAMETERS_VALIDATION
  
  if(gIdle_c != PhyGetSeqState())
  {
    return gPhyBusy_c;
  }
  
  phyLocal.macPanID[0] = *pPanId;
  phyLocal.macPanID[1] = *(++pPanId);
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPpSetShortAddr
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPpSetShortAddr(uint8_t *pShortAddr, uint8_t pan)
{
  (void)pan;
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pShortAddr)
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION

  if( gIdle_c != PhyGetSeqState() )
  {
    return gPhyBusy_c;
  }
  
  phyLocal.macShortAddress[0] = *pShortAddr; 
  phyLocal.macShortAddress[1] = *(++pShortAddr);
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPpSetLongAddr
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPpSetLongAddr(uint8_t *pLongAddr, uint8_t pan)
{
  (void)pan;
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pLongAddr)
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION

  if( gIdle_c != PhyGetSeqState() )
  {
    return gPhyBusy_c;
  }
  
  phyLocal.macLongAddress[0] = *pLongAddr;
  phyLocal.macLongAddress[1] = *(++pLongAddr);
  phyLocal.macLongAddress[2] = *(++pLongAddr);
  phyLocal.macLongAddress[3] = *(++pLongAddr);
  phyLocal.macLongAddress[4] = *(++pLongAddr);
  phyLocal.macLongAddress[5] = *(++pLongAddr);
  phyLocal.macLongAddress[6] = *(++pLongAddr);
  phyLocal.macLongAddress[7] = *(++pLongAddr);
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPpSetMacRole
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPpSetMacRole(bool_t macRole,  uint8_t pan)
{
  (void)pan;
  if( gIdle_c != PhyGetSeqState() )
  {
    return gPhyBusy_c;
  }  

  if(gMacRole_PanCoord_c == macRole)
  {
    phyLocal.flags.panCordntr = TRUE;
  }
  else
  {
    phyLocal.flags.panCordntr = FALSE;
  } 

  return gPhySuccess_c;
}


/*---------------------------------------------------------------------------
 * Name: Phy_IndirectQueueChecksum
 * Description: Function called to compute the checksum for a 16bit or 64bit address
 * in the same way as the transceiver
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_IndirectQueueChecksum(bool_t addrType, uint64_t address, uint16_t panId)
{
  uint16_t checksum;  
  
  if (addrType)   // Short Address 
  {
    checksum = (panId + (uint16_t)address) & 0xffff;
  }
  else  // Long Address
  { 
    checksum = (panId + (uint16_t)address) & 0xffff;
    checksum = (checksum + (uint16_t)(address >> 16)) & 0xffff;
    checksum = (checksum + (uint16_t)(address >> 32)) & 0xffff;
    checksum = (checksum + (uint16_t)(address >> 48)) & 0xffff;
  }
  return checksum;
}  

/*---------------------------------------------------------------------------
 * Name: PhyPp_IndirectQueueInsert
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPp_IndirectQueueInsert 
(
  uint8_t  index,
  uint16_t checkSum,
  instanceId_t instanceId
)
{
  if( index >= gPhyIndirectQueueSize_c )
    return gPhyInvalidParameter_c;

  phyLocal.phyIndirectQueue[index] = checkSum;
  phyLocal.phyUnavailableQueuePos |= (1 << index);  

  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPp_RemoveFromIndirect
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPp_RemoveFromIndirect(uint8_t index, instanceId_t instanceId)
{
  if( index >= gPhyIndirectQueueSize_c )
    return gPhyInvalidParameter_c;
  
  phyLocal.phyUnavailableQueuePos &= ~(1 << index);
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 *Name: PhyHwInit
 *Description: -
 *Parameters: -
 *Return: - 
 *---------------------------------------------------------------------------*/
void PhyHwInit(void)    
{ 
  /* XCVR init */
  XCVR_Init();
  XCVRDrv_RFdefaultInit();
  
  /* PHY ISR init */
  //PHY_InstallIsr();
 
  /* PHY PIB init */
  PhyPib_InitOnStartUp(); 
  
  /* PHY Timer init */
  PhyTimerInit();
}
