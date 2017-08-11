/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SX123xDrv.h
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

#ifndef _SX123XDRV_H_
#define _SX123XDRV_H_

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#if defined(__IAR_SYSTEMS_ICC__)
  #include "intrinsics.h"
#endif
   
#include "EmbeddedTypes.h"
#include "board.h"
   
#if BOARD_CLOCK_SOURCE_30_MHZ
    #include "SX123xReg.h"      // 30MHz FXOSC register settings
#else
    #include "SX123xReg32MHz.h" // 32MHz FXOSC register settings
#endif 

/*****************************************************************************
 *                             PRIVATE MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/  

#define gXcvrAssertReset   GPIO_DRV_SetPinOutput(kGpioXcvrReset)
#define gXcvrDeassertReset GPIO_DRV_ClearPinOutput(kGpioXcvrReset)
   
#define gXcvrClkoutReady   GPIO_DRV_ReadPinInput(kGpioXcvrClk)   
   
/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_Reset
 * Description: During MCU platform initialization at power-up this primitive may be invoked by 
 * the platform to perform a hardware reset of the transceiver. 
 * This must be done by asserting an active low pulse onto the RST pin through an MCU GPIO. 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t XCVRDrv_Reset
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ConfigureDIOPins
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ConfigureDIOPins
(
  uint8_t regDio1, 
  uint8_t regDio2
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ConfigureCLKO
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ConfigureCLKO
(
  uint8_t fxOscDiv
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_SetOperatingMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_SetOperatingMode
(
  uint8_t mode
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RadioWakeUpReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RadioWakeUpReq
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RadioSleepReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RadioSleepReq
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadRegister
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t XCVRDrv_ReadRegister
(
  uint8_t addr
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadFifo
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t XCVRDrv_ReadFifo
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadBytesFromFifoLSB
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ReadBytesFromFifoLSB
(
  uint8_t* buffer,
  uint8_t size
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteRegister
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteRegister
(
  uint8_t addr, 
  uint8_t val
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteBytesToFifoLSB
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteBytesToFifoLSB
(
  uint8_t* buffer,
  uint8_t size
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteFifo
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteFifo
(
  uint8_t val
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteFifoLSB
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteFifoLSB
(
  uint8_t val
); 

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RFdefaultInit
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RFdefaultInit
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO0_Enable
(
  uint8_t interruptCfg
);   

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO1_Enable
(
  uint8_t interruptCfg
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO4_Enable
(
  uint8_t interruptCfg
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO0_Disable
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO1_Disable
(
  void
);   

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO4_Disable
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t XCVRDrv_IRQ_DIO0_Detected
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t XCVRDrv_IRQ_DIO1_Detected
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t XCVRDrv_IRQ_DIO4_Detected
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO0_Clear
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO1_Clear
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_IRQ_DIO4_Clear
(
  void
);

/*---------------------------------------------------------------------------
 * Name: XCVR_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Init
(
  void
);

#endif
