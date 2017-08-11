/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyPib.h
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


#ifndef _PHYPIB_H_
#define _PHYPIB_H_
#include "PhyTypes.h"
/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/******************************************************************
 ** gPhyPibPhyModeSupported_c bitmap defintion                     |
 ** Bit pos:|    15-9      |     8-4        |          3-0         |
 **         |  Reserved    | Frequency band |  Bitmap for PHY mode |
 *******************************************************************/

#define gSUNPage_PhyMode_BitOffset_c          0
#define gSUNPage_ModulationScheme_BitOffset_c 20
#define gSUNPage_FreqBand_BitOffset_c         22
#define gSUNPage_ChannelPage_BitOffset_c      27

#define gPhyPibMaxPmbLen_c  4000
#define gPhyPibMinPmbLen_c  4

/* Extended PHY definitions */

#define mHighestGainMinus6_c		( 190 )          // -95dBm
#define mHighestGainMinus12_c		( 176 )			 // -88dBm		
#define mHighestGainMinus24_c		( 154 )			 // -77dBm
#define mHighestGainMinus36_c		( 136 )			 // -68dBm
#define mHighestGainMinus48_c		( 114 )			 // -57dBm

/*****************************************************************************
 *                             PUBLIC TYPE DEFINITIONS                       *
 *---------------------------------------------------------------------------* 
 *****************************************************************************/

//802.15.4g channel page
enum {
  gChannelPageSeven_c = 0x07,
  gChannelPageEight_c = 0x08,
};

//802.15.4g modulation schemes
enum {
  gFilteredFSK_c = 0x00,
  gOFDM_c        = 0x01,
  gOQPSK         = 0x02,
};

typedef struct phyRFConstatnts_tag  
{
  uint32_t firstChannelFrequency;
  uint32_t channelSpacing;
  uint16_t totalNumChannels;
  uint16_t bitRateReg;
  uint16_t fdevReg;
  uint8_t  dccFreq;
  uint8_t  rxFilterBw;
  uint8_t  dccFreqAfc;
  uint8_t  rxFilterBwAfc;
  uint8_t  modulationType;
  uint8_t  modulationShaping;
  uint8_t  ccaThreshold;
  uint8_t  rssiThreshold;
  uint8_t  lowBetaAfcOffset;
} phyRFConstants_t;

typedef struct phyPib_tag
{
  const phyRFConstants_t *pPIBphyRfConstants;
  uint16_t                mPIBphyTsCcaUs;
  uint16_t                mPIBphyTsRxTxUs;
  uint16_t                mPIBphyTsRxAckDelayUs;
  uint16_t                mPIBphyTsAckWaitUs;
  uint16_t                mPIBphyFSKPreambleRepetitions;
  uint16_t                mPIBphyCCADuration;
  uint16_t                mPIBphyAckWaitDuration;
  uint8_t                 mPIBphyCurrentMode;
  uint8_t                 mPIBphyCurrentChannel;
  uint8_t                 mPIBphyTransmitPower;
  bool_t                  mPIBphyFSKScramblePSDU;
  bool_t                  mPIBAfcEnabled;
  uint8_t                 mPIBRssiThreshold;
  bool_t                  mPIBTxAckFP;
#if CT_Feature_Calibration
  uint32_t mPIBAdditionalRFFrequencyOffset;
  uint8_t  mPIBAdditionalEDOffset;
#endif
} phyPib_t;

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
extern phyPib_t gPhyPib;
extern phyRFConstants_t const phyPibRFConstants[];
#if gFreqBand_920__928MHz_d          // Japan 920-928MHz ARIB custom mode
extern phyRFConstants_t const phyPibRFConstantsARIB[];
#endif

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
 * Name: PhyPib_InitOnStartUp
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_InitOnStartUp
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetCurrentPhyMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetCurrentPhyMode
(
  phyMode_t phyMode
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetCurrentChannel
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetCurrentChannel
(
  uint8_t channel
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetTransmitPower
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetTransmitPower
(
  uint8_t pwrLevel
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetFSKPreambleRepetitions
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetFSKPreambleRepetitions
(
  uint16_t preambleLength
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetFSKScramblePSDU
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetFSKScramblePSDU
(
  bool_t scramblePSDU
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_EnableAFC()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_EnableAFC
(
  bool_t afcEnabled
);

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
 * Name: PhyPib_RFUpdatePreambleLength
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdatePreambleLength
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdatePowerStep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdatePowerStep
(
  void
);            

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdateRFfrequency
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdateRFfrequency
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdateModulationParameters
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdateModulationParameters
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetDefaultValues
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetDefaultValues
(
  void
);

#endif /* _PHYPIB_H_ */
