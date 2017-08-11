/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyExtended.h
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

#ifndef PHYEXTENDED_H_
#define PHYEXTENDED_H_

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

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
 * Name: Phy_SetOperationMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOperationMode
(
  uint8_t opMode
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOperationModeFast
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetOperationModeFast
(
  uint8_t opMode
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetPowerStep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetPowerStep
(
  uint8_t outputPower
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetCarrierFreq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetCarrierFreq
(
  uint32_t carrierFrequency
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetBitrate
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetBitrate
(
  uint16_t bitrate
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetFreqDeviation
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetFreqDeviation
(
  uint16_t freqDev
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetPreambleSize
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetPreambleSize
(
  uint16_t preambleSize
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnableSequencer
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_EnableSequencer
(
  uint8_t enableSequencer
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetDataMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDataMode
(
  uint8_t dataMode
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetModulationType
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetModulationType
(
  uint8_t modulationType
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetModulationShaping
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetModulationShaping
(
  uint8_t modulationShaping
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetPowerAmpMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetPowerAmpMode
(
  uint8_t paMode
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetPaRamp
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetPaRamp
(
  uint8_t paRampValueUs
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetDccFreq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDccFreq
(
  uint8_t dccFreq
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetRxFilterBw
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetRxFilterBw
(
  uint8_t rxFilterBw
);   

/*---------------------------------------------------------------------------
 * Name: Phy_SetDccFreqAfc
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDccFreqAfc
(
  uint8_t dccFreqAfc
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetRxFilterBwAfc
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetRxFilterBwAfc
(
  uint8_t rxFilterBwAfc
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOokThresholdType
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokThresholdType
(
  uint8_t thresholdType
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOokFixedThreshold
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetOokFixedThreshold 
(
  uint8_t fixedThresholdDb
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOokPeakThreshStep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokPeakThreshStep 
(
  uint8_t peakThreshStep
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOokPeakThreshDec
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokPeakThreshDec 
(
  uint8_t peakThreshDec
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetOokAvgThreshFilt
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokAvgThreshFilt 
(
  uint8_t avgThresh
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnableAfcLowBeta
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcLowBeta 
(
  bool_t afcLowBetaOn
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetLowBetaAfcOffset
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetLowBetaAfcOffset 
(
  uint8_t lowBetaAfcOffset
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnableAfcAutoclear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcAutoclear 
(
  bool_t afcAutoClearOn
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnableAfcAuto
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcAuto 
(
  bool_t afcAutoOn
);

/*---------------------------------------------------------------------------
 * Name: Phy_ClearAfc
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_ClearAfc 
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetRxTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetRxTimeout 
(
  uint8_t timeout
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetRxTimeoutThreshold
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetRxTimeoutThreshold 
(
  uint8_t timeoutTresh
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetRssiThreshold
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetRssiThreshold 
(
  uint8_t rssiTresh
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetFifoThreshold
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetFifoThreshold 
(
  uint8_t fifoThresh
);

/*---------------------------------------------------------------------------
 * Name: Phy_ClearFifo
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_ClearFifo 
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_RestartRx
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RestartRx 
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetLnaInputImpedance
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetLnaInputImpedance 
(
  uint8_t lnaZin
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnableSensitivityBoost
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_EnableSensitivityBoost
(
  uint8_t sensitivityBoostOn
);

/*---------------------------------------------------------------------------
 * Name: Phy_GetAfc
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_GetAfc
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_GetFei
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t Phy_GetFei
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_GetRssi
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_GetRssi
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetConfig
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetConfig
(
  uint8_t operatingMode, 
  uint8_t outputPower, 
  uint8_t rfChannel
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetSyncWordSize
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetSyncWordSize
(
  uint8_t syncWordSize
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetSyncWordValue
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetSyncWordValue
(
  uint8_t syncWordValueReg, 
  uint8_t syncValue
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetInterPacketRxDelay
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetInterPacketRxDelay
(
  uint8_t interPacketRxDelay
);

/*---------------------------------------------------------------------------
 * Name: Phy_PABoostCfg
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_PABoostCfg
(
  uint8_t u8PAConfig
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetLnaGain
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetLnaGain
(
  uint8_t gainValue
); 
/*---------------------------------------------------------------------------
 * Name: Phy_SetListenCriteria
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCriteria
(
  bool_t threshWithSyncMatch
);
/*---------------------------------------------------------------------------
 * Name: Phy_SetListenEndCriteria
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenEndCriteria 
(
  uint8_t endCondition
);
/*---------------------------------------------------------------------------
 * Name: Phy_SetListenResRx
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenResRx 
(
  uint8_t listenResolution
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetListenCoefRx
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCoefRx 
(
  uint8_t listenCoef
);
/*---------------------------------------------------------------------------
 * Name: Phy_SetListenResIdle
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenResIdle 
(
  uint8_t listenResolution
);

/*---------------------------------------------------------------------------
 * Name: Phy_SetListenCoefIdle
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCoefIdle 
(
  uint8_t listenCoef
);
#endif /* PHYEXTENDED_H_ */
