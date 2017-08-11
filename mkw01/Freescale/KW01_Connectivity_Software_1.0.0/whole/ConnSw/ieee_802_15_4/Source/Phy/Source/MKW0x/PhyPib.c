/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyPib.c
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
#include "Phy.h"
#include "PhyExtended.h"
#include "PhyPib.h"
#include "SX123xDrv.h"
#if CT_Feature_Calibration
#include "Flash_Adapter.h"
#endif

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/* RF Center Frequency additional offset as set to Frf Msb/Mid/Lsb registers */
#if !CT_Feature_Calibration
#define gAdditionalRFCarrierFreqOffset_c   0x00
#else
#define gMinOffsetRF_c (-1000)
#define gMaxOffsetRF_c (1000)

#define gMinOffsetED_c (-30)
#define gMaxOffsetED_c (30)
#endif

phyRFConstants_t const phyPibRFConstants[] =
{
#if gFreqBand_470__510MHz_d     // China 470-510MHz
// 470 Operating Mode 1
  {
  //channel center frequency 0
    470200000,
  //channel spacing
    200000,
  //total number of channels
    199,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    81,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07 
  }
#elif gFreqBand_779__787MHz_d      // China 779-787MHz
// 779 Operating Mode 1
  {
  //channel center frequency 0
    779200000,
  //channel spacing
    200000,
  //total number of channels
    39,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    81,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07 
  }
#elif gFreqBand_863__870MHz_d      // Europe 863-870MHz
// 863 Operating Mode 1
  {
  //channel center frequency 0
    863125000,
  //channel spacing
    200000,
  //total number of channels
    34,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    81,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  },
// 863 Operating Mode 2
  {
  //channel center frequency 0
    863225000,
  //channel spacing
    400000,
  //total number of channels
    17,
  //Radio bit rate register
    Bitrate_100000,
  //Radio frequency deviation register
    Fdev_50000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_166700,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_250000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    78,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x0D
  }  
#elif gFreqBand_902__928MHz_d      // U.S. ISM 902-928MHz
// 902 Operating Mode 1
  {
  //channel center frequency 0
    902200000,
  //channel spacing
    200000,
  //total number of channels
    129,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    81,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  },
// 902 Operating Mode 2
  {
  //channel center frequency 0
    902400000,
  //channel spacing
    400000,
  //total number of channels
    64,
  //Radio bit rate register
    Bitrate_150000,
  //Radio frequency deviation register
    Fdev_37500,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_125000,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_200000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    76,
  //RSSI threshold
    0xB8,
  //Low Beta Afc Offset
    0x0D 
  }    
#elif gFreqBand_920__928MHz_d      // Japan 920-928MHz
// 920 Operating Mode 1
  {
  //channel center frequency 0
    920600000,
  //channel spacing
    200000,
  //total number of channels
    38,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_BT_05,
  //CCA threshold
    81,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  },   
// 920 Operating Mode 2
  {
  //channel center frequency 0
    920900000,
  //channel spacing
    400000,
  //total number of channels
    18,
  //Radio bit rate register
    Bitrate_100000,
  //Radio frequency deviation register
    Fdev_50000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_166700,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_250000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_BT_05,
  //CCA threshold
    78,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x0D
  }
#elif gFreqBand_865__867MHz_d      // India 865-867MHz
// 865 Operating Mode 1
  {
  //channel center frequency 0
    865750000,
  //channel spacing
    200000,
  //total number of channels
    10,
  //Radio bit rate register
    Bitrate_1200,
  //Radio frequency deviation register
    Fdev_2400, /* Modulation index 4 */
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_7800,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_10400,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    78,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  },
// 865 Operating Mode 2
  {
  //channel center frequency 0
    865750000,
  //channel spacing
    200000,
  //total number of channels
    10,
  //Radio bit rate register
    Bitrate_20000,
  //Radio frequency deviation register
    Fdev_5000, /* Modulation index 0.5 */
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_20800,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_25000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    78,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  },
// 865 Operating Mode 3
  {
  //channel center frequency 0
    865750000,
  //channel spacing
    200000,
  //total number of channels
    10,
  //Radio bit rate register
    Bitrate_100000,
  //Radio frequency deviation register
    Fdev_35000,
  //DC Cut-off frequency
    DccFreq_5,
  //Radio Filter Bandwidth
    RxBw_100000,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_NoShaping,
  //CCA threshold
    78,
  //RSSI threshold
    0xBE,
  //Low Beta Afc Offset
    0x07
  }  
#endif
};

#if gFreqBand_920__928MHz_d          // Japan 920-928MHz ARIB custom mode
phyRFConstants_t const phyPibRFConstantsARIB[] =
{
  // 920 Operating Mode 1 ARIB  - 50kbps
  {
  //channel center frequency 0
    920600000,
  //channel spacing
    200000,
  //total number of channels
    38,
  //Radio bit rate register
    Bitrate_50000,
  //Radio frequency deviation register
    Fdev_25000,
  //DC Cut-off frequency
    DccFreq_7,
  //Radio Filter Bandwidth
    RxBw_83300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_125000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_BT_05,
  //CCA threshold
    81,
  //RSSI threshold
    0xCE,
  //Low Beta Afc Offset
    0x06    
  },
  // 920 Operating Mode 2 ARIB
  {
  //channel center frequency 0
    920700000,
  //channel spacing
    200000,
  //total number of channels
    37,
  //Radio bit rate register
    Bitrate_100000,
  //Radio frequency deviation register
    Fdev_50000,
  //DC Cut-off frequency
    DccFreq_7,
  //Radio Filter Bandwidth
    RxBw_166700,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_250000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_BT_05,
  //CCA threshold
    78,
  //RSSI threshold
    0xC8,
  //Low Beta Afc Offset
    0x0D     
  },
  // 920 Operating Mode 3 ARIB
  {
  //channel center frequency 0
    920600000,
  //channel spacing
    200000,
  //total number of channels
    17,
  //Radio bit rate register
    Bitrate_200000,
  //Radio frequency deviation register
    Fdev_100000,
  //DC Cut-off frequency
    DccFreq_7,
  //Radio Filter Bandwidth
    RxBw_333300,
  //DC Cut-off frequency AFC
    DccFreq_7,
  //Radio Filter Bandwidth AFC
    RxBw_400000,
  //Radio modulation type
    DataModul_Modulation_Fsk,
  //Radio modulation shaping
    DataModul_ModulationShaping_BT_05,
  //CCA threshold
    75,
  //RSSI threshold
    0xB8,
  //Low Beta Afc Offset
    0x0D 
  } 
};
#endif

/* aTurnaroundTime 1 ms value expressed in MAC symbols */
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
const uint8_t gTurnaroundTimeTable[] =
{
    50,         /* China Phy Mode 0 */  
};
#elif gFreqBand_863__870MHz_d
const uint8_t gTurnaroundTimeTable[] =
{
    50,         /* Europe 863-870MHz Phy Mode 0 */
    100,        /* Europe 863-870MHz Phy Mode 1 */
};
#elif gFreqBand_902__928MHz_d
const uint8_t gTurnaroundTimeTable[] =
{
    50,         /* US 902-928MHz Phy Mode 0 */
    150,        /* US 902-928MHz Phy Mode 1 */
};
#elif gFreqBand_920__928MHz_d
const uint8_t gTurnaroundTimeTable[] =
{
    50,         /* Japan 920-928MHz Phy Mode 0 */
    100,        /* Japan 920-928MHz Phy Mode 1 */
    0,          /* Not Available */
    0,          /* Not Available */    
    50,         /* Japan 920 Operating Mode 1 ARIB */
    100,        /* Japan 920 Operating Mode 2 ARIB */
    200,        /* Japan 920 Operating Mode 3 ARIB */
};
#elif gFreqBand_865__867MHz_d
const uint8_t gTurnaroundTimeTable[] =
{
    8,          /* India 865-867MHz Phy Mode 0 */
    20,         /* India 865-867MHz Phy Mode 1 */
    100,        /* India 865-867MHz Phy Mode 2 */    
};
#endif

/* Phy Max Channel number */
#if gFreqBand_470__510MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    198,        /* China 470-510MHz Phy Mode 0 */
};
#elif gFreqBand_779__787MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    38,         /* China 779-787MHz Phy Mode 0 */
};
#elif gFreqBand_863__870MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    33,         /* Europe 863-870MHz Phy Mode 0 */
    16,         /* Europe 863-870MHz Phy Mode 1 */
};
#elif gFreqBand_902__928MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    128,         /* US 902-928MHz Phy Mode 0 */
    63,          /* US 902-928MHz Phy Mode 1 */
};
#elif gFreqBand_920__928MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    37,         /* Japan 920-928MHz Phy Mode 0 */
    17,         /* Japan 920-928MHz Phy Mode 1 */
    0,          /* Not Available */
    0,          /* Not Available */    
    37,         /* Japan 920 Operating Mode 1 ARIB */
    36,         /* Japan 920 Operating Mode 2 ARIB */
    16,         /* Japan 920 Operating Mode 3 ARIB */
};
#elif gFreqBand_865__867MHz_d
const uint8_t gPhyMaxChannelNumber[] =
{
    9,          /* India 865-867MHz Phy Mode 0 */
    9,          /* India 865-867MHz Phy Mode 1 */
    9,          /* India 865-867MHz Phy Mode 2 */  
};
#endif

/* MAC Scan channel masks */
#if gFreqBand_470__510MHz_d
const uint32_t gMacScanChannelMask[1][5] =
{
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}, // 0xFFFFFFFF, 0x7F}, /* China 470-510MHz Phy Mode 0 */
};
#elif gFreqBand_779__787MHz_d
const uint32_t gMacScanChannelMask[1][5] =
{
    {0xFFFFFFFF, 0x7F, 0x00, 0x00, 0x00},               /* China 779-787MHz Phy Mode 0 */
};
#elif gFreqBand_863__870MHz_d
const uint32_t gMacScanChannelMask[2][5] =
{
    {0xFFFFFFFF, 0x03, 0x00, 0x00, 0x00},         /* Europe 863-870MHz Phy Mode 0 */
    {0x1FFFF,    0x00, 0x00, 0x00, 0x00},         /* Europe 863-870MHz Phy Mode 1 */ 
};
#elif gFreqBand_902__928MHz_d
const uint32_t gMacScanChannelMask[2][5] =
{
    {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x01}, /* US 902-928MHz Phy Mode 0 */
    {0xFFFFFFFF, 0xFFFFFFFF,       0x00,       0x00, 0x00}, /* US 902-928MHz Phy Mode 1 */
};
#elif gFreqBand_920__928MHz_d
const uint32_t gMacScanChannelMask[7][5] =
{
    {0xFFFFFFFF, 0x3F, 0x00, 0x00, 0x00},         /* Japan 920-928MHz Phy Mode 0 */
    {0x3FFFF,    0x00, 0x00, 0x00, 0x00},         /* Japan 920-928MHz Phy Mode 1 */
    {0x00,       0x00, 0x00, 0x00, 0x00},         /* Not Available */
    {0x00,       0x00, 0x00, 0x00, 0x00},         /* Not Available */    
    {0xFFFFFFFF, 0x3F, 0x00, 0x00, 0x00},         /* Japan 920 Operating Mode 1 ARIB */
    {0xFFFFFEFF, 0x1F, 0x00, 0x00, 0x00},         /* Japan 920 Operating Mode 2 ARIB */
    {0x1FFFF,    0x00, 0x00, 0x00, 0x00},         /* Japan 920 Operating Mode 3 ARIB */
};
#elif gFreqBand_865__867MHz_d
const uint32_t gMacScanChannelMask[3][5] =
{
    {0x3FF, 0x00, 0x00, 0x00, 0x00},              /* India 865-867MHz Phy Mode 0 */
    {0x3FF, 0x00, 0x00, 0x00, 0x00},              /* India 865-867MHz Phy Mode 1 */
    {0x3FF, 0x00, 0x00, 0x00, 0x00},              /* India 865-867MHz Phy Mode 2 */
};
#endif

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
phyPib_t gPhyPib;

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
 * Name: PhyPib_InitOnStartUp()
 * Description: This function performs the PHY PIB initialization: initialize 
 *             the PIB structure with the default values, initialize 
 *             the transceiver registers with the current PHY PIB parameters.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_InitOnStartUp(void)
{  
#if CT_Feature_Calibration
  hardwareParameters_t hwParams;
#endif
  //ST  -- will be initialized in MAC PIB
  gPhyPib.mPIBphyFSKScramblePSDU = TRUE;
  gPhyPib.mPIBphyTransmitPower = 0x1F; // -18dBm + mPIBphyTransmitPower = +13dBm
  gPhyPib.mPIBphyFSKPreambleRepetitions = gPhyFSKPreambleLength_c - 2;
  gPhyPib.mPIBphyCurrentChannel = 0;
  gPhyPib.mPIBphyCurrentMode = gPhyModeDefault_d;
  gPhyPib.mPIBphyCCADuration = gCCADurationDefault_c;   // [symbols]
  gPhyPib.mPIBAfcEnabled = gAfcEnabled_d;
  gPhyPib.mPIBRssiThreshold = gRssiThreshold_c;
  gPhyPib.mPIBphyAckWaitDuration = (gTurnaroundTimeTable[gPhyModeDefault_d] + /* aUnitBackoffPeriod */
                                    gCCADurationDefault_c) +
                                    gTurnaroundTimeTable[gPhyModeDefault_d] + 
                                    gPhySHRDuration_c + 
                                   (9 * gPhySymbolsPerOctet_c); /* [symbols] */
  
#if( gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d || gFreqBand_863__870MHz_d || \
     gFreqBand_902__928MHz_d || gFreqBand_865__867MHz_d )
  gPhyPib.pPIBphyRfConstants = &phyPibRFConstants[gPhyPib.mPIBphyCurrentMode];
#elif gFreqBand_920__928MHz_d
  switch(gPhyPib.mPIBphyCurrentMode)
  {
    case gPhyMode1_c:
    case gPhyMode2_c:
      gPhyPib.pPIBphyRfConstants = &phyPibRFConstants[gPhyPib.mPIBphyCurrentMode];
    break;        
    case gPhyMode1ARIB_c:
    case gPhyMode2ARIB_c:
    case gPhyMode3ARIB_c:
      gPhyPib.pPIBphyRfConstants = &phyPibRFConstantsARIB[gPhyPib.mPIBphyCurrentMode - gPhyMode1ARIB_c];
    break;
  }
#endif

#if CT_Feature_Calibration
  NV_ReadHWParameters(&hwParams);
  if( hwParams.pllFStepOffset == 0xFFFFFFFF )
  {
    gPhyPib.mPIBAdditionalRFFrequencyOffset = 0x00;
  }
  else
  {
    gPhyPib.mPIBAdditionalRFFrequencyOffset = hwParams.pllFStepOffset;
    if( (hwParams.pllFStepOffset >> 24) == 0x01)
    {
      gPhyPib.mPIBAdditionalRFFrequencyOffset |= (0xFF << 24);
    } 
    int32_t signed_value = (int32_t)gPhyPib.mPIBAdditionalRFFrequencyOffset;
    if( (signed_value < gMinOffsetRF_c) || (signed_value > gMaxOffsetRF_c))
    {
      gPhyPib.mPIBAdditionalRFFrequencyOffset = 0x00;
    }
  }
  if(((hwParams.edCalibrationOffset & 0xFFFFFF00) >> 8) == 0xEDEDED)
  {
    gPhyPib.mPIBAdditionalEDOffset = (uint8_t) hwParams.edCalibrationOffset;
    int8_t signed_value = (int8_t)gPhyPib.mPIBAdditionalEDOffset;
    if( (signed_value < gMinOffsetED_c) || (signed_value > gMaxOffsetED_c))
    {
      gPhyPib.mPIBAdditionalEDOffset = 0x00;
    }
  }
  else
  {
    gPhyPib.mPIBAdditionalEDOffset = 0x00;
  }
#endif
  
  PhyPib_RFUpdatePreambleLength();
  PhyPib_RFUpdatePowerStep();  
  
  PhyPib_RFUpdateModulationParameters();
  PhyPib_RFUpdateRFfrequency();
  
  PhyPib_EnableAFC(gPhyPib.mPIBAfcEnabled);
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeGetPIBRequest()
 * Description: This function is used to get the value of the attribute. This function 
 *             performs the following action: checks the input parameters, return
 *             an error exit code if the wrong parameters are supplied, 
 *             otherwise placed the value of the attribute at the location *pAttributeValue
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPlmeGetPIBRequest(phyPibId_t pibId, uint64_t *pibValue, uint8_t phyRegistrySet, instanceId_t instanceId)
{
  phyStatus_t status = gPhySuccess_c;
  if(pibValue == NULL)
  {
    return gPhyInvalidParameter_c;
  }
  switch(pibId)
  {
    case gPhyPibCurrentMode_c:
      *(uint8_t *)pibValue = gPhyPib.mPIBphyCurrentMode;
      break;
    case gPhyPibCurrentChannel_c:
      *(uint8_t *)pibValue = gPhyPib.mPIBphyCurrentChannel;
      break;
    case gPhyPibTransmitPower_c:
      *(uint8_t *)pibValue = gPhyPib.mPIBphyTransmitPower;
      break;
    case gPhyPibFSKPreambleRepetitions_c:
      *(uint16_t *)pibValue = gPhyPib.mPIBphyFSKPreambleRepetitions;
      break;
    case gPhyPibFSKScramblePSDU_c:
      *(uint8_t *)pibValue = gPhyPib.mPIBphyFSKScramblePSDU;
      break;
    case gPhyPibCCADuration_c:
      *(uint16_t *)pibValue = gPhyPib.mPIBphyCCADuration;
      break;
    case gPhyPibFreqBandId_c:
      *(uint8_t *)pibValue = gFreqBandId_d;
      break;
    case gPhyPibLastTxAckFP_c:
      *(uint8_t *)pibValue = gPhyPib.mPIBTxAckFP;
      break;
    default:
      status = gPhyUnsupportedAttribute_c;
      break;
  }
  return status;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetCurrentPhyMode()
 * Description: This function is used to set the current PHY operating mode. 
 *             This function performs the following action: checks the phyMode 
 *             input parameter, return an error exit code if the wrong parameter 
 *             is supplied, otherwise update the PIB structure and Radio registers 
 *             based on phyMode value.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetCurrentPhyMode(phyMode_t phyMode)
{ 
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  
  if( (phyMode == gPhyMode1_c) || 
      (phyMode == gPhyMode2_c) 
#if gFreqBand_865__867MHz_d
    ||(phyMode == gPhyMode3_c)
#endif 
    )
  {
    gPhyPib.pPIBphyRfConstants = (phyRFConstants_t *) &phyPibRFConstants[phyMode];   
  }
#if gFreqBand_920__928MHz_d          // Japan 920-928MHz ARIB custom mode  
  else if((phyMode == gPhyMode1ARIB_c) ||
          (phyMode == gPhyMode2ARIB_c) ||
          (phyMode == gPhyMode3ARIB_c))
  {
    gPhyPib.pPIBphyRfConstants = (phyRFConstants_t *) &phyPibRFConstantsARIB[phyMode - gPhyMode1ARIB_c];
  }  
#endif  
  else
  {
    return gPhyInvalidParameter_c;
  }
  
  gPhyPib.mPIBphyCurrentMode = phyMode;
  gPhyPib.mPIBphyCurrentChannel = 0;
  
  PhyPib_RFUpdateModulationParameters();
  PhyPib_RFUpdateRFfrequency();
  
  PhyPib_EnableAFC(gPhyPib.mPIBAfcEnabled);
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetCurrentChannel()
 * Description: This function is used to set the current logical channel. 
 *              The channel will be used for every transceiver operation 
 *              that requires RF: CCA, ED, RX and TX. This function performs 
 *              the following action: checks the channel input parameter, 
 *              return an error exit code if the wrong parameter is supplied, 
 *              otherwise update the PIB structure and Radio registers based 
 *              on channel value.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetCurrentChannel(uint8_t channel)
{  
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  
  if(channel >= gPhyPib.pPIBphyRfConstants->totalNumChannels) 
  {
    return gPhyInvalidParameter_c;
  } 

  gPhyPib.mPIBphyCurrentChannel = channel;
  
  PhyPib_RFUpdateRFfrequency();

  return gPhySuccess_c;

}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetTransmitPower()
 * Description: This function is used to set the current power step. 
 *             The power step will be used for every transceiver operation that requires 
 *             RF: TX. This function performs the following action: checks the pwrLevel 
 *             input parameter, return an error exit code if the wrong parameter is supplied, 
 *             otherwise update the PIB structure and Radio registers based on pwrLevel value.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetTransmitPower(uint8_t pwrLevel)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  if( pwrLevel > 0x1F )
  {
    return gPhyInvalidParameter_c;
  }
  
  gPhyPib.mPIBphyTransmitPower = pwrLevel;
  
  PhyPib_RFUpdatePowerStep();                 
  
  return gPhySuccess_c;

}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetFSKPreambleRepetitions()
 * Description: This function is used to set the current preamble length. 
 *             The preamble length will be used for every transceiver operation that requires 
 *             RF: TX and RX. This function performs the following action: checks the preambleLength
 *             input parameter, return an error exit code if the wrong parameter is supplied, 
 *             otherwise update the PIB structure and Radio registers based on preambleLength value.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetFSKPreambleRepetitions(uint16_t preambleLength)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  if((preambleLength > gPhyPibMaxPmbLen_c) || (preambleLength < gPhyPibMinPmbLen_c))
  {
    return gPhyInvalidParameter_c;
  }
  gPhyPib.mPIBphyFSKPreambleRepetitions = preambleLength;
  
  PhyPib_RFUpdatePreambleLength();

  return gPhySuccess_c;
  
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetFSKScramblePSDU()
 * Description: This function is used to enable/disable the Radio DC-free 
 *             encoding/decoding mechanism (Data Whitening).
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetFSKScramblePSDU(bool_t scramblePSDU)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  gPhyPib.mPIBphyFSKScramblePSDU = scramblePSDU;   
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetCCADuration()
 * Description: This function is used to set the CCA duration 
 *             time 0 - 1000 symbols for SUN PHYs.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetCCADuration(uint16_t ccaDuration)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }

  gPhyPib.mPIBphyCCADuration = ccaDuration;   
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetAckWaitDuration()
 * Description: This function is used to set the ACK Wait Duration
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetAckWaitDuration(uint16_t ackWaitDuration)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }

  gPhyPib.mPIBphyAckWaitDuration = ackWaitDuration;   
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_SetTschTimeslotInfo()
 * Description: This function is used to set TSCH timings
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_SetTschTimeslotInfo(uint64_t tschTsInfo)
{  
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }

  gPhyPib.mPIBphyTsCcaUs = ((phyTschTsInfo_t*)&tschTsInfo)->tsCCA;
  gPhyPib.mPIBphyTsRxTxUs = ((phyTschTsInfo_t*)&tschTsInfo)->tsRxTx;
  gPhyPib.mPIBphyTsRxAckDelayUs = ((phyTschTsInfo_t*)&tschTsInfo)->tsRxAckDelay;
  gPhyPib.mPIBphyTsAckWaitUs = ((phyTschTsInfo_t*)&tschTsInfo)->tsAckWait;
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_EnableAFC()
 * Description: - This function enables or disables AFC
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPib_EnableAFC(bool_t afcEnabled)
{
  if( gIdle_c != PhyPpGetState() )
  {
    return gPhyBusy_c;
  }
  
  gPhyPib.mPIBAfcEnabled = afcEnabled;
  
  if( afcEnabled )
  {
    Phy_EnableAfcAuto(TRUE);
    Phy_EnableAfcLowBeta(TRUE);
    gPhyPib.mPIBRssiThreshold = gPhyPib.pPIBphyRfConstants->rssiThreshold;
  }
  else
  {
    Phy_EnableAfcAuto(FALSE);
    Phy_EnableAfcLowBeta(FALSE);
    gPhyPib.mPIBRssiThreshold = gRssiThreshold_c;
  }
  
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetPIBRequest()
 * Description: This function is used to set the attribute with the value pointed by pAttributeValue.
 *             This function performs the following action: checks the input parameters, 
 *             return an error exit code if the wrong parameters are supplied, 
  *             otherwise update the PIB structure and Radio registers 
*             based on attribute and *pAttributeValue value.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhyPlmeSetPIBRequest(phyPibId_t pibId, uint64_t pibValue, uint8_t phyRegistrySet, instanceId_t instanceId)
{
  phyStatus_t status = gPhySuccess_c;
  
  (void) phyRegistrySet;
  
  switch(pibId)
  {
    case gPhyPibCurrentMode_c:
        status = PhyPib_SetCurrentPhyMode((phyMode_t)pibValue);
        break;
    case gPhyPibCurrentChannel_c:
        status = PhyPib_SetCurrentChannel((uint8_t)pibValue);
        break;
    case gPhyPibTransmitPower_c:
        status = PhyPib_SetTransmitPower((uint8_t)pibValue);
        break;
    case gPhyPibLongAddress_c:
        {
          uint64_t longAddr = pibValue;
          status = PhyPpSetLongAddr((uint8_t *)&longAddr, 0);
        }
        break;
    case gPhyPibShortAddress_c:
        {
          uint16_t shortAddr = (uint16_t)pibValue;
          status = PhyPpSetShortAddr((uint8_t *)&shortAddr, 0);
        }
        break;
    case gPhyPibPanId_c:
        {
          uint16_t panId = (uint16_t)pibValue;
          status = PhyPpSetPanId((uint8_t *)&panId, 0);
        }
        break;
    case gPhyPibPanCoordinator_c:
        status = PhyPpSetMacRole((bool_t)pibValue, 0);
        break;
    case gPhyPibPromiscuousMode_c:
        PhyPpSetPromiscuous((bool_t)pibValue);
        break;
    case gPhyPibFSKPreambleRepetitions_c:
        status = PhyPib_SetFSKPreambleRepetitions((uint16_t)pibValue);
        break;
    case gPhyPibFSKScramblePSDU_c:
        status = PhyPib_SetFSKScramblePSDU((bool_t)pibValue);
        break;
    case gPhyPibRxOnWhenIdle:
        PhyPlmeSetRxOnWhenIdle((bool_t)pibValue, instanceId);
        break;
    case gPhyPibFrameWaitTime_c:    
        PhyPlmeSetFrameWaitTime((uint32_t)pibValue, instanceId);    
        break;    
    case gPhyPibCCADuration_c:
        status = PhyPib_SetCCADuration((uint16_t)pibValue);
        break;
    case gPhyPibCSLRxEnabled_c:
        PhyPlmeSetCslRxEnabled((bool_t)pibValue, instanceId);
        break;
    case gPhyPibAckWaitDuration_c:
        status = PhyPib_SetAckWaitDuration((uint16_t)pibValue);
        break;
    case gPhyPibFreqBandId_c:
        status = gPhyReadOnly_c;
        break;
    case gPhyPibTschEnabled_c:
        PhyPlmeSetTschEnabled((bool_t)pibValue, instanceId);
        break;
    case gPhyPibTschTimeslotInfo_c:
        status = PhyPib_SetTschTimeslotInfo(pibValue);
        break;
    default:
        status = gPhyUnsupportedAttribute_c;
    break;
  }
  return status;
}

/*****************************************************************************
 *                                PRIVATE FUNCTIONS                          *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdatePreambleLength()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdatePreambleLength(void)
{
  Phy_SetPreambleSize(gPhyPib.mPIBphyFSKPreambleRepetitions);
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdatePowerStep()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdatePowerStep(void)
{
  Phy_SetPowerStep(gPhyPib.mPIBphyTransmitPower);
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdateRFfrequency()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdateRFfrequency(void)
{
  uint32_t channelFreq = (uint32_t)gPhyPib.mPIBphyCurrentChannel;    
  
  // Fstep = 57.220458984375  - 30.0 MHz   CLKOUT
        // 200KHz spacing - 3495.253333333333
        // 400KHz spacing - 6990.506666666667
  
  // Fstep = 61.03515625      - 32.0 MHz   CLKOUT
        // 200KHz spacing - 3276.8
        // 400KHz spacing - 6553.6
  
  // Channel Frequency = (ch value * ch spacing + ch0) * Fstep
#if gFreqBand_470__510MHz_d    
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (470.200 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2); 
    channelFreq += 8217341;
  }
#elif gFreqBand_779__787MHz_d    
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (779.200 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2);
    channelFreq += 13617507;
  }
#elif gFreqBand_863__870MHz_d
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (863.125 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + ((channelFreq * 6) >> 3);
    channelFreq += 14141440;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2_c)
  {
    // Channel 0 (863.225 MHz) - Spacing 400KHz
    channelFreq = channelFreq * 6553 + (channelFreq >> 1);
    channelFreq += 14143079;        
  }
#elif gFreqBand_902__928MHz_d    
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (902.200 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + ((channelFreq * 6) >> 3);
    channelFreq += 14781645;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2_c)
  {
    // Channel 0 (902.400 MHz) - Spacing 400KHz
    channelFreq = channelFreq * 6553 + (channelFreq >> 1);      
    channelFreq += 14784922;
  }
#elif gFreqBand_920__928MHz_d    
#if BOARD_CLOCK_SOURCE_30MHZ  
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (920.600 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2);
    channelFreq += 16088651;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2_c)
  {
    // Channel 0 (920.900 MHz) - Spacing 400KHz
    channelFreq = channelFreq * 6990 + (channelFreq >> 1);    
    channelFreq += 16093894;
  }  
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode1ARIB_c)
  {
    // Channel 0 (920.600 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2); 
    channelFreq += 16088651;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2ARIB_c)
  {
    // Channel 0 (920.700 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2); 
    channelFreq += 16090399;   
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode3ARIB_c)
  {
    // Channel 0 (920.800 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3495 + (channelFreq >> 2); 
    channelFreq += 16092147;        
  }
#else /* BOARD_CLOCK_SOURCE_32MHZ */
  if(gPhyPib.mPIBphyCurrentMode == gPhyMode1_c)
  {
    // Channel 0 (920.600 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + (channelFreq >> 2);
    channelFreq += 15083110;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2_c)
  {
    // Channel 0 (920.900 MHz) - Spacing 400KHz
    channelFreq = channelFreq * 6553 + (channelFreq >> 1);    
    channelFreq += 15088026;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode1ARIB_c)
  {
    // Channel 0 (920.600 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + (channelFreq >> 2); 
    channelFreq += 15083110;
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode2ARIB_c)
  {
    // Channel 0 (920.700 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + (channelFreq >> 2); 
    channelFreq += 15084749;   
  }
  else if(gPhyPib.mPIBphyCurrentMode == gPhyMode3ARIB_c)
  {
    // Channel 0 (920.800 MHz) - Spacing 200KHz
    channelFreq = channelFreq * 3276 + (channelFreq >> 2); 
    channelFreq += 15086387;        
  }
#endif /* BOARD_CLOCK_SOURCE_30MHZ */
#elif gFreqBand_865__867MHz_d
    // Channel 0 (865.750 MHz) - Spacing 200KHz for all PHY modes
    channelFreq = channelFreq * 3276 + ((channelFreq * 6) >> 3);
    channelFreq += 14184448;
#endif
  
  /* Update frequency with offset */
#if CT_Feature_Calibration
  channelFreq += (int32_t)gPhyPib.mPIBAdditionalRFFrequencyOffset;
#else
  channelFreq += gAdditionalRFCarrierFreqOffset_c;
#endif

  Phy_SetCarrierFreq(channelFreq);  
}

/*---------------------------------------------------------------------------
 * Name: PhyPib_RFUpdateModulationParameters()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPib_RFUpdateModulationParameters(void)
{
  Phy_SetBitrate(gPhyPib.pPIBphyRfConstants->bitRateReg);
  Phy_SetFreqDeviation(gPhyPib.pPIBphyRfConstants->fdevReg);
  Phy_SetModulationType(gPhyPib.pPIBphyRfConstants->modulationType);   
  Phy_SetModulationShaping(gPhyPib.pPIBphyRfConstants->modulationShaping);
  Phy_SetDccFreq(gPhyPib.pPIBphyRfConstants->dccFreq);
  Phy_SetRxFilterBw(gPhyPib.pPIBphyRfConstants->rxFilterBw);
  Phy_SetDccFreqAfc(gPhyPib.pPIBphyRfConstants->dccFreqAfc);
  Phy_SetRxFilterBwAfc(gPhyPib.pPIBphyRfConstants->rxFilterBwAfc);
  Phy_SetLowBetaAfcOffset(gPhyPib.pPIBphyRfConstants->lowBetaAfcOffset);
}
