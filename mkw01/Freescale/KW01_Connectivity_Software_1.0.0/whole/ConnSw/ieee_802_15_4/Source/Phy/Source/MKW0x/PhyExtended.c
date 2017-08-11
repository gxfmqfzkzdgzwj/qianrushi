/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyExtended.c
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
#include "SX123xDrv.h"

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
 *                                PRIVATE FUNCTIONS                          *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: Phy_SetInterPacketRxDelay
 * Description: This function sets the inter packet Rx delay
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetInterPacketRxDelay(uint8_t interPacketRxDelay)
{
  uint8_t regPacketConfig2;
	
  if(!((InterPacketRxDelay_0 == interPacketRxDelay) || (InterPacketRxDelay_1 == interPacketRxDelay) || 
       (InterPacketRxDelay_2 == interPacketRxDelay) || (InterPacketRxDelay_3 == interPacketRxDelay) ||
	   (InterPacketRxDelay_4 == interPacketRxDelay ) || ( InterPacketRxDelay_5 == interPacketRxDelay ) || 
  	   (InterPacketRxDelay_6 == interPacketRxDelay ) ||  ( InterPacketRxDelay_7 == interPacketRxDelay ) ||
	   (InterPacketRxDelay_8 == interPacketRxDelay ) || ( InterPacketRxDelay_9 == interPacketRxDelay ) || 
	   (InterPacketRxDelay_A == interPacketRxDelay ) || ( InterPacketRxDelay_B == interPacketRxDelay )))
  {
	return gPhyInvalidParameter_c;
  }
  
  regPacketConfig2 = XCVRDrv_ReadRegister(XCVR_Reg_PacketConfig2);
  regPacketConfig2 &= 0x0F;
  regPacketConfig2 |= interPacketRxDelay;
  
  /* Radio output power initialization @0x11*/  
  XCVRDrv_WriteRegister(XCVR_Reg_PacketConfig2, regPacketConfig2 );
  
  return gPhySuccess_c;  
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetPowerStep
 * Description: This function sets the transciever output power.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetPowerStep(uint8_t outputPower)
{
  uint8_t pwrReg;
  pwrReg = XCVRDrv_ReadRegister(XCVR_Reg_PaLevel);
  pwrReg &= 0xE0;
  pwrReg |= outputPower;
  
  /* Radio output power initialization @0x11*/  
  XCVRDrv_WriteRegister(XCVR_Reg_PaLevel, (uint8_t) (pwrReg) );
}

/*---------------------------------------------------------------------------
 * Name: Phy_PABoostCfg
 * Description: This function configures the PA_BOOST options
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_PABoostCfg (uint8_t u8PaConfig )
{
  uint8_t paLevelReg;
  paLevelReg = XCVRDrv_ReadRegister(XCVR_Reg_PaLevel);
  paLevelReg &= 0x1F;
  paLevelReg |= u8PaConfig;
  
  /* Configure the PA0/PA1/PA2 bits */  
  XCVRDrv_WriteRegister(XCVR_Reg_PaLevel, (uint8_t) (paLevelReg) );
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetOperationMode
 * Description: This function sets the transceiver operation mode.
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOperationMode(uint8_t opMode)
{
  uint8_t opModeRegTemp;

  if(!((OpMode_Sleep == opMode) || (OpMode_StandBy == opMode) || 
       (OpMode_FreqSynt == opMode) || (OpMode_Transmitter == opMode) ||
	   (OpMode_Receiver == opMode || OpMode_Listen_On == opMode ) ))
  {
	return gPhyInvalidParameter_c;
  }
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OpMode);
  if (OpMode_Listen_On == opMode)
  {
    opModeRegTemp &= 0x60;
    opModeRegTemp |= opMode;	
  }
  else 
  {	
    opModeRegTemp &= 0xE3;
	opModeRegTemp |= opMode;
  }		
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, opModeRegTemp );
		
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetOperationModeFast
 * Description: This function sets the transceiver operation mode with the following:
 *              - sequencer enabled (default value)
 *              - listen mode off (default value)
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetOperationModeFast(uint8_t opMode)
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, opMode);
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetCarrierFreq
 * Description: This function sets the carrier frequency through RegFrf, 
 * split accross addresses 0x07 to 0x09.
 *
 * Parameters: Uint32_t carrierFrequency: where carrierFrequency is an hexadecimal 
 * value that represents the carrier frequency divided by the constant 61.03516. 
 * The result of dividing the carrier frequency by that constant, should result
 * in a number that can be represented in 24 bits. This number should be 
 * right-aligned in carrierFrequency parameter of the function.
 *
 * Return: gPhyPibInvalidParameter_c - If the carrier frequency is a number
 *                                       bigger tha 24 bits.
 *         gPhyPibSuccess_c - If operation was succesfully done.
 *---------------------------------------------------------------------------*/
void Phy_SetCarrierFreq(uint32_t carrierFrequency)
{	  
  XCVRDrv_WriteRegister(XCVR_Reg_FrfMsb, (uint8_t) ( carrierFrequency >> 16 ) );
  XCVRDrv_WriteRegister(XCVR_Reg_FrfMid, (uint8_t) ( carrierFrequency >> 8 ) );
  XCVRDrv_WriteRegister(XCVR_Reg_FrfLsb, (uint8_t) ( carrierFrequency ));
}

/*---------------------------------------------------------------------------
  * Description : This function sets the bit rate through RegBitrate, split 
  * accross addresses 0x03 and 0x04.
  *               
  * Assumptions : function adds 1 to the value received before writing to 
  * the registers.  
  *               
  * Inputs      :
  * uint16_t bitrate: where bitrate is an hexadecimal value that represents the 
  * oscillator frequency (32MHz) divided by the desired value of bitrate. 
  * The result of dividing the oscilator frequency by the birtate, should result
  * in a number that can be represented in 2 bytes. This number should be in a 
  * big-endian fashion in bitrate parameter of the function.
  * 
  *  
  * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_SetBitrate(uint16_t bitrate) 
{   
  XCVRDrv_WriteRegister(XCVR_Reg_BitrateMsb, (uint8_t) ( bitrate >> 8 ) );
  XCVRDrv_WriteRegister(XCVR_Reg_BitrateLsb, (uint8_t) ( bitrate ) );
}

/*---------------------------------------------------------------------------
 * Description : This function sets the frequency deviation through RegFdev, 
 * split accross addresses 0x05 and 0x06.
 *               
 * Assumptions : None.
 *               
 * Inputs      : 
 * uint16_t freqDev: where freqDev is an hexadecimal value that represents the 
 * desired frequency deviation (a value between 600Hz and 300KHz) divided by 
 * the constant 61.03516.
 * The result of dividing the frequency deviation by the constant of frequency 
 * step (61.03516), should result in a number that can be represented in 2 bytes. 
 * This number should be in a big-endian fashion in freqDev parameter of the function.
 *  
 * Output      : Void.
 *  
 *---------------------------------------------------------------------------*/
void Phy_SetFreqDeviation(uint16_t freqDev) 
{
  XCVRDrv_WriteRegister(XCVR_Reg_FdevMsb, (uint8_t) ( freqDev >> 8 ) );
  XCVRDrv_WriteRegister(XCVR_Reg_FdevLsb, (uint8_t) ( freqDev ) );
}

/*---------------------------------------------------------------------------
 * Description : This function sets the preamble repetitions in register 0x2C/2D
 *               
 * Assumptions : None.
 *               
 * Inputs      : 
 * uint16_t preambleSize - number of bytes to be generated as preamble
 *  
 * Output      : Void.
 *  
 *---------------------------------------------------------------------------*/
void Phy_SetPreambleSize(uint16_t preambleSize)
{
  XCVRDrv_WriteRegister(XCVR_Reg_PreambleMsb, (uint8_t) ( preambleSize >> 8 ) );
  XCVRDrv_WriteRegister(XCVR_Reg_PreambleLsb, (uint8_t) ( preambleSize ) );
}

/*---------------------------------------------------------------------------
 * Description : This function enables the automatic sequencer.
 *               
 * Assumptions : 
 *               
 * Inputs      : 
 * Uint8_t enableSequencer: This function uses the constants OpMode_Sequencer_Off
 * or OpMode_Sequencer_On in RadioReg.h to write the MSB bit in REgOpMode register 
 * (transceiver address 0x01).
 *  
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the enable sequencer options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_EnableSequencer(uint8_t enableSequencer)  
{
  uint8_t opModeRegTemp;
	
  if( !( (OpMode_Sequencer_Off == enableSequencer) || (OpMode_Sequencer_On == enableSequencer)))
  {
	return gPhyInvalidParameter_c;
  }
	
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OpMode);
  opModeRegTemp &= 0x7F;
  opModeRegTemp |= enableSequencer;
	
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, opModeRegTemp );
	
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the data mode which can be ON-Packet 
 * handler, ON-continous, OFF-continous.
 *               
 * Assumptions : None.
 *               
 * Inputs      :
 * This function receive as the parameter one of the constants for 
 * DataModul_DataMode_x in RadioReg.h:
 *     -	DataModul_DataMode_Packet to set packet mode
 *     -	DataModul_DataMode_Continous to set continuous mode with bit synchronizer
 *     -	DataModul_DataMode_ContinousNoBitSync to set continuous mode without bit 
 *        synchronizer
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the data mode options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDataMode(uint8_t dataMode)  
{
  uint8_t dataModulRegTemp; 
	
  if(!((DataModul_DataMode_Packet == dataMode) || (DataModul_DataMode_Continous == dataMode)||
       (DataModul_DataMode_ContinousNoBitSync == dataMode)))
  {
	return gPhyInvalidParameter_c;
  }
	
  dataModulRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_DataModul);
  dataModulRegTemp &= 0x9F;
  dataModulRegTemp |= dataMode;
		
  XCVRDrv_WriteRegister(XCVR_Reg_DataModul, dataModulRegTemp );	
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the modulation type which can be FSK or OOK.
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants 
 *                for DataModul_Modulation_x in RadioReg.h:
 *                  -	DataModul_Modulation_Fsk to set FSK modulation type.
 *                  -	DataModul_Modulation_Ook to set OOK modulation type.
 *                  
 *              It modifies bits 4 and 3 of the RegDataModul register 
 *              (address 0x02).
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the modulation type options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetModulationType(uint8_t modulationType)  
{
  uint8_t dataModulRegTemp; 
		
  if(!((DataModul_Modulation_Fsk == modulationType) || (DataModul_Modulation_Ook == modulationType)))
  {
	return gPhyInvalidParameter_c;
  }
		
  dataModulRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_DataModul);
  dataModulRegTemp &= 0xE7;
  dataModulRegTemp |= modulationType;
			
  XCVRDrv_WriteRegister(XCVR_Reg_DataModul, dataModulRegTemp );	
		
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the modulation shaping. The modulation 
 *   shaping depends on the selected modulation.
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants 
 *               for DataModul_ModulationShaping_x in RadioReg.h:
 *                  -	DataModul_ModulationShaping_NoShaping for no 
 *                       shaping in modulation.
 *                  -	DataModul_ModulationShaping_ModulationShaping_BT_1 
 *                       for Gaussian filter, BT=1.0 (in case of FSK) or for 
 *                       filtering with fCutOff=BR (in case of OOK modulation).
 *                  -	DataModul_ModulationShaping_BT_05 for Gaussian filter, 
 *                       BT=0.5 (in case of FSK) or for filtering with fCutOff=2*BR 
 *                       (in case of OOK modulation).
 *                  -	DataModul_ModulationShaping_BT_03 for Gaussian filter, 
 *                       BT=0.3 (in case of FSK). For OOK modulation this option 
 *                       is invalid.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the modulation shaping options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetModulationShaping(uint8_t modulationShaping)  
{
  uint8_t dataModulRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_DataModul);
			
  if(DataModul_Modulation_Fsk == ((dataModulRegTemp & 0x18) >> 3))
  {
	if(DataModul_ModulationShaping_BT_03 < modulationShaping)
	{
	  return gPhyInvalidParameter_c;
	}	
  }
	
  if(DataModul_Modulation_Ook == ((dataModulRegTemp & 0x18) >> 3))
  {
	if(DataModul_ModulationShaping_BT_05 < modulationShaping)
	{
	  return gPhyInvalidParameter_c;
	}
  }

  dataModulRegTemp &= 0xFC;
  dataModulRegTemp |= modulationShaping;
				
  XCVRDrv_WriteRegister(XCVR_Reg_DataModul, dataModulRegTemp );	
			
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the power amplifier mode.
 *               
 * Assumptions : None.
 *               
 * Inputs      :  This function receive as the parameter one of the constants 
 *                for PaLevel_x in RadioReg.h:
 *                  -	PaLevel_Pa0_On to choose mode: PA0 output on pin RFIO
 *                  -	PaLevel_Pa1_On to choose mode: PA1 enabled on pin PA_BOOST
 *                  -	PaLevel_Pa2_On to choose mode : PA2 enabled on pin PA_BOOST
 *                  -    PaLevel_Pa1_Pa2_On to choose mode: PA1 and PA2 combined on 
 *                       pin PA_BOOST
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the power amplitude modes options.
 *  gPhyPibSuccess_c - If operation was done successfully. 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetPowerAmpMode(uint8_t paMode)   
{
  uint8_t paLevelRegTemp; 
		
  if(!((PaLevel_Pa0_On == paMode) || (PaLevel_Pa1_Pa2_On==paMode) || 
       (PaLevel_Pa1_On == paMode ) || (PaLevel_Pa2_On == paMode)))
  {
	return gPhyInvalidParameter_c;
  }
	
  paLevelRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_PaLevel);
  paLevelRegTemp &= 0x1F;
  paLevelRegTemp |= paMode;
			
  XCVRDrv_WriteRegister(XCVR_Reg_PaLevel, paLevelRegTemp );	
	
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the rise/fall time of ramp up/down in 
 * 				microseconds. 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				PaRamp_x in RadioReg.h.
 * 				It modifies bits 0 to 3 of the RegPaRamp register (address 0x12).
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Pa ramp options.
 *  gPhyPibSuccess_c - If operation was done successfully. 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetPaRamp(uint8_t paRampValueUs)    
{
  uint8_t paRampRegTemp; 
	
  if (PaRamp_10 < paRampValueUs )
  {
	return gPhyInvalidParameter_c;
  }
  paRampRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_PaRamp);
  paRampRegTemp &= 0xF0;
  paRampRegTemp |= paRampValueUs;
				
  XCVRDrv_WriteRegister(XCVR_Reg_PaRamp, paRampRegTemp );	
	
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the cut-off frequency of the DC offset 
 * 				canceller. 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				DccFreq_x in RadioReg.h.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Dcc Freq options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDccFreq(uint8_t dccFreq)    
{
  uint8_t rxBwRegTemp; 
			
  if(!((DccFreq_0 == dccFreq) || (DccFreq_1 == dccFreq) || (DccFreq_2 == dccFreq) || 
       (DccFreq_3 == dccFreq) || (DccFreq_4 == dccFreq) || (DccFreq_5 == dccFreq) || 
       (DccFreq_6 == dccFreq) || (DccFreq_7 == dccFreq)))
  {
	return gPhyInvalidParameter_c;
  }
		
  rxBwRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_RxBw);
  rxBwRegTemp &= 0x1F;
  rxBwRegTemp |= dccFreq;
				
  XCVRDrv_WriteRegister(XCVR_Reg_RxBw, rxBwRegTemp );	
			
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the values for the channel filter bandwidth 
 * 				control (parameters RxBwMant and RxBwExp).
 * 				Every value is a combination of the RxBwExp with the three possible 
 * 				values for RxBwMant. The result is a value given in kHz and it is 
 * 				different with each modulation type.* 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				RxBw_x in RadioReg.h
 * 
 * Output      :  Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Rx Filter Bw options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetRxFilterBw(uint8_t rxFilterBw)    
{
  uint8_t rxBwRegTemp; 
				
  if(!((RxBw_2600 == rxFilterBw) || (RxBw_3100 == rxFilterBw) || (RxBw_3900 == rxFilterBw) 
	  || (RxBw_5200 == rxFilterBw) || (RxBw_6300 == rxFilterBw) || (RxBw_7800 == rxFilterBw)
	  || (RxBw_10400 == rxFilterBw) || (RxBw_12500 == rxFilterBw) || (RxBw_15600 == rxFilterBw) 
	  || (RxBw_20800 == rxFilterBw) || (RxBw_25000 == rxFilterBw) || (RxBw_31300 == rxFilterBw)
      || (RxBw_41700 == rxFilterBw) || (RxBw_41700 == rxFilterBw) || (RxBw_50000 == rxFilterBw) 
	  || (RxBw_62500 == rxFilterBw) || (RxBw_83300 == rxFilterBw) || (RxBw_100000 == rxFilterBw)
	  || (RxBw_125000 == rxFilterBw) || (RxBw_166700 == rxFilterBw) || (RxBw_200000 == rxFilterBw)
	  || (RxBw_250000 == rxFilterBw) || (RxBw_333300 == rxFilterBw) || (RxBw_400000 == rxFilterBw)
	  || (RxBw_500000 == rxFilterBw)))
  {
	return gPhyInvalidParameter_c;
  }
			
  rxBwRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_RxBw);
  rxBwRegTemp &= 0xE0;
  rxBwRegTemp |= rxFilterBw;
					
  XCVRDrv_WriteRegister(XCVR_Reg_RxBw, rxBwRegTemp );	
				
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : Function similar to Phy_SetDccFreq.
 * 				This function sets the cut-off frequency of the DC offset 
 * 				canceller during AFC. 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				DccFreq_x in RadioReg.h
 * 
 * Output      :  Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Dcc Freq Afc options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *  
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetDccFreqAfc(uint8_t dccFreqAfc)  
{
  uint8_t afcBwRegTemp; 
					
  if(!((DccFreq_0 == dccFreqAfc) || (DccFreq_1 == dccFreqAfc) || (DccFreq_2 == dccFreqAfc) 
	  || (DccFreq_3 == dccFreqAfc) || (DccFreq_4 == dccFreqAfc) || (DccFreq_5 == dccFreqAfc)
	  || (DccFreq_6 == dccFreqAfc) || (DccFreq_7 == dccFreqAfc)))
  {
	return gPhyInvalidParameter_c;
  }
				
  afcBwRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcBw);
  afcBwRegTemp &= 0x1F;
  afcBwRegTemp |= dccFreqAfc;
						
  XCVRDrv_WriteRegister(XCVR_Reg_AfcBw, afcBwRegTemp );	
					
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : Function similar to Phy_SetRxFilterBw.
 *               This function sets the values for the channel filter bandwidth 
 *               control during AFC (parameters RxBwMantAfc and RxBwExpAfc).
 *               Every value is a combination of the RxBwExpAfc with the three 
 *               possible values for RxBwMantAfc. The result is a value given in 
 *               kHz and it is different with each modulation type.
 *                
 * Assumptions : None. 
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				RxBw_x in RadioReg.h.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Rx Filger Bw Afc options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetRxFilterBwAfc(uint8_t rxFilterBwAfc) 
{
  uint8_t afcBwRegTemp; 
					
  if( !((RxBw_2600 == rxFilterBwAfc) || (RxBw_3100 == rxFilterBwAfc) || (RxBw_3900 == rxFilterBwAfc) 
	  || (RxBw_5200 == rxFilterBwAfc) || (RxBw_6300 == rxFilterBwAfc) || (RxBw_7800 == rxFilterBwAfc)
	  || (RxBw_10400 == rxFilterBwAfc) || (RxBw_12500 == rxFilterBwAfc) || (RxBw_15600 == rxFilterBwAfc) 
	  || (RxBw_20800 == rxFilterBwAfc) || (RxBw_25000 == rxFilterBwAfc) || (RxBw_31300 == rxFilterBwAfc)
	  || (RxBw_41700 == rxFilterBwAfc) || (RxBw_41700 == rxFilterBwAfc) || (RxBw_50000 == rxFilterBwAfc) 
	  || (RxBw_62500 == rxFilterBwAfc) || (RxBw_83300 == rxFilterBwAfc) || (RxBw_100000 == rxFilterBwAfc)
	  || (RxBw_125000 == rxFilterBwAfc) || (RxBw_166700 == rxFilterBwAfc) || (RxBw_200000 == rxFilterBwAfc)
	  || (RxBw_250000 == rxFilterBwAfc) || (RxBw_333300 == rxFilterBwAfc) || (RxBw_400000 == rxFilterBwAfc)
	  || (RxBw_500000 == rxFilterBwAfc)))
  {
	return gPhyInvalidParameter_c;
  }
				
  afcBwRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcBw);
  afcBwRegTemp &= 0xE0;
  afcBwRegTemp |= rxFilterBwAfc;
						
  XCVRDrv_WriteRegister(XCVR_Reg_AfcBw, afcBwRegTemp );	
					
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the OOK threshold type, which can be :
 * 				-	Fixed
 * 				-	Peak
 * 				-	Average 
 *               
 * Assumptions :  None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				OokThreshType_x.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Ook Threshold options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokThresholdType(uint8_t thresholdType)  
{
  uint8_t ookPeakRegTemp; 
						
  if(!((OokThreshType_Fixed == thresholdType) || (OokThreshType_Peak == thresholdType) || (OokThreshType_Average == thresholdType)))
  {
	return gPhyInvalidParameter_c;
  }
					
  ookPeakRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OokPeak);
  ookPeakRegTemp &= 0x3F;
  ookPeakRegTemp |= thresholdType;
							
  XCVRDrv_WriteRegister(XCVR_Reg_OokPeak, ookPeakRegTemp );	
						
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the threshold in dB in the OOK demodulator 
 * 				when the threshold type has been set to Fixed. 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter an unsigned value of 
 * 				8 bits that represents a value in dB. The minimum number is 0 
 * 				and max value is 255.
 * 
 * Output      : Void. 
 *---------------------------------------------------------------------------*/
void Phy_SetOokFixedThreshold (uint8_t fixedThresholdDb) 
{
  XCVRDrv_WriteRegister(XCVR_Reg_OokFix, fixedThresholdDb );	
}

/*---------------------------------------------------------------------------
 * Description : This function sets the size of each decrement of the RSSI 
 * 				threshold in the OOK demodulator when threshold type has been 
 * 				set to Peak. 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants 
 * 				for OokThreshStep_x.
 * 
 * Output      :  Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Ook Pek Threshold options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *  
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokPeakThreshStep (uint8_t peakThreshStep) 
{
  uint8_t ookPeakRegTemp; 
							
  if(!((OokPeakThreshStep_0_5dB == peakThreshStep) || (OokPeakThreshStep_1_0dB == peakThreshStep) 
	  || (OokPeakThreshStep_1_5dB == peakThreshStep) || (OokPeakThreshStep_2_0dB == peakThreshStep) 
	  || (OokPeakThreshStep_3_0dB == peakThreshStep) || (OokPeakThreshStep_4_0dB == peakThreshStep)
	  || (OokPeakThreshStep_5_0dB == peakThreshStep) || (OokPeakThreshStep_6_0dB == peakThreshStep)))
  {
	return gPhyInvalidParameter_c;
  }
						
  ookPeakRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OokPeak);
  ookPeakRegTemp &= 0xC7;
  ookPeakRegTemp |= peakThreshStep;
								
  XCVRDrv_WriteRegister(XCVR_Reg_OokPeak, ookPeakRegTemp );	
							
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the period of decrement of the RSSI 
 * 				threshold in the OOK demodulator when threshold type has been 
 * 				set to Peak
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				OokPeakThreshDec_x.
 * 
 * Output      :  Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Ook Pek Threshold Dec options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *  
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokPeakThreshDec (uint8_t peakThreshDec) 
{
  uint8_t ookPeakRegTemp; 
							
  if(!((OokPeakThreshDec_OncePerChip == peakThreshDec) || (OokPeakThreshDec_OnceEveryTwoChips == peakThreshDec) 
	  || (OokPeakThreshDec_OnceEveryFourChips == peakThreshDec) || (OokPeakThreshDec_OnceEveryEightChips == peakThreshDec) 
	  || (OokPeakThreshDec_TwiceEachChip == peakThreshDec) || (OokPeakThreshDec_FourEachChip == peakThreshDec)
	  || (OokPeakThreshDec_EightEachChip == peakThreshDec) || (OokPeakThreshDec_SixteenEachChip == peakThreshDec)))
  {
	return gPhyInvalidParameter_c;
  }
						
  ookPeakRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OokPeak);
  ookPeakRegTemp &= 0xF8;
  ookPeakRegTemp |= peakThreshDec;
								
  XCVRDrv_WriteRegister(XCVR_Reg_OokPeak, ookPeakRegTemp );	
							
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the filter coefficients in average mode of 
 * 				the OOK demodulator.
 *               
 * Assumptions : None.
 *               
 * Inputs      :  This function receive as the parameter one of the constants for 
 * 				 OokPeakThreshFilt_x.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Ook Avg Thres Filt options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetOokAvgThreshFilt (uint8_t avgThresh) 
{
  uint8_t ookAvgRegTemp; 
								
  if( !((OokAvgThreshFilt_48 == avgThresh) || (OokAvgThreshFilt_191 == avgThresh)
	  || (OokAvgThreshFilt_382 == avgThresh) || (OokAvgThreshFilt_764 == avgThresh)))
  {
	return gPhyInvalidParameter_c;
  }
							
  ookAvgRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_OokAvg);
  ookAvgRegTemp &= 0x3F;
  ookAvgRegTemp |= avgThresh;
									
  XCVRDrv_WriteRegister(XCVR_Reg_OokAvg, ookAvgRegTemp );	
								
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the AFC low beta or on off for fading margin 
 * 				improvement.
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter TRUE/FALSE.
 * 				TRUE to enable AfcLowBeta. FALSE otherwise.
 * 
 * Output      : Void
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcLowBeta (bool_t afcLowBetaOn)  
{
  XCVRDrv_WriteRegister(XCVR_Reg_AfcCtrl, (afcLowBetaOn << 5));	
  if(afcLowBetaOn)
  {
	XCVRDrv_WriteRegister(XCVR_Reg_TestDagc, 0x20 );
  }
  else
  {
	XCVRDrv_WriteRegister(XCVR_Reg_TestDagc, 0x30 );
  }
}

/*---------------------------------------------------------------------------
 * Description : This function sets the AFC offset set for low modulation index 
 * 				systems. This function is useful when AFC Low Beta has been enabled.
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as parameter an 8 bit value that, multiplied 
 * 				by 488, gives as a result the desired value in Hertz.
 * 
 * Output      : void 
 *---------------------------------------------------------------------------*/
void Phy_SetLowBetaAfcOffset (uint8_t lowBetaAfcOffset)  
{
  XCVRDrv_WriteRegister(XCVR_Reg_TestAfc, lowBetaAfcOffset );
}

/*---------------------------------------------------------------------------
 * Description : This function sets AFC auto clear on or off.
 * 				If AFC auto clear is on, AFC register is cleared before a new 
 * 				AFC phase, if it is off AFC register is not cleared.* 
 *               
 * Assumptions : The value of this register is only valid if AfcAuto is on.
 *               
 * Inputs      : This function receive as the parameter TRUE/FALSE.
 * 				TRUE to enable AFC auto clear. FALSE otherwise.
 *  
 * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcAutoclear (bool_t afcAutoClearOn)  
{		
  uint8_t afcFeiRegTemp; 
																		
  afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
  afcFeiRegTemp &= 0xF7;
  if(afcAutoClearOn)
  {
	afcFeiRegTemp |= (0x01 << 3);
  }	
											
  XCVRDrv_WriteRegister(XCVR_Reg_AfcFei, afcFeiRegTemp );	
}

/*---------------------------------------------------------------------------
 * Description : This function sets AFC aut on or off.
 * 				If AFC auto is on, AFC is performed each time Rx mode is entered. 
 * 				If AFC auto is off, AFC is performed each time AfcStart is set.* 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter TRUE/FALSE.
 * 				TRUE to enable AFC. FALSE otherwise. 
 * 
 * Output      : Void. 
 *---------------------------------------------------------------------------*/
void Phy_EnableAfcAuto (bool_t afcAutoOn) 
{
  uint8_t afcFeiRegTemp; 
																			
  afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
  afcFeiRegTemp &= 0xFB;
  if(afcAutoOn)
  {
	afcFeiRegTemp |= (afcAutoOn << 2);
  }	
												
  XCVRDrv_WriteRegister(XCVR_Reg_AfcFei, afcFeiRegTemp );	
}

/*---------------------------------------------------------------------------
 * Description : This function clears the AfcValue if set in Rx mode.
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_ClearAfc (void) 
{
  uint8_t afcFeiRegTemp; 
																				
  afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
  afcFeiRegTemp &= 0xFD;
  afcFeiRegTemp |= 0x02;
													
  XCVRDrv_WriteRegister(XCVR_Reg_AfcFei, afcFeiRegTemp );	
}

/*---------------------------------------------------------------------------
 * Description : This function sets the time in ms to generate the timeout 
 * 				interrupt.
 *               
 * Assumptions : None.
 *               
 * Inputs      : timeout value between 0x00 and 0xFF, being 850 ms the maximum 
 * 				timeout allowed.
 * 				Set 0x00 for no timeout (timeoutRxStart disabled).
 *  
 * Output      : Void. 
 *---------------------------------------------------------------------------*/
void Phy_SetRxTimeout (uint8_t timeout) 
{
  XCVRDrv_WriteRegister(XCVR_Reg_RxTimeout1, timeout );
}

/*---------------------------------------------------------------------------
 * Description : This function sets the time in ms to generate the timeout 
 * 				threshold interrupt.
 *               
 * Assumptions : None.
 *               
 * Inputs      : timeout value between 0x00 and 0xFF, being 850 ms the maximum 
 * 				timeout allowed.
 * 
 * Output      : Void. 
 *---------------------------------------------------------------------------*/
void Phy_SetRxTimeoutThreshold (uint8_t timeoutTresh) 
{
  XCVRDrv_WriteRegister(XCVR_Reg_RxTimeout2, timeoutTresh );
}

/*---------------------------------------------------------------------------
 * Description : This function sets the RSSI trigger level for Rssi interrupt.
 *               
 * Assumptions : None.
 *               
 * Inputs      : timeout value between 0x00 and 0xFF.
 * 
 * Output      : Void. 
 *---------------------------------------------------------------------------*/
void Phy_SetRssiThreshold (uint8_t rssiTresh) 
{
  XCVRDrv_WriteRegister(XCVR_Reg_RssiThresh, rssiTresh );
}

/*---------------------------------------------------------------------------
 * Description : This function sets the FifoThreshold value to trigger FifoLevel 
 * 				      interrupt.
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_SetFifoThreshold (uint8_t fifoThresh) 
{
  uint8_t fifoThreshRegTemp; 
																				
  fifoThreshRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_FifoThresh);
  fifoThreshRegTemp &= 0x80;
  fifoThreshRegTemp |= fifoThresh;
													
  XCVRDrv_WriteRegister(XCVR_Reg_FifoThresh, fifoThreshRegTemp);	
}

/*---------------------------------------------------------------------------
 * Description : This function clears the transceiver FIFO
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_ClearFifo(void) 
{
  //while FIFO not empty
  while(XCVRDrv_ReadRegister(XCVR_Reg_IrqFlags2) & 0x40)
  {
    (void)XCVRDrv_ReadRegister(XCVR_Reg_Fifo);
  }
}

/*---------------------------------------------------------------------------
 * Description : This function forces the receiver in WAIT mode, in Continuous 
 * 				Rx mode.
 *               
 * Assumptions : Void.
 *               
 * Inputs      : Void.
 * 
 * Output      : Void.
 *---------------------------------------------------------------------------*/
void Phy_RestartRx (void) 
{
  uint8_t packetConfig2RegTemp; 
																				
  packetConfig2RegTemp = XCVRDrv_ReadRegister(XCVR_Reg_PacketConfig2);
  packetConfig2RegTemp &= 0xFB;
  packetConfig2RegTemp |= 0x04;
													
  XCVRDrv_WriteRegister(XCVR_Reg_PacketConfig2, packetConfig2RegTemp );
}

/*---------------------------------------------------------------------------
 * Description : This function sets LNA’s input impedance, which can be :
 * 					-	50 ohms
 * 					-	200 ohms 
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				Lna_LnaZin_x in RadioReg.h.
 * 
 * Output      :  Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Lna Input options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetLnaInputImpedance (uint8_t lnaZin) 
{
  uint8_t lnaRegTemp; 
									
  if( !((Lna_LnaZin_200 == lnaZin) || (Lna_LnaZin_50 == lnaZin)))
  {
	return gPhyInvalidParameter_c;
  }
								
  lnaRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Lna);
  lnaRegTemp &= 0x7F;
  lnaRegTemp |= lnaZin;
										
  XCVRDrv_WriteRegister(XCVR_Reg_Lna, lnaRegTemp );	
									
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function enables or disables sensitivity boost.
 *               
 * Assumptions : None.
 *               
 * Inputs      : This function receive as the parameter one of the constants for 
 * 				sensitivity boost.
 * 
 * Output      : Returns an error code:
 *  gPhyPibInvalidParameter_c - If the parameter received does not correspond
 *  								to one of the Sensitivity Boost options.
 *  gPhyPibSuccess_c - If operation was done successfully.
 *---------------------------------------------------------------------------*/
uint8_t Phy_EnableSensitivityBoost(uint8_t sensitivityBoostOn) 
{
  if( !((NormalSensitivity == sensitivityBoostOn) || (HighSensitivity == sensitivityBoostOn)))
  {
	return gPhyInvalidParameter_c;
  }
  
  XCVRDrv_WriteRegister(XCVR_Reg_TestLna, sensitivityBoostOn );	
									
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetLnaGain
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetLnaGain(uint8_t gainValue) 
{
  uint8_t lnaRegTemp;
  lnaRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Lna);
  lnaRegTemp &= 0xF8;			
  lnaRegTemp |= gainValue;	
  XCVRDrv_WriteRegister(XCVR_Reg_Lna, lnaRegTemp );		
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetSyncWordSize
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetSyncWordSize(uint8_t syncWordSize) 
{
  uint8_t syncConfigRegTemp;

  if( !((SyncConfig_SyncSize_1 == syncWordSize) || (SyncConfig_SyncSize_2 == syncWordSize) 
		|| (SyncConfig_SyncSize_3 == syncWordSize) || (SyncConfig_SyncSize_4 == syncWordSize)
		||(SyncConfig_SyncSize_5 == syncWordSize) || (SyncConfig_SyncSize_6 == syncWordSize)
		|| (SyncConfig_SyncSize_7 == syncWordSize) || (SyncConfig_SyncSize_8 == syncWordSize)))
  {
	return gPhyInvalidParameter_c;
  }
	
  syncConfigRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_SyncConfig);
  syncConfigRegTemp &= 0xC7;
  syncConfigRegTemp |= syncWordSize;
											
  XCVRDrv_WriteRegister(XCVR_Reg_SyncConfig, syncConfigRegTemp );	
  return gPhySuccess_c;	
}

/*---------------------------------------------------------------------------
 * Description :
 *               
 * Assumptions : 
 *               
 * Inputs      : 
 * 	uint8_t syncWordValueReg: It must be the next values
 *  
 *  uint8_t syncValue: It is the value to be stored in the SyncValue register.                
 * 
 * Output      : 
 * Errors      : 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetSyncWordValue(uint8_t syncWordValueReg, uint8_t syncValue) 
{	
  if( !((XCVR_Reg_SyncValue1 == syncWordValueReg) || (XCVR_Reg_SyncValue2 == syncWordValueReg) 
		|| (XCVR_Reg_SyncValue3 == syncWordValueReg) || (XCVR_Reg_SyncValue4 == syncWordValueReg)
		||(XCVR_Reg_SyncValue5 == syncWordValueReg) || (XCVR_Reg_SyncValue6 == syncWordValueReg)
		|| (XCVR_Reg_SyncValue7 == syncWordValueReg) || (XCVR_Reg_SyncValue8 == syncWordValueReg)))
  {
	return gPhyInvalidParameter_c;
  }
											
  XCVRDrv_WriteRegister(syncWordValueReg, syncValue);	
  return gPhySuccess_c;	
}

/*---------------------------------------------------------------------------
 * Description : This function returns a 16-bit value that represents the AFC 
 * 				value in 2’s complement format.
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      :  16-bit value that represents the AFC value in 2’s complement format.
 *---------------------------------------------------------------------------*/
uint16_t Phy_GetAfc(void) 
{
  uint16_t afcValue;
  uint8_t afcFeiRegTemp;
	
  afcValue = 0x0000;
	
  afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
  afcFeiRegTemp &= 0x04;
  if(!afcFeiRegTemp)
  {
	afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
	afcFeiRegTemp &= 0xFE;
	afcFeiRegTemp |= 0x01;
	XCVRDrv_WriteRegister(XCVR_Reg_AfcFei, afcFeiRegTemp);
			
	while(!(XCVRDrv_ReadRegister(XCVR_Reg_AfcFei) && 0x10));	
  }	
	
  afcValue |= ((uint16_t)XCVRDrv_ReadRegister(XCVR_Reg_AfcMsb) << 8);	
  afcValue |= (uint16_t)XCVRDrv_ReadRegister(XCVR_Reg_AfcLsb);
										
  return afcValue;
}

/*---------------------------------------------------------------------------
 * Description : This function returns a 16-bit value that represents the FEI 
 * 				value in 2’s complement format.
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      :  16-bit value that represents the FEI value in 2’s complement 
 * 				format.
 *---------------------------------------------------------------------------*/
uint16_t Phy_GetFei(void) 
{
  uint16_t feiValue;
  uint8_t afcFeiRegTemp;
	
  afcFeiRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_AfcFei);
  afcFeiRegTemp &= 0xDF;
  afcFeiRegTemp |= 0x20;
  XCVRDrv_WriteRegister(XCVR_Reg_AfcFei, afcFeiRegTemp );	
  while(!(XCVRDrv_ReadRegister(XCVR_Reg_AfcFei) && 0x40));
	
  feiValue = 0x0000;
  feiValue |= ((uint16_t)XCVRDrv_ReadRegister(XCVR_Reg_FeiMsb) << 8);	
  feiValue |= (uint16_t)XCVRDrv_ReadRegister(XCVR_Reg_FeiLsb);
										
  return feiValue;
}

/*---------------------------------------------------------------------------
 * Description : This function returns an 8-bit value that represents the RSSI 
 * 				value. The value returned can be converted to dBm with the formula:
 * 					RSSI = -RssiValue/2* 
 *               
 * Assumptions : None.
 *               
 * Inputs      : Void.
 * 
 * Output      :  8-bit value that represents the RSSI value.
 *---------------------------------------------------------------------------*/
uint8_t Phy_GetRssi(void) 
{
  uint8_t rssiValue = 0x00;
  			  
//  if( 0x00 == XCVRDrv_ReadRegister(XCVR_Reg_TestDagc) )
//  {
//    XCVRDrv_WriteRegister(XCVR_Reg_RssiConfig, 0x01 );
//    while(!XCVRDrv_ReadRegister(XCVR_Reg_RssiConfig));  
//  }
  rssiValue = XCVRDrv_ReadRegister(XCVR_Reg_RssiValue);
										
  return rssiValue;
}
/*---------------------------------------------------------------------------
 * Description : This function sets the Listen End bits in RegListen1
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * uint8_t endCondition: accepted values are 0x00, 0x02, 0x04;
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If operating mode was changed successfully.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenEndCriteria (uint8_t endCondition)
{
  uint8_t opModeRegTemp;
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Listen1);
  opModeRegTemp &= 0xF9;
  opModeRegTemp |= endCondition;
  XCVRDrv_WriteRegister(XCVR_Reg_Listen1, opModeRegTemp );		
  return gPhySuccess_c;
}
/*---------------------------------------------------------------------------
 * Description : This function sets the Listen criteria bit in RegListen1
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * bool_t threshWithSyncMatch: TRUE means signal above threshold and sync
 *                             address match. FALSE means no sync address match
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If operating mode was changed successfully.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCriteria (bool_t threshWithSyncMatch)
{
  uint8_t opModeRegTemp;
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Listen1);
  opModeRegTemp &= 0xF7;
  if(threshWithSyncMatch)
  {
    opModeRegTemp |= 0x08;
  }
  XCVRDrv_WriteRegister(XCVR_Reg_Listen1, opModeRegTemp );		
  return gPhySuccess_c;
}
/*---------------------------------------------------------------------------
 * Description : This function sets the Listen resolution RX bits in RegListen1
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * Uint8_t listenResolution: the accepted values are 0x10, 0x20, 0x30
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If operating mode was changed successfully.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenResRx (uint8_t listenResolution)
{
  uint8_t opModeRegTemp;
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Listen1);
  opModeRegTemp &= 0xCF;
  opModeRegTemp |= listenResolution;		
  XCVRDrv_WriteRegister(XCVR_Reg_Listen1, opModeRegTemp );		
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the Listen Coefl Rx in RegListen3 register
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * Uint8_t listenCoef: This value ranges from 0 to 255 
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If Listen Coef was changed.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCoefRx (uint8_t listenCoef)
{	
  XCVRDrv_WriteRegister(XCVR_Reg_Listen3, listenCoef);		
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Description : This function sets the Listen resolution Idle bits in RegListen1
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * Uint8_t listenResolution: the accepted values are 0x40, 0x80, 0xC0
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If operating mode was changed successfully.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenResIdle (uint8_t listenResolution)
{
  uint8_t opModeRegTemp;
  opModeRegTemp = XCVRDrv_ReadRegister(XCVR_Reg_Listen1);
  opModeRegTemp &= 0x3F;
  opModeRegTemp |= listenResolution;		
  XCVRDrv_WriteRegister(XCVR_Reg_Listen1, opModeRegTemp );		
  return gPhySuccess_c;
}
/*---------------------------------------------------------------------------
 * Description : This function sets the Listen Coefl Idle in RegListen2 register
 *               
 * Assumptions : None
 *               
 * Inputs      : 
 * Uint8_t listenCoef: This value ranges from 0 to 255 
 * 
 * Output      : Returns an error code:
 *  gPhyPibSuccess_c - If Listen Coef was changed.
 * 
 *---------------------------------------------------------------------------*/
uint8_t Phy_SetListenCoefIdle (uint8_t listenCoef)
{	
  XCVRDrv_WriteRegister(XCVR_Reg_Listen2, listenCoef);		
  return gPhySuccess_c;
}
