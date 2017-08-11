/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file App_Custom.c
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "Connectivity_Test_Platform.h"

/************************************************************************************
*************************************************************************************
* Macros
*************************************************************************************
************************************************************************************/
#define gCTSelf_EVENT_c (1<<7)
#define SelfNotificationEvent() (OSA_EventSet(&gTaskEvent, gCTSelf_EVENT_c));

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
uint8_t crtBitrate;
SelectBitrateStates_t bsState;
bool_t bUsingCCA128us;

#if CT_Feature_Afc
bool_t bEnableAfc;
bool_t bEnableAfcLowBeta;
bool_t bEnableAfcAutoClear;
#endif

#if CT_Feature_Calibration
EDCalibrationMeasurement_t edCalState;
int32_t gOffsetIncrement = gMinAdditionalRFOffset_c;
#endif

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
registerLimits_t registerIntervals[] = {
                                        {.regStart=0x00 , .regEnd=0x72, TRUE},
					{.regStart=0x00 , .regEnd=0x00, FALSE}
                                       };

/************************************************************************************
*************************************************************************************
* Private function prototypes
*************************************************************************************
************************************************************************************/
static void AppSetCCADuration(uint32_t u32CCADurationUS);



/************************************************************************************
*
* InitApp_custom
*
************************************************************************************/
void InitApp_custom()
{
 (void)MLMESetPhyMode(gDefaultMode1_c);

#if CT_Feature_Calibration
  EnableFlashCalibrationFeature();
 (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
  crtBitrate = gMode1Bitrate_c;
  bUsingCCA128us = TRUE;
  AppSetCCADuration(128);  
#if CT_Feature_Afc
  Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
  (void)Asp_DisableAfc();
#endif
}

/************************************************************************************
*
* InitProject_custom
*
************************************************************************************/
void InitProject_custom()
{
#if CT_Feature_Afc
  bEnableAfc       = FALSE;
  bEnableAfcAutoClear = FALSE;
  bEnableAfcLowBeta = FALSE;
#endif
#if CT_Feature_Calibration
  gOffsetIncrement = (int32_t)MLMEGetAdditionalRFOffset();
#endif
}

/************************************************************************************
*
* EDCalibrationMeasurement
*
* Handler for the Energy Detect Calibration Feature.
*
************************************************************************************/
#if CT_Feature_Calibration
bool_t EDCalibrationMeasurement(void)
{
  bool_t   bBackFlag = FALSE;
  static uint32_t  u32InputSignal = 0;
  static uint8_t   u8MeasIndex = 0;
  static uint32_t  u32EDSum = 0;

  switch(edCalState)
  {
  case gEdCalStateInit_c:
    Serial_Print(mAppSer, "\r\n\r\n\r\n\r\nWelcome to ED Calibration. Connect your signal source to the board\r\n\r\n", gAllowToBlock_d);
    Serial_Print(mAppSer, "\r\n  Press [ENTER] to begin calibration", gAllowToBlock_d);
    Serial_Print(mAppSer, "\r\n  Press [p] if you want to exit this test\r\n", gAllowToBlock_d);
    u32InputSignal = 0;
    u8MeasIndex    = 0;
    u32EDSum       = 0;
    edCalState = gEdCalStatePrepare_c;
    shortCutsEnabled = FALSE;
    break;
  case gEdCalStatePrepare_c:
    if(evDataFromUART)
    {
      if('\r' == gu8UartData)
      {
        Serial_Print(mAppSer, "\r\n\r\nResetting previous calibration value (writting 0xFFFFFFFF)\r\n", gAllowToBlock_d);
        StoreTrimValueToFlash(0xFFFFFFFF, gUpdateEDCalibration);
        (void)MLMESetAdditionalEDOffset(0x00);
        Serial_Print(mAppSer, "\r\nPlease type the power of the modulated input "
                                "signal (between -1 and -130 dBm) and press [ENTER]\r\n", gAllowToBlock_d);
        Serial_Print(mAppSer, "-",gAllowToBlock_d);
        edCalState = gEdCalStateWaitInput_c;
      }
      else if('p' == gu8UartData)
      {
        bBackFlag = TRUE;
        edCalState = gEdCalStateInit_c;
      }
      evDataFromUART = FALSE;
    }
  case gEdCalStateWaitInput_c:
    if(evDataFromUART)
    {
      if('\r' == gu8UartData && 0 != u32InputSignal)
      {
        edCalState = gEdCalStateMeasure_c;
        (void)MLMEScanRequest(testChannel);
        u8MeasIndex++;
      }
      else if('0' <= gu8UartData && '9' >= gu8UartData)
      {
        u32InputSignal = (u32InputSignal*10) + (gu8UartData - '0');
        Serial_PrintDec(mAppSer, (uint32_t)(gu8UartData - '0'));
        if(u32InputSignal > 130)
        {
          Serial_Print(mAppSer,"\r\n Error: Input value too big. [-1,-130] is the interval\r\n",gAllowToBlock_d);
          edCalState = gEdCalStateInit_c;
          SelfNotificationEvent();
        }
      }
      evDataFromUART = FALSE;
    }
    break;
  case gEdCalStateMeasure_c:
    if(bEdDone)
    {
      u32EDSum += au8ScanResults[testChannel];
      bEdDone   = FALSE;
      u8MeasIndex++;
      if(u8MeasIndex < 128)
      {
        (void)MLMEScanRequest(testChannel);
      }
      else
      {
        u32EDSum >>= 7;
        u32EDSum   = (int32_t)(u32InputSignal - u32EDSum);
        
        (void)MLMESetAdditionalEDOffset((uint8_t)(u32EDSum & 0x000000FF));
        
        u32EDSum   = (u32EDSum & 0x000000FF) | 0xEDEDED00;
        edCalState = gEdCalStateStoreToFlash_c;
        SelfNotificationEvent();
      }
    }
    break;
  case gEdCalStateStoreToFlash_c:
    StoreTrimValueToFlash(u32EDSum, gUpdateEDCalibration);
    Serial_Print(mAppSer, "\r\nTest done. ED calibrated. Press [ENTER] to continue.\r\n", gAllowToBlock_d);
    edCalState = gEdCalStateEnd_c;
    break;
  case gEdCalStateEnd_c:
    if(evDataFromUART && '\r' == gu8UartData)
    {
      bBackFlag        = TRUE;
      shortCutsEnabled = TRUE;
      edCalState       = gEdCalStateInit_c;
      evDataFromUART = FALSE;
    }
    break;
  default:
    break;
  }
  return bBackFlag;
}
#endif
/************************************************************************************
*
* Bitrate_Select
*
* Select RF Mode between RF1 and RF2 (bitrate is also changed)
*
************************************************************************************/
bool_t Bitrate_Select(void)
{
  bool_t bBackFlag = FALSE;
  smacErrors_t err = gErrorNoError_c;
  if(evTestParameters)
  {
#if CT_Feature_Calibration
    (void)MLMESetAdditionalRFOffset(gOffsetIncrement);
#endif
    (void)MLMESetChannelRequest(testChannel);                                
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  switch(bsState)
  {
  case gBSStateInit_c:
    Serial_Print(mAppSer,"\r\n", gAllowToBlock_d);
    PrintMenu(cu8RadioBitrateSelectMenu, mAppSer);
    PrintTestParameters(FALSE);
    shortCutsEnabled = TRUE;      
    bsState = gBSStateModify_c;
    break;
  case gBSStateModify_c:
    if(evDataFromUART && gu8UartData == '1')
    {
      err = MLMESetPhyMode(gDefaultMode1_c);
      //when setting phy mode, if afc is enabled, low beta and autoclear are
      //configured automatically to defaults, so we need to restore user setting
      if(gErrorNoError_c == err)
      {
#if CT_Feature_Afc
        if(bEnableAfc)
        {
          //enables afc with low beta and autoclear configured by user
          //if afc is already enabled, this function just configures low beta and autoclear
          Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
        }
#endif
        crtBitrate = gMode1Bitrate_c;
        testChannel = gChannel0_c;
        Serial_Print(mAppSer, "\r\n\r\n -Bitrate set to ", gAllowToBlock_d);
        Serial_PrintDec(mAppSer, (uint32_t)crtBitrate);
        Serial_Print(mAppSer, "kbps!", gAllowToBlock_d);
      }
      else
      {
        Serial_Print(mAppSer, "\r\n\r\n - SMAC can't perform this right now!\r\n",gAllowToBlock_d);
      }
    }
    else if(evDataFromUART && gu8UartData == '2')
    { 
      err = MLMESetPhyMode(gDefaultMode2_c);
      
      if(gErrorNoError_c == err)
      {
#if CT_Feature_Afc
        if(bEnableAfc)
        {
          //enables afc with low beta and autoclear configured by user
          //if afc is already enabled, this function just configures low beta and autoclear
          Asp_EnableAfc(bEnableAfcLowBeta,bEnableAfcAutoClear);
        }
#endif
        crtBitrate = gMode2Bitrate_c;
        testChannel = gChannel0_c;
        Serial_Print(mAppSer, "\r\n\r\n -Bitrate set to ", gAllowToBlock_d);
        Serial_PrintDec(mAppSer, (uint32_t)crtBitrate);
        Serial_Print(mAppSer, "kbps!", gAllowToBlock_d);
      }
      else
      {
        Serial_Print(mAppSer, "\r\n\r\n - SMAC can't perform this right now!\r\n",gAllowToBlock_d);
      }
    }
    else if(evDataFromUART && gu8UartData == 'p')
    {
      bBackFlag = TRUE;
      bsState = gBSStateInit_c;
      SelfNotificationEvent();
    }
    evDataFromUART = FALSE;
    break;
  default:
    break;
  }
  return bBackFlag;  
}

/************************************************************************************
*
* AppSetCCADuration
*
************************************************************************************/
static void AppSetCCADuration(uint32_t u32CCADurationUS)
{
    MLMESetCCADuration(u32CCADurationUS);
}

/************************************************************************************
*
* PrintTestParameters
*
************************************************************************************/
void PrintTestParameters(bool_t bEraseLine)
{
  uint8_t u8Index1;
  uint8_t u8Index2;
  if(bEraseLine)
  {
    //goes back X lines ([XA) and to line start : works with any ANSI C compliant terminal
#if CT_Feature_Afc
    Serial_Print(mAppSer,"\033[2A",gAllowToBlock_d);
    Serial_Print(mAppSer,"\r",gAllowToBlock_d);
    for(u8Index1 = 0; u8Index1 < 2; u8Index1++)
#else
    Serial_Print(mAppSer,"\033[1A",gAllowToBlock_d);
    Serial_Print(mAppSer,"\r",gAllowToBlock_d);
    for(u8Index1 = 0; u8Index1 < 1; u8Index1++)  
#endif
    {
      for(u8Index2 = 0; u8Index2 < 60; u8Index2++)
      {
        Serial_Print(mAppSer," ",gAllowToBlock_d);
      }
      Serial_Print(mAppSer,"\n\r",gAllowToBlock_d);
    }
    for(u8Index2 = 0; u8Index2 < 60; u8Index2++)
    {
        Serial_Print(mAppSer," ",gAllowToBlock_d);
    }
#if CT_Feature_Afc
    Serial_Print(mAppSer,"\b\033[2A",gAllowToBlock_d);
#else
    Serial_Print(mAppSer,"\b\033[1A",gAllowToBlock_d);
#endif
    Serial_Print(mAppSer,"\r",gAllowToBlock_d);
  }
  
  Serial_Print(mAppSer, "Mode ", gAllowToBlock_d);
  if(mTxOperation_c == testOpMode)
  {
    Serial_Print(mAppSer, "Tx", gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer, "Rx", gAllowToBlock_d);
  }
  Serial_Print(mAppSer, ", Channel ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer, (uint32_t)testChannel);
  Serial_Print(mAppSer,", Power ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer,(uint32_t)testPower);
  Serial_Print(mAppSer,", Payload ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer, (uint32_t)testPayloadLen);
  Serial_Print(mAppSer,", Bitrate ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer, (uint32_t)crtBitrate);
  Serial_Print(mAppSer,"kbps",gAllowToBlock_d);
#if CT_Feature_Calibration
  Serial_Print(mAppSer,",\r\n FrOff ", gAllowToBlock_d);
  if(gOffsetIncrement >= 0) 
  {
    Serial_PrintDec(mAppSer, (uint32_t)gOffsetIncrement);
  }
  else
  {
    Serial_Print(mAppSer,"-", gAllowToBlock_d);
    Serial_PrintDec(mAppSer, (uint32_t)(-1*gOffsetIncrement));
  }
  Serial_Print(mAppSer,"Fsteps",gAllowToBlock_d);
#endif
  Serial_Print(mAppSer,", CCA Thresh ", gAllowToBlock_d);
  if(ccaThresh != 0)
  {
    Serial_Print(mAppSer,"-",gAllowToBlock_d);
  }
  Serial_PrintDec(mAppSer, (uint32_t)ccaThresh);
  Serial_Print(mAppSer,"dBm",gAllowToBlock_d);
  if(bUsingCCA128us)
  {
    Serial_Print(mAppSer,", CCA Duration 128us",gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer,", CCA Duration 5ms", gAllowToBlock_d);
  }
#if CT_Feature_Afc
  if(bEnableAfc)
  {
    Serial_Print(mAppSer,",  \r\n Afc On ",gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer,"\r\n Afc Off", gAllowToBlock_d);
  }
  if(bEnableAfcLowBeta)
  {
    Serial_Print(mAppSer,", Afc LowB On ", gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer,", Afc LowB Off", gAllowToBlock_d);
  }
  if(bEnableAfcAutoClear)
  {
    Serial_Print(mAppSer,", Afc AClr On ", gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer,", Afc AClr Off", gAllowToBlock_d);
  }
#endif
  Serial_Print(mAppSer," >", gAllowToBlock_d);
}

/************************************************************************************
*
* ShortCutsParser
*
* Performs changes in different menus whenever shortcuts are allowed
*
************************************************************************************/
void ShortCutsParser(uint8_t u8UartData)
{
  evTestParameters = TRUE;
  evDataFromUART = FALSE;
  switch(u8UartData){
  case 't':
    testOpMode = mTxOperation_c;
    break;
  case 'r':
    testOpMode = mRxOperation_c;
    break;
  case 'q': 
    testChannel++;
    if(testChannel == gChannel8_c && crtBitrate == gMode2Bitrate_c && gDefaultMode2_c == gRFMode6_c) 
    { //case ARIB2 is used as default mode 2 then skip channel 8
      testChannel++;
    }
    if(gTotalChannels <= testChannel)
    {
      testChannel = gChannel0_c;
    }
    break;
  case 'w':
    testChannel--;
    if(testChannel == gChannel8_c && crtBitrate == gMode2Bitrate_c && gDefaultMode2_c == gRFMode6_c) 
    {  //case ARIB2 is used as default mode 2 then skip channel 8
      testChannel--;
    }
    if(testChannel >= gTotalChannels){       
      testChannel = (channels_t)(gTotalChannels - 1);
    }
    break;
  case 'a':
    testPower++;
    if(gMaxOutputPower_c < testPower){
      testPower = gMinOutputPower_c;
    }
    break;
  case 's':
    if(testPower == gMinOutputPower_c)
      testPower = gMaxOutputPower_c;
    else
      testPower--;
    break;
  case 'n':
    testPayloadLen++;
    if(gMaxSmacSDULength_c < testPayloadLen){
      testPayloadLen = 17;
    }    
    break;
  case 'm':
    testPayloadLen--;
    if(17 > testPayloadLen){
      testPayloadLen = gMaxSmacSDULength_c;
    }    
    break;
#if CT_Feature_Calibration
  case 'g':
    gOffsetIncrement++;
    if(gMaxAdditionalRFOffset_c < gOffsetIncrement) {
      gOffsetIncrement = gMinAdditionalRFOffset_c;
    }
    break;
  case 'h':
    gOffsetIncrement--;
    if(gMinAdditionalRFOffset_c > gOffsetIncrement){
      gOffsetIncrement = gMaxAdditionalRFOffset_c;
    }
    break;
  case 'j':
    StoreTrimValueToFlash(gOffsetIncrement, gUpdateRFOffset);
    break;
#endif
  case 'k':
    ccaThresh++;
    if(ccaThresh > gMaxCCAThreshold_c)
      ccaThresh = gMinCCAThreshold_c;
    break;
  case 'l':
    ccaThresh--;
    if(ccaThresh > gMaxCCAThreshold_c)
      ccaThresh = gMaxCCAThreshold_c;
    break;
  case 'b':
    bUsingCCA128us = !bUsingCCA128us;
    if(bUsingCCA128us)
      AppSetCCADuration(128);
    else
      AppSetCCADuration(5000);
    break;
#if CT_Feature_Afc
  case 'z':
    bEnableAfc = !bEnableAfc;
    if(bEnableAfc)
    {
      bEnableAfcAutoClear = TRUE;
      bEnableAfcLowBeta = TRUE;
      Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
    }
    else
    {
      bEnableAfcAutoClear = FALSE;
      bEnableAfcLowBeta   = FALSE;
      //first disable low beta and auto-clear
      Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
      Asp_DisableAfc();
    }
    break;
  case 'x':
    if(bEnableAfc)
    {
      bEnableAfcLowBeta = !bEnableAfcLowBeta;
      Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
    }
    break;
  case 'c':
    if(bEnableAfc)
    {
      bEnableAfcAutoClear = !bEnableAfcAutoClear;
      Asp_EnableAfc(bEnableAfcLowBeta, bEnableAfcAutoClear);
    }
    break;
#endif
  default:
    evDataFromUART = TRUE;
    evTestParameters = FALSE;
    break;
  }
}

/************************************************************************************
*
* EnableFlashCalibrationFeature
*
************************************************************************************/
#if CT_Feature_Calibration
void EnableFlashCalibrationFeature()
{
  NV_Init();
}

/************************************************************************************
*
* StoreTrimValueToFlash
*
* Stores Calibration Value depending on type (ED measurement calibration or 
* frequency calibration
*
************************************************************************************/
void StoreTrimValueToFlash (uint32_t value, CalibrationOptionSelect_t option)
{
  uint32_t retCode;
  hardwareParameters_t hwParams;
  /*read existing parameters*/
  NV_ReadHWParameters(&hwParams);
  
  if(gUpdateRFOffset == option)
  {
    if((int32_t)(value) >= 0)
    {
      hwParams.pllFStepOffset = (0x00FFFFFF & value);
    }
    else
    {
      hwParams.pllFStepOffset = (0x00FFFFFF & value) | 0x01000000;
    }
  }
  else if(gUpdateEDCalibration == option)
  {
    hwParams.edCalibrationOffset = value;
  }
  else
  {
    return;
  }
  
  OSA_EnterCritical(kCriticalDisableInt);
  retCode = NV_WriteHWParameters(&hwParams);
  OSA_ExitCritical(kCriticalDisableInt);
  
  if(retCode != FTFx_OK) {
    Serial_Print(mAppSer,"\r\n\r\n Failed to write parameters...r\n\r\n\r\n\r\n", gAllowToBlock_d);
  }
  Serial_Print(mAppSer,"\r\n\r\n Value Stored...\r\n\r\n\r\n\r\n", gAllowToBlock_d);
  return;
}
#endif