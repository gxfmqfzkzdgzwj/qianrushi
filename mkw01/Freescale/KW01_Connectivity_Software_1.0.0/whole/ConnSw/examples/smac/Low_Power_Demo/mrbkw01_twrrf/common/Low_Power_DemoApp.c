/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file Low_Power_DemoApp.c
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

#include "Application_Interface.h"
#include "SMAC_Config.h"
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define SelfNotificationEvent() ((void)OSA_EventSet(&gTaskEvent, gLDPSelf_EVENT_c))
#define OpModeRX                (1)
#define OpModeTX                (2)
/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
osa_status_t 	      myAppStatus;
event_t               gTaskEvent;
event_flags_t         gTaskEventFlags;
                                              
channels_t        testChannel;
uint8_t           testPower;

LPDStates_t       lpdState;
LPDManualStates_t lpdmState;
                                                                                           
bool_t evDataFromUART;
bool_t evTestParameters;
uint8_t gu8UartData;

bool_t shortCutsEnabled;

extern const clock_manager_user_config_t g_defaultClockConfigVlpr;
extern const clock_manager_user_config_t g_defaultClockConfigRun;
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint8_t gau8RxDataBuffer[gMaxSmacSDULength_c  + sizeof(txPacket_t)];                         
static uint8_t gau8TxDataBuffer[gMaxSmacSDULength_c  + sizeof(rxPacket_t)];                        

static txPacket_t * gAppTxPacket;
static rxPacket_t * gAppRxPacket;

static uint8_t mAppSer;

static bool_t  evKeyPressed;
static bool_t  bTxDone;
static bool_t  bRxDone;
static uint8_t AppDelayTmr;
static uint8_t timePassed;
static uint8_t lpd_listen_config;
static bool_t  bPrevModeIsVlpr;
static bool_t  bWakeUpLLWU;
static int8_t  subMode;
static ptr_set_mcu_mode pPwrp_set_mcu_mode_no_vlls;
static ptr_set_mcu_mode_vlls pPwrp_set_mcu_mode_vlls;
static pwrp_xcvr_modes_t xcvr_modes[] = {pwrp_xcvr_sleep, pwrp_xcvr_listen, pwrp_xcvr_standby,
                                          pwrp_xcvr_synth, pwrp_xcvr_rx, pwrp_xcvr_tx};
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define gUART_RX_EVENT_c         (1<<0)
#define gMcps_Cnf_EVENT_c        (1<<1)
#define gMcps_Ind_EVENT_c        (1<<2)
#define gLDPSelf_EVENT_c         (1<<3)
#define gTimePassed_EVENT_c      (1<<4)
#define gKeyPressed_EVENT_c      (1<<5)

#define gEventsAll_c	(gUART_RX_EVENT_c | gMcps_Cnf_EVENT_c | gMcps_Ind_EVENT_c | \
			gLDPSelf_EVENT_c | gTimePassed_EVENT_c | gKeyPressed_EVENT_c)

#define FlaggedDelay_ms(a)       TMR_StartSingleShotTimer(AppDelayTmr, a, DelayTimeElapsed, NULL)
/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

void UartRxCallBack(void * param);
static void HandleEvents(int32_t evSignals);
//static uint16_t HexString2Dec16(uint8_t * au8String);
static bool_t stringComp(uint8_t * au8leftString, uint8_t * au8RightString, uint8_t bytesToCompare);

static void SerialUIStateMachine(void);
static void ShortCutsParser(uint8_t u8UartData);
static void PrintTestParameters(bool_t bEraseLine);
static void DelayTimeElapsed();
static bool_t ListenModeScenario(void);
static bool_t ManualLPMScenario(void);
static void dummyFunction(uint8_t ev);
static void SetMCUConfig(uint8_t mode);
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
extern void ResetMCU(void);
/*New section added*/
void InitProject(void);
void InitSmac(void);
void Clear_CurrentLine();

/************************************************************************************
*
* InitProject
*
************************************************************************************/
void InitProject(void)
{   
  /*Global Data init*/
  testChannel                       = gDefaultChannelNumber_c;
  testPower                         = gDefaultOutputPower_c;
  shortCutsEnabled                  = FALSE; 
  evDataFromUART                    = FALSE; 
  bPrevModeIsVlpr                   = FALSE;
  bWakeUpLLWU                       = FALSE;
  evKeyPressed                      = FALSE;
  subMode                           = -1;
  pPwrp_set_mcu_mode_no_vlls        = NULL;
  pPwrp_set_mcu_mode_vlls           = NULL;
  lpdState                          = gLPDInitState_c;
}

/************************************************************************************
*************************************************************************************
* SAP functions
*************************************************************************************
************************************************************************************/
//(Management) Sap handler for managing timeout indication and ED confirm
smacErrors_t smacToAppMlmeSap(smacToAppMlmeMessage_t* pMsg, instanceId_t instance)
{
  switch(pMsg->msgType)
  {
  case gMlmeEdCnf_c:
    break;
  case gMlmeCcaCnf_c:
    break;
  case gMlmeTimeoutInd_c:
    break;
  default:
    break;
  }
  MEM_BufferFree(pMsg);
  return gErrorNoError_c;
}
//(Data) Sap handler for managing data confirm and data indication
smacErrors_t smacToAppMcpsSap(smacToAppDataMessage_t* pMsg, instanceId_t instance)
{
  switch(pMsg->msgType)
  {
  case gMcpsDataInd_c:
    if(pMsg->msgData.dataInd.pRxPacket->rxStatus == rxSuccessStatus_c)
    {
      (void)OSA_EventSet(&gTaskEvent, gMcps_Ind_EVENT_c);
    }
    break;
  case gMcpsDataCnf_c:
    (void)OSA_EventSet(&gTaskEvent, gMcps_Cnf_EVENT_c);
    break;
  default:
    break;
  }
  
  MEM_BufferFree(pMsg);
  return gErrorNoError_c;
}
/************************************************************************************
*
* Performs actions based on the event flags given as param
*
************************************************************************************/
static void HandleEvents(int32_t evSignals)
{
  uint16_t u16SerBytesCount = 0;
  
  if(evSignals & gUART_RX_EVENT_c)
  {
    if(gSerial_Success_c == Serial_GetByteFromRxBuffer(mAppSer, &gu8UartData, &u16SerBytesCount))
    {
      if(shortCutsEnabled)
      {
        ShortCutsParser(gu8UartData);  
      }
      else
      {
        evDataFromUART = TRUE;
      }
      Serial_RxBufferByteCount(mAppSer, &u16SerBytesCount);
      if(u16SerBytesCount)
      {
        (void)OSA_EventSet(&gTaskEvent, gUART_RX_EVENT_c);
      }
    }
  }
  if(evSignals & gMcps_Cnf_EVENT_c)
  {
    bTxDone = TRUE;
  }
  if(evSignals & gMcps_Ind_EVENT_c)
  {
    bRxDone = TRUE;
  }
  if(evSignals & gLDPSelf_EVENT_c)
  {
  }
  if(evSignals & gKeyPressed_EVENT_c)
  {
    evKeyPressed = TRUE;
  }
  if(evSignals & gTimePassed_EVENT_c)
  {
    timePassed = TRUE;
  }
}
/*************************************************************************
*Main Task: Application entry point*
**************************************************************************/
void main_task(uint32_t param)
{ 
  static bool_t bIsInitialized = FALSE;
  static bool_t bUserInteraction = FALSE;
  //Initialize Memory Manager, Timer Manager and LEDs.
  if( !bIsInitialized )
  {
    hardware_init();
    
    MEM_Init();
    TMR_Init();
    LED_Init();
    //initialize PHY
    Phy_Init();
    //initialize Serial Manager
    SerialManager_Init();
    
    LED_StartSerialFlash(LED1);
    
    KBD_Init(dummyFunction);
    
    OSA_EventCreate(&gTaskEvent, kEventAutoClear);
    InitApp();
    
    /*Prints the Welcome screens in the terminal*/  
    PrintMenu(cu8FreescaleLogo, mAppSer);
    bIsInitialized = TRUE;
  }
  if(!bUserInteraction)
  {
    while(1)
    {
      (void)OSA_EventWait(&gTaskEvent, gEventsAll_c, FALSE, OSA_WAIT_FOREVER ,&gTaskEventFlags);
      HandleEvents(gTaskEventFlags);
      if(evDataFromUART)
      {
        evDataFromUART = FALSE;
        if(gu8UartData == '\r')
        {
          LED_StopFlashingAllLeds();
          SelfNotificationEvent();
          bUserInteraction = TRUE;
          break;
        }
        else
        {
          PrintMenu(cu8FreescaleLogo, mAppSer);
        }
      }
      if(gUseRtos_c == 0)
      {
        break;
      }
    }
  }
  if(bUserInteraction)
  {
    while(1)
    {
      (void)OSA_EventWait(&gTaskEvent, gEventsAll_c, FALSE, OSA_WAIT_FOREVER ,&gTaskEventFlags);
      HandleEvents(gTaskEventFlags);
      SerialUIStateMachine();  
      if (gUseRtos_c == 0)
      {
        break;
      } 
    }
  } 
}

/******************************************************************************
*InitApp
******************************************************************************/
void InitApp()
{
  AppDelayTmr  = TMR_AllocateTimer();             //Allocate a timer for inserting delays whenever needed
  
  gAppTxPacket = (txPacket_t*)gau8TxDataBuffer;   //Map TX packet to buffer
  gAppRxPacket = (rxPacket_t*)gau8RxDataBuffer;   //Map Rx packet to buffer     
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
  
  Serial_InitInterface( &mAppSer, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE); //Get handle of uart interface
  Serial_SetBaudRate (mAppSer, gUARTBaudRate115200_c);   //Set 115200 as default baud
  Serial_SetRxCallBack(mAppSer, UartRxCallBack, NULL);   //Set Receive callback for uart 
  //Initialise SMAC
  InitSmac();
  //Tell SMAC who to call when it needs to pass a message to the application thread.
  Smac_RegisterSapHandlers((SMAC_APP_MCPS_SapHandler_t)smacToAppMcpsSap,(SMAC_APP_MLME_SapHandler_t)smacToAppMlmeSap,0);
  
  InitProject();
  
  SMACFillHeader(&(gAppTxPacket->smacHeader), gBroadcastAddress_c);                   //Destination Address is set in InitProject;
  
#ifdef gPHY_802_15_4g_d
  (void)MLMESetPhyMode(gDefaultMode1_c);
#endif
  (void)MLMEPAOutputAdjust(testPower);
  (void)MLMESetChannelRequest(testChannel);
  
  //prepare TX packet for Listen Mode Test.
  FLib_MemCpy(gAppTxPacket->smacPdu.smacPdu, "Wake Up Message", 15);
  gAppTxPacket->u8DataLength = 15;
  
  //prepare RX packet for Listen Mode Test.
  gAppRxPacket->u8MaxDataLength = gMaxSmacSDULength_c;
  
  /*set rx and idle resolution*/
  //Resolution 64us;
  Phy_SetListenResRx(0x10);
  //Resolution 64us;
  Phy_SetListenResIdle(0x40);
  //62*64 = 3.968 ms
  Phy_SetListenCoefRx(62);
  //187*64 =11.968 ms
  Phy_SetListenCoefIdle(187);
  
  //Phy treats listen as RX. Set to FALSE to use RX;
  PhySetRxToListen(TRUE, 0);
  
  //initialize the custom platform power mode manager
  PWRP_Init();
}

/************************************************************************************
*
* Low Power Demo State Machine
*
************************************************************************************/
void SerialUIStateMachine(void)
{
  if(gLPDSelectOption_c == lpdState && evTestParameters)
  {
    (void)MLMESetChannelRequest(testChannel);                                   
    (void)MLMEPAOutputAdjust(testPower);
    PrintTestParameters(TRUE);
    evTestParameters = FALSE;
  }
  switch(lpdState)
  {
  case gLPDInitState_c:
    shortCutsEnabled = TRUE;
    PrintMenu(cu8MainMenu, mAppSer);
    PrintTestParameters(FALSE);
    lpdState = gLPDSelectOption_c;
    break;
  case gLPDSelectOption_c:
    if(evDataFromUART)
    {
      evDataFromUART = FALSE;
      if(gu8UartData == '1')
      {
        shortCutsEnabled = FALSE;
        lpdState = gLPDManual_c;
        SelfNotificationEvent();
      }
      else if(gu8UartData == '2')
      {
        shortCutsEnabled = FALSE;
        lpdState = gLPDListen_c ;
        SelfNotificationEvent();
      }
      else if(gu8UartData == '!')
      {
        ResetMCU();
      }
    }
    break;
  case gLPDManual_c:
    if(ManualLPMScenario())
    {
      lpdState = gLPDInitState_c;
      SelfNotificationEvent();
    }
    break;
  case gLPDListen_c:
    if(ListenModeScenario())
    {
      lpdState = gLPDInitState_c;
      SelfNotificationEvent();
    }
    break;
  default:
    break;
  }
}
static bool_t ListenModeScenario(void)
{
  static bool_t firstEntry = TRUE;
  
  bool_t bBackFlag = FALSE;
  
  if(firstEntry) //if this the first call in this test, print the menu
  {
    firstEntry = FALSE;
    PrintMenu(cu8ListenMenu, mAppSer);
    if(lpd_listen_config == OpModeRX)
    {
      Serial_Print(mAppSer, "\r\n\t Going into listen mode.", gAllowToBlock_d);
      uint16_t i;
      //Need some delay so that the serial message is sent entirely
      //before changing the clock configuration
      for(i=0; i < 1000; i++)
        asm("nop");
      /*Disable TPM to adjust prescaler*/
      PhyTimerStop();
      /*change prescaler so that stack timer tick is close to the 
      tick in normal run mode*/
      TPM_HAL_SetClockDiv(g_tpmBase[gPhyTime_TPM_instance_c], kTpmDividedBy4);
      /*switch tpm clock source so phy timers work*/
      CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcMcgIrClk);
      //switch to internal clock
      CLOCK_SYS_SetConfiguration(&g_defaultClockConfigVlpr);
      /*Enable Phy Timer*/
      PhyTimerStart();
      //set XCVR into RX (or Listen) mode.
      //PHY and SMAC into RX;
      MLMERXEnableRequest(gAppRxPacket, 0);
    }
    else
    {
      Serial_Print(mAppSer, "\r\n\t Starting periodic transmission. ", gAllowToBlock_d);
      //if the configuration is OpModeTX then send the first packet
      MCPSDataRequest(gAppTxPacket);
    }
  }
  if(lpd_listen_config == OpModeRX)
  {
    if(bRxDone) //if a packet is received then the XCVR is in idle. 
    {
      bRxDone = FALSE;
      if(stringComp(gAppRxPacket->smacPdu.smacPdu, "Wake Up Message", 15))
      {
        /*Disable TPM to adjust prescaler*/
        PhyTimerStop();
        /*change prescaler so that stack timer tick is restored to the 
        tick in normal run mode*/
        TPM_HAL_SetClockDiv(g_tpmBase[gPhyTime_TPM_instance_c], kTpmDividedBy128);
        /*switch tpm clock source so phy timer works*/
        CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcPllFllSel);
        //if the packet is intended to wake up the device,
        //change clock source to external.
        CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
        /*Enable Phy Timer*/
        PhyTimerStart();
        Serial_Print(mAppSer,"\r\n\tPacket received. Exiting listen mode. Press 'p' to continue\r\n", gAllowToBlock_d);
      }
      else
      {
        //if packet is not intended to wake up the device,
        //need to switch back to listen because after packet reception,
        //the XCVR is in stand-by.
        MLMERXEnableRequest(gAppRxPacket, 0);
      }
    }
  }
  else
  {
    if(bTxDone) //if packet is sent, set timer to 5 ms
    {
      bTxDone = FALSE;
      FlaggedDelay_ms(5);
    }
    if(timePassed) //when time expires fire another packet OTA.
    {
      timePassed = FALSE;
      MLMERXDisableRequest();
      MCPSDataRequest(gAppTxPacket);
    }
  }
  if(evDataFromUART && gu8UartData == 'p')
  {
    //make sure that if a TX is in progress, to abort it
    MLMETXDisableRequest();
    
    evDataFromUART = FALSE;
    firstEntry = TRUE;
    bBackFlag = TRUE;
  }
  return bBackFlag;
}


static bool_t ManualLPMScenario(void)
{
  bool_t bBackFlag = FALSE;
  
  switch(lpdmState)
  {
  case gLPDMStateInit_c:
    PrintMenu(cu8LPCSelectMCUPowerModeMenuString, mAppSer);
    lpdmState = gLPDMStateSelectMcuMode_c;
    break;
  case gLPDMStateSelectMcuMode_c:
    if(evDataFromUART)
    {
      evDataFromUART = FALSE;
      if(gu8UartData == 'p')
      {
        lpdmState = gLPDMStateInit_c;
        bBackFlag = TRUE;
        break;
      }
      if(gu8UartData < '1' || gu8UartData > '9')
      {
        break;
      }
      SetMCUConfig(gu8UartData - '0');
      PrintMenu(cu8XCVRModes, mAppSer);
      
      lpdmState = gLPDMStateSelectXcvrMode_c;
    }
    break;
  case gLPDMStateSelectXcvrMode_c:
    if(evDataFromUART)
    {
      evDataFromUART = FALSE;
      if(gu8UartData < '1' || gu8UartData > '6')
      {
        break;
      }
      PWRP_ConfigureXCVRMode(xcvr_modes[gu8UartData-'1']);
      if(bWakeUpLLWU)
      {
        if(subMode == 0 && pPwrp_set_mcu_mode_vlls != NULL)
        {
          Serial_Print(mAppSer, "\r\nNote: Wake up from VLLS0 can be done only via GPIO" 
                       "(SW3) since LPO is disabled\r\n",
                       gAllowToBlock_d);
          Serial_Print(mAppSer, "\r\n-Press [1] to select GPIO (SW3) as wake up source\r\n",
                       gAllowToBlock_d);
        }
        else
        {
          PrintMenu(cu8WakeUpLLWU, mAppSer);
        }
        lpdmState = gLPDMStateSelectWakeUpSource_c;
      }
      else if (!bPrevModeIsVlpr)
      {
        PrintMenu(cu8WakeUpNonLLWU, mAppSer);
        lpdmState = gLPDMStateSelectWakeUpSource_c;
      }
      else
      {
        Serial_Print(mAppSer, "\r\n\r\n Long press any push button to exit VLPR\r\n\r\n", gAllowToBlock_d);
        lpdmState = gLPDMStateEnterLowPower_c;
        FlaggedDelay_ms(4);
      }
    }
    break;
  case gLPDMStateSelectWakeUpSource_c:
    if(evDataFromUART)
    {
      bool_t validKey = TRUE;
      evDataFromUART = FALSE;
      if(gu8UartData == '1' && !bWakeUpLLWU)
      {
        PWRP_SetWakeUpSource(UART_WAKE_UP);
      }
      else if( (gu8UartData == '2' && !bWakeUpLLWU) ||
              (gu8UartData == '1' && bWakeUpLLWU))
      {
        PWRP_SetWakeUpSource(GPIO_WAKE_UP);
      }
      else if( (gu8UartData == '3' && !bWakeUpLLWU) ||
              (gu8UartData == '2' && bWakeUpLLWU && subMode !=0))
      {
        PWRP_SetWakeUpSource(LPTMR_WAKE_UP);
        PWRP_UpdateLPTMRCount(gLPTMR_MS_COUNT_c);
      }
      else
      {
        validKey = FALSE;
      }
      
      if(validKey)
      {
        lpdmState = gLPDMStateEnterLowPower_c;
        SelfNotificationEvent();
      }
    }
    break;
  case gLPDMStateEnterLowPower_c:
    if(timePassed)
    {
      timePassed = FALSE;
    }
    if(subMode != -1 && pPwrp_set_mcu_mode_vlls != NULL)
    {
      /*never returns*/
      pPwrp_set_mcu_mode_vlls(subMode);
    }
    else
    {
      pPwrp_set_mcu_mode_no_vlls();
      if(!bPrevModeIsVlpr)
      {
        if(PWRP_GetWakeUpReason() != UNKNOWN_WAKE_UP)
        {
          Serial_Print(mAppSer, "\r\n\t--Back in run mode--\r\n\r\nPress [ENTER] to continue\r\n", gAllowToBlock_d);
        }
        else
        {
          Serial_Print(mAppSer, "\r\n\t--Back in run mode (other interrupt)--\r\n\r\n"
                       "Press [ENTER] to continue\r\n", gAllowToBlock_d);
        }
      }
      else
      {
        SelfNotificationEvent();
      }
      lpdmState = gLPDMStateHandleExitLowPower_c;
    }
    
    break;
  case gLPDMStateHandleExitLowPower_c:
    if(evDataFromUART)
    {
      evDataFromUART = FALSE;
      if(gu8UartData == '\r' && !bPrevModeIsVlpr)
      {
        lpdmState = gLPDMStateInit_c;
        SelfNotificationEvent();
      }
    }
    if(bPrevModeIsVlpr && evKeyPressed)
    {
      evKeyPressed = FALSE;
      bPrevModeIsVlpr = FALSE;
      
      Enter_RUN();
      /*restore clock source used by tpm in run mode*/
      CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcPllFllSel);
      
      Serial_Print(mAppSer, "\r\n\t--Back in run mode--\r\n", gAllowToBlock_d);
      lpdmState = gLPDMStateInit_c;
      SelfNotificationEvent();
    }
    break;
  default:
    break;
  }
  return bBackFlag;
}
/************************************************************************************
*
* Sets the appropriate handler for MCU low power mode
*
************************************************************************************/
static void SetMCUConfig(uint8_t mode)
{
  pPwrp_set_mcu_mode_no_vlls = NULL;
  pPwrp_set_mcu_mode_vlls = NULL;
  subMode = -1;
  bWakeUpLLWU = FALSE;
  bPrevModeIsVlpr = FALSE;
  switch(mode)
  {
  case 1:
    pPwrp_set_mcu_mode_no_vlls = Enter_Wait;
    break;
  case 2:
    pPwrp_set_mcu_mode_no_vlls = Enter_Stop;
    break;
  case 3:
    pPwrp_set_mcu_mode_no_vlls = Enter_VLPR;
    /*when switching to the vlpr clock config
    the clock source for TPM is changed to mcgirclk so to use
    keyboard to exit this mode the clock source must be changed*/
    CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcMcgIrClk);
    bPrevModeIsVlpr = TRUE;
    break;
  case 4:
    pPwrp_set_mcu_mode_no_vlls = Enter_VLPW;
    break;
  case 5:
    pPwrp_set_mcu_mode_no_vlls = Enter_VLPS;
    break;
  case 6:
    pPwrp_set_mcu_mode_no_vlls = Enter_LLS;
    bWakeUpLLWU = TRUE;
    break;
  case 7:
  case 8:
  case 9:
    pPwrp_set_mcu_mode_vlls = Enter_VLLS;
    bWakeUpLLWU = TRUE;
    if(mode == 7)
    {
      subMode = 3;
    }
    else if(mode == 8)
    {
      subMode = 1;
    }
    else
    {
      subMode = 0;
    }
    break;
  default:
    pPwrp_set_mcu_mode_no_vlls = Enter_LLS;
    bWakeUpLLWU = TRUE;
    break;
  }
}
/************************************************************************************
*
* Prints parameters related menu (shortcut menu)
*
************************************************************************************/
void PrintTestParameters(bool_t bEraseLine)
{
  uint8_t u8Index1;
  
  if(bEraseLine)
  {
    Serial_Print(mAppSer,"\r",gAllowToBlock_d);
    for(u8Index1 = 0; u8Index1 < 80; u8Index1++)
    {
      Serial_Print(mAppSer," ",gAllowToBlock_d);
    }
    Serial_Print(mAppSer,"\r",gAllowToBlock_d);
  }
  
  Serial_Print(mAppSer, " Channel ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer, (uint32_t)testChannel);
  Serial_Print(mAppSer,", Power ", gAllowToBlock_d);
  Serial_PrintDec(mAppSer,(uint32_t)testPower);
  Serial_Print(mAppSer,", Listen Mode Test: ", gAllowToBlock_d);
  if(lpd_listen_config == OpModeRX)
  {
    Serial_Print(mAppSer,"Rx", gAllowToBlock_d);
  }
  else
  {
    Serial_Print(mAppSer,"Tx", gAllowToBlock_d);
  }
  Serial_Print(mAppSer," >", gAllowToBlock_d);
}
/*****************************************************************************
* dummyFunction function
*
* Interface assumptions:
* This callback is triggered when a push button is pressed
*
* Return Value:
* None
*****************************************************************************/
static void dummyFunction(uint8_t ev)
{
  (void)ev;
  if(bPrevModeIsVlpr)
  {
    OSA_EventSet(&gTaskEvent, gKeyPressed_EVENT_c);
  }
}
/*****************************************************************************
* UartRxCallBack function
*
* Interface assumptions:
* This callback is triggered when a new byte is received over the UART
*
* Return Value:
* None
*****************************************************************************/
void UartRxCallBack(void * param) 
{
  (void)OSA_EventSet(&gTaskEvent, gUART_RX_EVENT_c);
}

/************************************************************************************
*
* Performs actions based on shortcut key pressed in the menus where this is allowed
*
************************************************************************************/
void ShortCutsParser(uint8_t u8UartData)
{
  evTestParameters = TRUE;
  evDataFromUART = FALSE;
  switch(u8UartData){
  case 'q': 
    if(gMaxChannel_c == testChannel)
      testChannel = gMinChannel_c;
	else
	  testChannel++;
    break;
  case 'w':
    if(gMinChannel_c == testChannel)
      testChannel = gMaxChannel_c;
    else
      testChannel--;
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
  case 'r':
      lpd_listen_config = OpModeRX;
      break;
  case 't':
      lpd_listen_config = OpModeTX;
      break;
  default:
    evDataFromUART = TRUE;
    evTestParameters = FALSE;
    break;
  }
}
/***********************************************************************
*********************Utilities Software********************************
************************************************************************/

//uint16_t HexString2Dec16(uint8_t * au8String)
//{
//  uint8_t u8LocIndex=0;
//  uint8_t u8LocIndex2=0;
//  uint16_t u16DecValue = 0;
//  
//  while(au8String[u8LocIndex]){
//    u8LocIndex++;
//  }
//  
//  while(u8LocIndex--){
//    if((au8String[u8LocIndex] >= '0') && (au8String[u8LocIndex] <= '9'))
//      u16DecValue |= ((uint16_t)(au8String[u8LocIndex] - '0'))<<(u8LocIndex2*4);
//    else if((au8String[u8LocIndex] >= 'A') && (au8String[u8LocIndex] <= 'F')){
//      u16DecValue |= ((uint16_t)(au8String[u8LocIndex] - 'A' + 0x0A))<<(u8LocIndex2*4);    
//    }else{
//      u16DecValue |= ((uint16_t)(au8String[u8LocIndex] - 'a' + 0x0A))<<(u8LocIndex2*4);        
//    }
//    u8LocIndex2++;
//  }
//  
//  return u16DecValue;
//}

bool_t stringComp(uint8_t * au8leftString, uint8_t * au8RightString, uint8_t bytesToCompare)
{
  do
  {
  }while((*au8leftString++ == *au8RightString++) && --bytesToCompare);
  return(0 == bytesToCompare);
}

static void DelayTimeElapsed()
{
  (void)OSA_EventSet(&gTaskEvent, gTimePassed_EVENT_c);
}

inline void Clear_CurrentLine()
{
  uint8_t index;
  Serial_Print(mAppSer,"\r",gAllowToBlock_d);
  for(index = 0; index  < gMaxSmacSDULength_c; index++)
    Serial_Print(mAppSer," ",gAllowToBlock_d);
  Serial_Print(mAppSer,"\r",gAllowToBlock_d);
}
/***********************************************************************
************************************************************************/