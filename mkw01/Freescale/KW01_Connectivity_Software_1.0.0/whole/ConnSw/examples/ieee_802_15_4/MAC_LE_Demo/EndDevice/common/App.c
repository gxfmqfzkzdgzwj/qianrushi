/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file App.c
* MAC LE Demo Coordinator application.
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

#include "App.h"

/* Drv */
#include "LED.h"
#include "Keyboard.h"

/* Fwk */
#include "SerialManager.h"
#include "RNG_Interface.h"
#include "Panic.h"
#include "MemManager.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_osa_ext.h"

/* 802.15.4 */
#include "PhyInterface.h"
#include "MacInterface.h"

/* KSDK */
#include "board.h"
#include "fsl_os_abstraction.h"

#if mDemoLowPower_d
  #include "PWR_Interface.h"
#endif

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

#define mAppStackSize_c 600
#define mAppTaskPrio_c  4

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/* Forward declarations of helper functions */
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg);
static void    App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn);
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType);
static void    App_ConfigureMAC_LE_PIBs(void);

void main_task(uint32_t param);
void AppThread(uint32_t param);
void App_Idle_Task(uint32_t param);

void App_init( void );

resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);

extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);

static void App_HandleKeys(uint8_t events);

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

OSA_TASK_DEFINE( APP, mAppStackSize_c );

/* The short address and PAN ID of the coordinator*/
static const uint16_t macShortAddress = mDefaultValueOfShortAddress_c;
static const uint16_t macPanId = mDefaultValueOfPanId_c;

/* The current logical channel (frequency band) */
static uint8_t macLogicalChannel = mDefaultValueOfLogicalChannel_c;

/* Number of pending data packets */
static uint8_t mcPendingPackets;

/* Application input queues */
static anchor_t mMlmeNwkInputQueue;
static anchor_t mMcpsNwkInputQueue;

static const uint64_t mExtendedAddress = mMacExtendedAddress_c;

static instanceId_t mMacInstance;

static uint8_t      interfaceId;

event_t             mAppEvent;
task_handler_t      mAppTaskHandler;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/* The current state of the applications state machine */
uint8_t gState;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This is the first task created by the OS. This task will initialize 
*         the system
*
* \param[in]  param
*
********************************************************************************** */
void main_task(uint32_t param)
{
    static uint8_t initialized = FALSE;
    osa_status_t status = kStatus_OSA_Success;
    
    if( !initialized )
    {
        initialized = TRUE; 
        hardware_init();
        MEM_Init();
        Phy_Init();
        TMR_Init();
        LED_Init();
        SerialManager_Init();
        RNG_Init();
         
        /* 802.15.4 MAC init */
        MAC_Init();
    
#if mDemoLowPower_d
        PWR_Init();
        PWR_DisallowDeviceToSleep();
#endif

        /* Bind to MAC layer */
        mMacInstance = BindToMAC((instanceId_t)0);
        Mac_RegisterSapHandlers(MCPS_NWK_SapHandler, MLME_NWK_SapHandler, mMacInstance);
        
        App_init();
        
        /* Create application task */
        status = OSA_TaskCreate((task_t)AppThread, "App_Task", mAppStackSize_c, APP_stack, mAppTaskPrio_c, 0, FALSE, &mAppTaskHandler);
        if( kStatus_OSA_Success != status )
        {
            panic(0,0,0,0);
            return;
        }
    }
    
    /* Call application Idle task */
    App_Idle_Task( param );
}

/*! *********************************************************************************
* \brief  This is the application's Idle task.
*
* \param[in]  argument
*
* \return  None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void App_Idle_Task(uint32_t param)
{
    while(1)
    {
#if mDemoLowPower_d
        if( PWR_CheckIfDeviceCanGoToSleep() )
        {
            Led1Off();
            Led2Off();
            Led3Off();
            Led4Off();
            
            PWR_EnterLowPower();
            
            Led1On();
            Led2On();
            Led3On();
            Led4On();
        }
#endif
        if( !gUseRtos_c )
        {
            break;
        }
    }
}

/*****************************************************************************
* Initialization function for the App Task. This is called during
* initialization and should contain any application specific initialization
* (ie. hardware initialization/setup, table initialization, power up
* notificaiton.
*
* Interface assumptions: None
*
* Return value: None
*
*****************************************************************************/
void App_init(void)
{    
    OSA_EventCreate(&mAppEvent, kEventAutoClear);

    /* The initial application state */
    gState = stateInit;
    
    /* Reset number of pending packets */
    mcPendingPackets = 0;
    
    /* register keyboard callback function */
    KBD_Init(App_HandleKeys);
    
    /* Initialize the UART so that we can print out status messages */
    Serial_InitInterface(&interfaceId, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);
    Serial_SetBaudRate(interfaceId, gUARTBaudRate115200_c);    
    
    /* Prepare input queues */
    MSG_InitQueue(&mMlmeNwkInputQueue); 
    MSG_InitQueue(&mMcpsNwkInputQueue);

    /* Initialize the MAC 802.15.4 extended address */
    Mac_SetExtendedAddress( (uint8_t*)&mExtendedAddress, mMacInstance );
    
    /* Signal app ready */  
    LED_StartSerialFlash(LED1);
    
    /* Press a switch to start Demo */
}

/*! *********************************************************************************
* \brief  This function represents the Application task. 
*         This task reuses the stack alocated for the MainThread.
*
* \param[in]  argument
*
********************************************************************************** */
void AppThread (uint32_t param)
{
  event_flags_t ev;
  /* Pointer for storing the messages from MLME */
  void *pMsgIn;
  /* Stores the error/success code returned by some functions. */
  uint8_t ret;
    
  while(1)
  {
      OSA_EventWait(&mAppEvent, 0x00FFFFFF, FALSE, OSA_WAIT_FOREVER, &ev);
	  
      if( !gUseRtos_c && !ev )
      {
          break;
      }
      
	  pMsgIn = NULL;
      
      /* Dequeue the MLME message */
      if (ev & gAppEvtMessageFromMLME_c)
      {
          /* Get the message from MLME */
          pMsgIn = MSG_DeQueue(&mMlmeNwkInputQueue);
          
          /* Any time a beacon might arrive. Always handle the beacon frame first */
          if (pMsgIn)
          {               
              ret = App_WaitMsg(pMsgIn, gMlmeBeaconNotifyInd_c);
              if(ret == errorNoError)
              {
                  /* ALWAYS free the beacon frame contained in the beacon notify indication.*/
                  /* ALSO the application can use the beacon payload.*/
                  MSG_Free(((nwkMessage_t *)pMsgIn)->msgData.beaconNotifyInd.pBufferRoot);
              }
          }
      }
      /* The application state machine */
      
      switch(gState)
      {
      case stateInit:
          gState = stateListen;
                      
          /* Configure MAC LE PIBs */
          App_ConfigureMAC_LE_PIBs();
          
#if mDemoLowPower_d
          /* Allow sleep by MAC LE events */
          PWR_AllowDeviceToSleep();
#endif
          break;
       
        
      case stateListen:
          /* Stay in this state forever. 
          Transmit the data received on UART */
          if (ev & gAppEvtMessageFromMLME_c)
          {
              /* Get the message from MLME */
              if (pMsgIn)
              {      
                  /* Process it */
                  ret = App_HandleMlmeInput(pMsgIn);
                  /* Messages from the MLME must always be freed. */
              }
          }
          break;
      }
      
      if (pMsgIn)
      {
          /* Messages must always be freed. */ 
          MSG_Free(pMsgIn);
          pMsgIn = NULL;
      }
      
      if (ev & gAppEvtMessageFromMCPS_c)
      {      
          /* Get the message from MCPS */
          pMsgIn = MSG_DeQueue(&mMcpsNwkInputQueue);
          if (pMsgIn)
          {
              /* Process it */
              App_HandleMcpsInput(pMsgIn);
              /* Messages from the MCPS must always be freed. */
              MSG_Free(pMsgIn);
              pMsgIn = NULL;
          }
      }
      
      /* Check for pending messages in the Queue */
      if( MSG_Pending(&mMcpsNwkInputQueue) )
      {
          OSA_EventSet(&mAppEvent, gAppEvtMessageFromMCPS_c);
      }
         
      if( MSG_Pending(&mMlmeNwkInputQueue) )
      {
          OSA_EventSet(&mAppEvent, gAppEvtMessageFromMLME_c);
      }
      
      if( !gUseRtos_c )
      {
          break;
      }
  }/* while(1) */
}

/************************************************************************************
*************************************************************************************
* Private functions
**************************************************************************************/

/******************************************************************************
* The App_HandleMlmeInput(nwkMessage_t *pMsg) function will handle various
* messages from the MLME, e.g. (Dis)Associate Indication.
*
* The function may return either of the following values:
*   errorNoError:   The message was processed.
*   errorNoMessage: The message pointer is NULL.
******************************************************************************/
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg)
{
  if(pMsg == NULL)
    return errorNoMessage;
  
  /* Handle the incoming message. The type determines the sort of processing.*/
  switch(pMsg->msgType) {    
  case gMlmeCommStatusInd_c:
    /* Sent by the MLME after the Association Response has been transmitted. */
    //Serial_Print(interfaceId,"Received an MLME-Comm-Status Indication from the MAC\n\r", gAllowToBlock_d);
    break;
  }
  return errorNoError;
}

/******************************************************************************
* The App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn) function will handle 
* messages from the MCPS, e.g. Data Confirm, and Data Indication.
*
******************************************************************************/
static void App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn)
{
  switch(pMsgIn->msgType)
  {
    /* The MCPS-Data confirm is sent by the MAC to the network 
       or application layer when data has been sent. */
  case gMcpsDataCnf_c:
    if(mcPendingPackets)
      mcPendingPackets--;
    break;
  
  case gMcpsDataInd_c:
    /* The MCPS-Data indication is sent by the MAC to the network 
       or application layer when data has been received. We simply 
       copy the received data to the UART. */
    //Serial_SyncWrite( interfaceId,pMsgIn->msgData.dataInd.pMsdu, pMsgIn->msgData.dataInd.msduLength );
    break;
  }
}

/******************************************************************************
* The App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType) function does not, as
* the name implies, wait for a message, thus blocking the execution of the
* state machine. Instead the function analyzes the supplied message to determine
* whether or not the message is of the expected type.
* The function may return either of the following values:
*   errorNoError: The message was of the expected type.
*   errorNoMessage: The message pointer is NULL.
*   errorWrongConfirm: The message is not of the expected type.
*
******************************************************************************/
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType)
{
  /* Do we have a message? If not, the exit with error code */
  if(pMsg == NULL)
    return errorNoMessage;

  /* Is it the expected message type? If not then exit with error code */
  if(pMsg->msgType != msgType)
    return errorWrongConfirm;

  /* Found the expected message. Return with success code */
  return errorNoError;
}

void App_ConfigureMAC_LE_PIBs()
{
    uint32_t value;
    
    /* Message for the MLME will be allocated and attached to this pointer */
    mlmeMessage_t *pMsg;
    
    /* Allocate a message for the MLME (We should check for NULL). */
    pMsg = MSG_AllocType(mlmeMessage_t);
    if( pMsg != NULL )
    {
        /* Configure addressing PIBs */
        /* MAC short address */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibShortAddress_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint16_t*)&macShortAddress;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* MAC PAN ID */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibPanId_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint16_t*)&macPanId;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* MAC logical channel */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibLogicalChannel_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint8_t*)&macLogicalChannel;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
#if mDemoCsl_d
        /* Initialize the MAC CSL Max Period */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibCslPeriod_c;
        value = gMPibCslPeriodValue_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint16_t*)&value;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );   
#elif mDemoRit_d
        /* Initialize the MAC RIT Tx Wait Duration */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibRitTxWaitDuration_c;
        value = gMPibRitTxWaitDurationValue_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint32_t*)&value;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Initialize the MAC RIT Data Wait Duration */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibRitDataWaitDuration_c;
        value = gMPibRitDataWaitDurationValue_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint8_t*)&value;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance ); 
        
        /* Initialize the MAC RIT IE Listening schedule */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibRitIe_c;
        ((macRitIe_t*)&value)->T0 = gMPibRitIeT0_c;
        ((macRitIe_t*)&value)->N = gMPibRitIeN_c;
        ((macRitIe_t*)&value)->T = gMPibRitIeT_c;
        pMsg->msgData.setReq.pibAttributeValue = (macRitIe_t*)&value;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );

        /* Initialize the MAC RIT Period */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibRitPeriod_c;
        value = gMPibRitPeriodValue_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint32_t*)&value;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance ); 
#endif
        
        /* Free message, all requests were sync */
        MSG_Free(pMsg);
    }
}

static void App_HandleKeys
  (
  uint8_t events  /*IN: Events from keyboard modul  */
  )
{
#if gKBD_KeysCount_c > 0
  switch ( events ) 
    { 
      case gKBD_EventSW1_c:
      case gKBD_EventSW2_c:
      case gKBD_EventSW3_c:
      case gKBD_EventSW4_c:
      case gKBD_EventLongSW1_c:
      case gKBD_EventLongSW2_c:
      case gKBD_EventLongSW3_c:
      case gKBD_EventLongSW4_c:
        if(gState == stateInit)
          {
          LED_StopFlashingAllLeds();
          OSA_EventSet(&mAppEvent, gAppEvtDummyEvent_c);
          }
    break;
    }
#endif
}

/******************************************************************************
* The following functions are called by the MAC to put messages into the
* Application's queue. They need to be defined even if they are not used
* in order to avoid linker errors.
******************************************************************************/
resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId)
{
  /* Put the incoming MLME message in the applications input queue. */
  MSG_Queue(&mMlmeNwkInputQueue, pMsg);
  OSA_EventSet(&mAppEvent, gAppEvtMessageFromMLME_c);
  return gSuccess_c;
}

resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId)
{
  /* Put the incoming MCPS message in the applications input queue. */
  MSG_Queue(&mMcpsNwkInputQueue, pMsg);
  OSA_EventSet(&mAppEvent, gAppEvtMessageFromMCPS_c);
  return gSuccess_c;
}
