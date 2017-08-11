/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file App.c
* MAC TSCH Demo End Device application.
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

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/* Forward declarations of helper functions */
static uint8_t App_HandleMlmeInput(nwkMessage_t *pMsg);
static void    App_HandleMcpsInput(mcpsToNwkMessage_t *pMsgIn);
static void    App_TransmitData(void* param);
static uint8_t App_WaitMsg(nwkMessage_t *pMsg, uint8_t msgType);
static uint8_t App_StartScan(void);
static uint8_t App_HandleScanPassiveConfirm(nwkMessage_t *pMsg);
static uint8_t App_Configure_MAC_TSCH_Params(void);
static void    App_HandleKeys(key_event_t events);

void main_task(uint32_t param);
void AppThread(uint32_t param);

void App_init( void );

resultType_t MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId);
resultType_t MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);

extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);

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

/* The short address of the ED */
static const uint16_t macShortAddress = mDefaultValueOfShortAddress_c;

/* The current logical channel (frequency band) */
static uint8_t macLogicalChannel = mDefaultValueOfLogicalChannel_c;

/* Data request packet  */
static nwkToMcpsMessage_t *mpPacket;

/* The MSDU handle is a unique data packet identifier */
static uint8_t mMsduHandle;

/* Number of pending data packets */
static uint8_t mcPendingPackets;

/* Application input queues */
static anchor_t mMlmeNwkInputQueue;
static anchor_t mMcpsNwkInputQueue;

static const uint64_t mExtendedAddress = mMacExtendedAddress_c;

static instanceId_t mMacInstance;

/* Information about the PAN we are part of */
static panDescriptor_t mCoordInfo;

/* TSCH specific */
static uint8_t        mHoppingSequenceList[] = mMacHoppingSequenceList_c;

static uint8_t dataBuffer[mDataTxMsduLen_c];

static tmrTimerID_t mAppTxDataTimerID;
event_t mAppEvent;

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

        /* Bind to MAC layer */
        mMacInstance = BindToMAC((instanceId_t)0);
        Mac_RegisterSapHandlers(MCPS_NWK_SapHandler, MLME_NWK_SapHandler, mMacInstance);

        App_init();
    }

    /* Call application task */
    AppThread( param );
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
    
    /* Prepare input queues */
    MSG_InitQueue(&mMlmeNwkInputQueue); 
    MSG_InitQueue(&mMcpsNwkInputQueue);

    /* Initialize the MAC 802.15.4 extended address */
    Mac_SetExtendedAddress( (uint8_t*)&mExtendedAddress, mMacInstance );
    
    /*signal app ready*/  
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
  void *pMsgIn = NULL;
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
          gState = stateScanPassiveStart;
          OSA_EventSet(&mAppEvent, gAppEvtDummyEvent_c);
          break;
      
      case stateScanPassiveStart:
          ret = App_StartScan();
          if(ret == errorNoError)
          {
              gState = stateScanPassiveWaitConfirm;
          }
          break;
          
      case stateScanPassiveWaitConfirm:
          /* Stay in this state until the MLME-SCAN confirm message
          arrives, and then goto the TSCH On state. */
          if (ev & gAppEvtMessageFromMLME_c)
          {
              if (pMsgIn)
              {        
                  ret = App_WaitMsg(pMsgIn, gMlmeScanCnf_c);
                  if(ret == errorNoError)
                  {
                      ret = App_HandleScanPassiveConfirm(pMsgIn);
                      if(ret == errorNoError)
                      {                            
                          gState = stateTschOn;
                      }
                      else
                      {                           
                          /* Restart Passive scan */
                          gState = stateScanPassiveStart;
                      }
					  OSA_EventSet(&mAppEvent, gAppEvtDummyEvent_c);
                  }
              }
          }
          break; 
          
      case stateTschOn:
          App_Configure_MAC_TSCH_Params();
          gState = stateListen;
          
          /* Initialize Data Tx interval timer */
          mAppTxDataTimerID = TMR_AllocateTimer();
          if( gTmrInvalidTimerID_c != mAppTxDataTimerID )
          {
              TMR_StartIntervalTimer(mAppTxDataTimerID, gDataTxIntervalMs_c, App_TransmitData, NULL);
          }
           
          /* Initialize data buffer */
          for( uint32_t i=0; i<mDataTxMsduLen_c; i++)
          {
              dataBuffer[i] = i;
          }
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
      
      if (gUseRtos_c == 0)
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

/******************************************************************************
* The App_StartScan() function will start the scan process of the
* specified type in the MAC. This is accomplished by allocating a MAC message,
* which is then assigned the desired scan parameters and sent to the MLME
* service access point.
* The function may return either of the following values:
*   errorNoError:          The Scan message was sent successfully.
*   errorInvalidParameter: The MLME service access point rejected the
*                          message due to an invalid parameter.
*   errorAllocFailed:      A message buffer could not be allocated.
*
******************************************************************************/
static uint8_t App_StartScan(void)
{
  mlmeMessage_t *pMsg;
  mlmeScanReq_t *pScanReq;

  /* Allocate a message for the MLME (We should check for NULL). */
  pMsg = MSG_AllocType(mlmeMessage_t);
  if(pMsg != NULL)
  {
    /* This is a MLME-SCAN.req command */
    pMsg->msgType = gMlmeScanReq_c;
    /* Create the Scan request message data. */
    pScanReq = &pMsg->msgData.scanReq;
    /* gScanModeED_c, gScanModeActive_c, gScanModePassive_c, or gScanModeOrphan_c */
    pScanReq->scanType = gScanModePassive_c;

    pScanReq->channelPage = gChannelPageId9_c;
    pScanReq->scanChannels[0] = 1 << mDefaultValueOfLogicalChannel_c;
    FLib_MemSet(pScanReq->scanChannels+1, 0, sizeof(channelMask_t)-sizeof(uint32_t));

    /* Duration per channel 0-14 (dc). T[sec] = (16*960*((2^dc)+1))/1000000.
       A scan duration of 3 on 16 channels approximately takes 2 secs. */
    pScanReq->scanDuration = mDefaultValueOfScanDuration_c;
    pScanReq->securityLevel = gMacSecurityNone_c;
    
    /* Send the Scan request to the MLME. */
    if(NWK_MLME_SapHandler( pMsg, mMacInstance ) == gSuccess_c)
    {
      return errorNoError;
    }
    else
    {
      return errorInvalidParameter;
    }
  }
  else
  {
    /* Allocation of a message buffer failed. */
    return errorAllocFailed;
  }
}

/******************************************************************************
* The App_HandleScanPassiveConfirm(nwkMessage_t *pMsg) function will handle the
* Active Scan confirm message received from the MLME when the Active scan has
* completed. The message contains a list of PAN descriptors. Based on link
* quality inforamtion in the pan descriptors the nearest coordinator is chosen.
* The corresponding pan descriptor is stored in the global variable mCoordInfo. 
*
* The function may return either of the following values:
*   errorNoError:       A suitable pan descriptor was found.
*   errorNoScanResults: No scan results were present in the confirm message.
*
******************************************************************************/
static uint8_t App_HandleScanPassiveConfirm(nwkMessage_t *pMsg)
{
  void    *pBlock;
  uint8_t panDescListSize = pMsg->msgData.scanCnf.resultListSize;
  uint8_t rc = errorNoScanResults;
  uint8_t j;
  uint8_t bestLinkQuality = 0;  
  
  panDescriptorBlock_t *pDescBlock = pMsg->msgData.scanCnf.resList.pPanDescriptorBlockList;   

  /* Check if the scan resulted in any coordinator responses. */
  if (panDescListSize > 0)
  {    
    /* Check all PAN descriptors. */
    while (NULL != pDescBlock)
    {
      for (j = 0; j < pDescBlock->panDescriptorCount; j++)
      {
        /* Find the nearest coordinator using the link quality measure. */
        /* Keep only short address coordinators as candidate */
        if( ( pDescBlock->panDescriptorList[j].linkQuality > bestLinkQuality ) &&
            ( pDescBlock->panDescriptorList[j].coordAddrMode == gAddrModeShortAddress_c ) )
        {
          /* Save the information of the coordinator candidate. If we
             find a better candiate, the information will be replaced. */
          FLib_MemCpy(&mCoordInfo, &pDescBlock->panDescriptorList[j], sizeof(panDescriptor_t));
          bestLinkQuality = pDescBlock->panDescriptorList[j].linkQuality;
          rc = errorNoError;
        }
      }
      
      /* Free current block */
      pBlock = pDescBlock;
      pDescBlock = pDescBlock->pNext;              
      MSG_Free(pBlock);
    }
  }
  
  if(rc == errorNoError)
  {
    /* If we have found a coodinator we must setup the MAC to
       synchronize to the PANC timeslot start and ASN. This requires 
       us to set the PAN ID attribute of the MAC PIB to the PAN ID 
       of the coordinator and the coord short address */
      mlmeMessage_t *pMsgOut = MSG_AllocType(mlmeMessage_t);
      if(pMsgOut != NULL)
      {
        pMsgOut->msgType = gMlmeSetReq_c;
        pMsgOut->msgData.setReq.pibAttribute = gMPibPanId_c;
        pMsgOut->msgData.setReq.pibAttributeValue = &mCoordInfo.coordPanId;
        (void)NWK_MLME_SapHandler( pMsgOut, mMacInstance );

        pMsgOut->msgType = gMlmeSetReq_c;
        pMsgOut->msgData.setReq.pibAttribute = gMPibCoordShortAddress_c;
        pMsgOut->msgData.setReq.pibAttributeValue = &mCoordInfo.coordAddress;
        (void)NWK_MLME_SapHandler( pMsgOut, mMacInstance );
        
        /* Free message, all requests were sync */
        MSG_Free(pMsgOut);
      }
  }
  else if (pDescBlock)
  {
      MSG_Free(pDescBlock);
  }

  return rc;
}

static uint8_t App_Configure_MAC_TSCH_Params(void)
{
    uint32_t i;
    uint16_t hoppingSequenceLength = mMacHoppingSequenceLen_c;
    macTschRole_t tschRole = gMacTschRoleDevice_c;  

    /* Message for the MLME will be allocated and attached to this pointer */
    mlmeMessage_t *pMsg = MSG_AllocType(mlmeMessage_t);
    
    if( pMsg != NULL )
    {
        /* Configure addressing PIBs */
        /* MAC short address */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibShortAddress_c;
        pMsg->msgData.setReq.pibAttributeValue = (uint16_t*)&macShortAddress;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Set MAC Hopping Sequence Length */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibHoppingSequenceLength_c;
        pMsg->msgData.setReq.pibAttributeValue = &hoppingSequenceLength;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Set MAC Hopping Sequence List */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibHoppingSequenceList_c;
        pMsg->msgData.setReq.pibAttributeValue = mHoppingSequenceList;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Set slotframes */
        for( i=0; i<sizeof(slotframeArray)/sizeof(macSlotframeIe_t); i++ )
        {
            pMsg->msgType = gMlmeSetSlotframeReq_c;
            pMsg->msgData.setSlotframeReq.operation = gMacSetSlotframeOpAdd_c;
            pMsg->msgData.setSlotframeReq.slotframeHandle = slotframeArray[i].macSlotframeHandle;
            pMsg->msgData.setSlotframeReq.size = slotframeArray[i].macSlotframeSize;
            (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        }

        /* Set links */
        for( i=0; i<sizeof(linkArray)/sizeof(macLink_t); i++ )
        {
            pMsg->msgType = gMlmeSetLinkReq_c;
            pMsg->msgData.setLinkReq.operation = gMacSetLinkOpAdd_c;
            pMsg->msgData.setLinkReq.slotframeHandle = linkArray[i].slotframeHandle;
            pMsg->msgData.setLinkReq.linkHandle = linkArray[i].macLinkHandle;
            pMsg->msgData.setLinkReq.linkType = linkArray[i].macLinkType;
            pMsg->msgData.setLinkReq.nodeAddr = linkArray[i].macNodeAddress;
            pMsg->msgData.setLinkReq.timeslot = linkArray[i].macLinkIe.timeslot;
            pMsg->msgData.setLinkReq.channelOffset = linkArray[i].macLinkIe.channelOffset;
            pMsg->msgData.setLinkReq.linkOptions = linkArray[i].macLinkIe.macLinkOptions;
            (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        }
        
        /* Configure PANC as time source neighbor */
        pMsg->msgType = gMlmeKeepAliveReq_c;
        pMsg->msgData.keepAliveReq.dstAddr = mDefaultValueOfCoordAddress_c;
        pMsg->msgData.keepAliveReq.keepAlivePeriod = mMacKeepAlivePeriod_c;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Set TSCH Role as normal node */
        pMsg->msgType = gMlmeSetReq_c;
        pMsg->msgData.setReq.pibAttribute = gMPibTschRole_c;
        pMsg->msgData.setReq.pibAttributeValue = &tschRole;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );          
        
        /* Enable TSCH */
        pMsg->msgType = gMlmeTschModeReq_c;
        pMsg->msgData.tschModeReq.tschMode = gMacTschModeOn_c;
        (void)NWK_MLME_SapHandler( pMsg, mMacInstance );
        
        /* Free message, all requests were sync */
        MSG_Free(pMsg);
    }
    else
    {
        /* Allocation of a message buffer failed. */
        return errorAllocFailed;
    }

    return errorNoError;
}

void App_TransmitData(void *param)
{
    uint8_t temp;
    uint32_t i;
    
    /* Allow max mDefaultValueOfMaxPendingDataPackets_c sent to MAC */
    if( mcPendingPackets > mDefaultValueOfMaxPendingDataPackets_c )
    {
        return;
    }
  
    mpPacket = MSG_Alloc(sizeof(nwkToMcpsMessage_t) + mDataTxMsduLen_c);

    if(mpPacket != NULL)
    {
        /* Shift data buffer */
        temp = dataBuffer[0];
        for( i=0; i<mDataTxMsduLen_c-1; i++)
        {
            dataBuffer[i] = dataBuffer[i+1];
        }
        dataBuffer[mDataTxMsduLen_c-1] = temp;
      
        /* Create an MCPS-Data Request message. */
        mpPacket->msgData.dataReq.pMsdu = &dataBuffer[0];
        mpPacket->msgType = gMcpsDataReq_c;
        /* Create the header using device information stored when creating 
        the association response. In this simple example the use of short
        addresses is hardcoded. In a real world application we must be
        flexible, and use the address mode required by the given situation. */
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstAddr, (void*)&mCoordInfo.coordAddress, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcAddr, (void*)&macShortAddress, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.dstPanId, (void*)&mCoordInfo.coordPanId, 2);
        FLib_MemCpy(&mpPacket->msgData.dataReq.srcPanId, (void*)&mCoordInfo.coordPanId, 2);
        mpPacket->msgData.dataReq.dstAddrMode = gAddrModeShortAddress_c;
        mpPacket->msgData.dataReq.srcAddrMode = gAddrModeShortAddress_c;
        mpPacket->msgData.dataReq.msduLength = mDataTxMsduLen_c;
        /* Request MAC level acknowledgement */
        mpPacket->msgData.dataReq.txOptions = gMacTxOptionsAck_c;
        /* Give the data packet a handle. The handle is
        returned in the MCPS-Data Confirm message. */
        mpPacket->msgData.dataReq.msduHandle = mMsduHandle++;
        /* Don't use security */      
        mpPacket->msgData.dataReq.securityLevel = gMacSecurityNone_c;

        /* Send the Data Request to the MCPS */
        (void)NWK_MCPS_SapHandler(mpPacket, mMacInstance);
        /* Prepare for another data buffer */
        mpPacket = NULL;
        mcPendingPackets++;
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
