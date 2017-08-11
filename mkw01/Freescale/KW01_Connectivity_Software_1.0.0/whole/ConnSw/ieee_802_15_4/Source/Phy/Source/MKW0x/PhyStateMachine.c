/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyStateMachine.c
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
#include "MemManager.h"
#include "FunctionLib.h"
#include "Messaging.h"
#include "Panic.h"
#include "SX123xDrv.h"
#include "PhyInterface.h"
#include "Phy.h"
#include "PhyISR.h"
#include "PhyTime.h"
#include "PhyPib.h"
#include "cmsis_os2.h"

#include "AspInterface.h"
//#include "fsl_os_abstraction.h"
#include "thread.h"
#define MSGQUEUE_OBJECTS      16      
/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#ifdef gSrcTask_d
#undef gSrcTask_d
#endif

#define gSrcTask_d PHY

#define mPhyTxOverheadSetupTime_c       (12)        /* [sym] */
#define mPhyMaxIdleRxDuration_c         (0xF00000)  /* [sym] */
#define gPhyEndtimeGuardTicks_c         (600)       /* [ticks] */

#define gRadio_AnyEventsMask_c            (   gRadio_PdDataConfirm_c \
                                            | gRadio_PdDataIndication_c \
                                            | gRadio_PlmeCcaConfirm_c \
                                            | gRadio_PlmeEdConfirm_c \
                                            | gRadio_RxTimeoutIndication_c \
                                            | gRadio_StartEventIndication_c \
                                            | gRadio_RxSfdDetect_c \
                                            | gRadio_FilterFailRx_c)

#define gMAC_AnyEventsMask_c              (   gMAC_PdDataRequest_c \
                                            | gMAC_PlmeCCaRequest_c \
                                            | gMAC_PlmeEdRequest_c )

const uint8_t gUseRtos_c;

/*****************************************************************************
 *                             PRIVATE TYPE DEFINITIONS                      *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
typedef enum{
    gRadio_PdDataConfirm_c                    = (1<<0),
    gRadio_PdDataIndication_c                 = (1<<1),
    gRadio_PlmeCcaConfirm_c                   = (1<<2),
    gRadio_PlmeEdConfirm_c                    = (1<<3),
    gRadio_RxTimeoutIndication_c              = (1<<4),
    gRadio_StartEventIndication_c             = (1<<5),    
    gRadio_RxSfdDetect_c                      = (1<<6),
    gRadio_FilterFailRx_c                     = (1<<7),    
    gRadio_WaitTimeoutIndication_c            = (1<<8),
    gRadio_DummyEvent_c                       = (1<<9),
}Radio_eventType_t;

typedef enum{
    gMAC_AsyncReq_c                           = (1<<11)
}MAC_eventType_t;

osThreadId_t tid_Phy_Task;
extern osMessageQueueId_t mid_MsgQueue; 
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
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: Phy_PhySubGHzTask
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Phy_Task
(
  void* taskParam
);

/*---------------------------------------------------------------------------
 * Name: Phy_TaskInitialize
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Phy_TaskInitialize
(
  void
);

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePdDataReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePdDataReq
( 
  macToPdDataMessage_t * pMsg
);

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePlmeCcaReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePlmeCcaReq
( 
  macToPlmeMessage_t * pMsg
);

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePlmeEdReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePlmeEdReq
( 
  macToPlmeMessage_t * pMsg
);

/*---------------------------------------------------------------------------
 * Name: Phy_EnterIdle
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Phy_EnterIdle
( 
  void
);

static phyStatus_t PLME_MAC_SendMessage(plmeToMacMessage_t * msg);

static phyStatus_t PD_MAC_SendMessage(pdDataToMacMessage_t * msg);

/*****************************************************************************
 *                             PUBLIC FUNCTIONS                              *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
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

/* PHY RTOS objects */
EventGroupHandle_t     gPhyEvent;
TaskHandle_t gPhyTaskHandler;

osEventFlagsId_t evt_id; //新增加的，底下要用到
osThreadId_t 	thread_id;//新增加的，底下线程用到
uint32_t  rflags,ev;//新加入

/*const osThreadAttr_t thread1_attr = {
  .cb_mem  = &thread1_tcb,
  .cb_size = sizeof(thread1_tcb),
	.priority=gPhyTaskPriority_c;
	
};*/
//OSA_TASK_DEFINE(PHY, gPhyTaskStackSize_c );//如何将任务名称与其堆栈大小 综合到一个函数中
//定义任务函数以及句柄
Phy_PhyLocalStruct_t phyLocal;
uint8_t              rxData[gMaxPHYPacketSize_c];

pdDataReq_t          ackDataReq;
uint8_t              ackBuffer[3];
#if gSnifferCRCEnabled_d
bool_t               crcValid;
#endif
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
 * Name: Phy_Init()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_Init( void )
{
#if gFsciHostMacSupport_c
  /* Nothing to do if this a FSCI Host */
  return;
#else
//  osStatus_t status =osOK;
  
  /* XCVR, TMR, PHY ISR init */
  PhyHwInit();
  
  /* Ack Data Req init */
  ackDataReq.psduLength = 3; 
  ackDataReq.CCABeforeTx = gPhyNoCCABeforeTx_c;
  ackDataReq.slottedTx = gPhyUnslottedMode_c;
  ackDataReq.ackRequired = gPhyNoAckRqd_c;
  ackDataReq.pPsdu = (uint8_t *)&ackBuffer;
  
  /* PHY task and event init */
  //status = OSA_EventCreate(&gPhyEvent, kEventAutoClear);
   evt_id = osEventFlagsNew(NULL);	//这里创建新的事件，问题是不知道如何将事件与PhyEvent连接起来，过程中生成句柄函数
  //if( kStatus_OSA_Success != status )
/*	if(evt_id == NULL )//这里存在疑问  gPhyEvent不是osEventFlagsId_t
  {
    panic(0,0,0,0);
    return;
  }*/
	
  /* Initialize variables and structures that will be used by the PHY Task */
  Phy_TaskInitialize();
	
  /* The PHY instance is passed at task creaton */
 /* status = OSA_TaskCreate(Phy_Task, "PHY_Task", gPhyTaskStackSize_c, PHY_stack,
                          gPhyTaskPriority_c, 0, FALSE, &gPhyTaskHandler);//定义一个osThreadAttr_t类型的变量*/
	
	//将task改为新建的一个线程
	//    void *pvParameters, 指针用于一个参数 指向任务
 // if(osOK!= status )
	
   tid_Phy_Task = osThreadNew(Phy_Task, NULL,NULL);
	/*if((xTaskCreate(Phy_Task, "PHY_Task", gPhyTaskStackSize_c/sizeof(portSTACK_TYPE), NULL , gPhyTaskPriority_c, &gPhyTaskHandler))!= pdPASS)//问题 如何建立一个堆栈
  { 
    panic(0,0,0,0);
    return;
  }*/
#endif  
}

/*! *********************************************************************************
* \brief  Signal events to PHY task
*
* \param[in]  events  event flags to set
*
********************************************************************************** */
void PHY_EventSet(uint32_t events)
{
    //(void)OSA_EventSet(&gPhyEvent, events);
	osEventFlagsSet(evt_id, events);//改正后的
}

/*---------------------------------------------------------------------------
 * Name: BindToPHY()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
instanceId_t BindToPHY( instanceId_t macInstance )
{
  return 0;
}
 
/*---------------------------------------------------------------------------
 * Name: Phy_Init()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_Task(void* taskParam)
{
  phyStatus_t           status = gPhySuccess_c;
  phyMessageHeader_t   *pMsgIn;
  uint32_t              ev;
 // AppToAspMessage_t *pMsg; //////////////////////////////////////////////////
	uint8_t ptr[5];
 while(1)
  {

		 //PHY_EventSet(gRadio_PdDataConfirm_c);
		 xEventGroupSetBits ((EventGroupHandle_t)evt_id, (EventBits_t)gRadio_PdDataIndication_c);
		 rflags=osEventFlagsWait(evt_id, 0x00FFFFFF,osFlagsWaitAny, osWaitForever);
		
   //OSA_EventWait(&gPhyEvent, 0x00FFFFFF, FALSE, osWaitForever, &ev);
  //这里是改过的，但是不确定对不对，担心参数对应不上
  	
		ev=0x00FFFFFF&rflags;//这个也是新加的，不知道对不对，因为osEventFlagsWait 参数类型里不存在ev，根据函数里好像是这么定义的
	 	
		if( ev & gRadio_PdDataIndication_c )
    {
      pdDataToMacMessage_t *pDataInd = pvPortMalloc((sizeof(pdDataToMacMessage_t) + phyLocal.rxParams.psduLength));
      if( NULL != pDataInd )
      {
        pDataInd->msgType = gPdDataInd_c;
        /* Convert LQI - RSSI[dBm] = -RssiValue / 2 */
        pDataInd->msgData.dataInd.ppduLinkQuality = Phy_LqiConvert(phyLocal.rxParams.linkQuality >> 1);
        /* Convert timestamp from us to MAC symbols */
        Phy_TimeDivider((uint64_t*)&phyLocal.rxParams.timeStamp);
        pDataInd->msgData.dataInd.timeStamp = phyLocal.rxParams.timeStamp;
        pDataInd->msgData.dataInd.psduLength = phyLocal.rxParams.psduLength - 2;
        pDataInd->msgData.dataInd.pPsdu = (uint8_t *)(&pDataInd->msgData.dataInd) + sizeof(pdDataInd_t);
        FLib_MemCpy(pDataInd->msgData.dataInd.pPsdu, rxData, pDataInd->msgData.dataInd.psduLength);
#if gSnifferCRCEnabled_d      
        pDataInd->msgData.dataInd.crcValue = rxData[pDataInd->msgData.dataInd.psduLength];
        pDataInd->msgData.dataInd.crcValue |= (uint32_t)(rxData[pDataInd->msgData.dataInd.psduLength+1] << 8);
        pDataInd->msgData.dataInd.crcValid = crcValid;
#endif
        PD_MAC_SendMessage(pDataInd); 
      }
    }

    if( ev & gRadio_PdDataConfirm_c )
    {
      //pdDataToMacMessage_t *pDataCnf = MEM_BufferAlloc(sizeof(phyMessageHeader_t) + sizeof(pdDataCnf_t));
			pdDataToMacMessage_t *pDataCnf =pvPortMalloc (sizeof(phyMessageHeader_t)+sizeof(pdDataCnf_t));
     if( NULL != pDataCnf )
			 {
        pDataCnf->msgType = gPdDataCnf_c;
        if( phyLocal.flags.rxFramePending )
        {
          pDataCnf->msgData.dataCnf.status = gPhyFramePending_c;
        }
        else
        {
          pDataCnf->msgData.dataCnf.status = gPhySuccess_c;
        }
        PD_MAC_SendMessage(pDataCnf);
      }
    }

    if( ev & gRadio_PlmeCcaConfirm_c )
    {
      plmeToMacMessage_t *pPlmeCcaCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeCcaCnf )
      {
        pPlmeCcaCnf->msgType = gPlmeCcaCnf_c;
        pPlmeCcaCnf->msgData.ccaCnf.status = phyLocal.channelParams.channelStatus;
        PLME_MAC_SendMessage(pPlmeCcaCnf);
      }
    }

    if( ev & gRadio_PlmeEdConfirm_c )
    {
      plmeToMacMessage_t *pPlmeEdCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeEdCnf )
      {
        pPlmeEdCnf->msgType                     = gPlmeEdCnf_c;
        pPlmeEdCnf->msgData.edCnf.status        = gPhySuccess_c;
        pPlmeEdCnf->msgData.edCnf.energyLeveldB = phyLocal.channelParams.energyLeveldB;
        pPlmeEdCnf->msgData.edCnf.energyLevel   = Phy_GetEnergyLevel(pPlmeEdCnf->msgData.edCnf.energyLeveldB);
        PLME_MAC_SendMessage(pPlmeEdCnf);
      }
    }

    if( ev & gRadio_RxTimeoutIndication_c )
    {
      plmeToMacMessage_t *pPlmeTimeoutInd = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeTimeoutInd )
	  {
	     pPlmeTimeoutInd->msgType = gPlmeTimeoutInd_c;
		   PLME_MAC_SendMessage(pPlmeTimeoutInd);
	  }      
    }

#ifdef MAC_PHY_DEBUG
    if( ev & gRadio_StartEventIndication_c )
    {
      plmeToMacMessage_t *pPlmeCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeCnf )
      {
        pPlmeCnf->msgType = gPlme_StartEventInd_c;
        PLME_MAC_SendMessage(pPlmeCnf);        
      }
    }

    if( ev & gRadio_RxSfdDetect_c )
    {
      plmeToMacMessage_t *pPlmeCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeCnf )
      {
        pPlmeCnf->msgType = gPlme_RxSfdDetectInd_c;
        PLME_MAC_SendMessage(pPlmeCnf);      
      }
    }

    if( ev & gRadio_FilterFailRx_c )
    {
      plmeToMacMessage_t *pPlmeCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
      if( NULL != pPlmeCnf )
      {
        pPlmeCnf->msgType = gPlme_FilterFailInd_c;
        PLME_MAC_SendMessage(pPlmeCnf);
      }
    }
#endif

    /* handling messages from upper layer */    
    while( MSG_Pending(&phyLocal.macPhyInputQueue) )
    {
      if( PhyIsIdleRx(0) )
      {
        PhyPlmeForceTrxOffRequest();
      }
      else if( gIdle_c != PhyGetSeqState() )
      {
        break;
      }
      phyLocal.flags.idleRx = 0;	   
		 		 
      /* PHY doesn't free dynamic alocated messages! */
      pMsgIn = MSG_DeQueue( &phyLocal.macPhyInputQueue );
	  phyLocal.currentMacInstance = pMsgIn->macInstance;              
      
      switch( pMsgIn->msgType )
      {
        case gPdDataReq_c:
          status = Phy_HandlePdDataReq( (macToPdDataMessage_t *) pMsgIn );                   
          break;  
        case gPlmeCcaReq_c:
          status = Phy_HandlePlmeCcaReq( (macToPlmeMessage_t *) pMsgIn );
          break; 
        case gPlmeEdReq_c:
          status = Phy_HandlePlmeEdReq( (macToPlmeMessage_t *) pMsgIn );
          break;
        default:
          status = gPhyInvalidPrimitive_c;
      }      
	  
	  if( gPhySuccess_c != status )
	  {
         plmeToMacMessage_t * pPhyCnf = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));
         if( NULL != pPhyCnf )
		 {
		    switch( pMsgIn->msgType )
			{
			   case gPdDataReq_c:
                   ((pdDataToMacMessage_t*)pPhyCnf)->msgType = gPdDataCnf_c;
                   ((pdDataToMacMessage_t*)pPhyCnf)->msgData.dataCnf.status = status;
				break;
			   case gPlmeCcaReq_c:
                   pPhyCnf->msgType = gPlmeCcaCnf_c;
                   pPhyCnf->msgData.ccaCnf.status = status;
				break;
			   case gPlmeEdReq_c:
                   pPhyCnf->msgType = gPlmeEdCnf_c;
                   pPhyCnf->msgData.edCnf.status = status;
				break;
			   default:
                   pPhyCnf->msgType = gPlmeTimeoutInd_c;
			}
			
            if( pMsgIn->msgType == gPdDataReq_c )
            {
               PD_MAC_SendMessage((pdDataToMacMessage_t*)pPhyCnf);
            }
            else
            {
               PLME_MAC_SendMessage(pPhyCnf);
		    }
         }
	  }
	  else
	  {
	     break;
	  }
	}
	
    /* Check if PHY can enter Idle state */    
    if( gIdle_c == PhyGetSeqState() )
    {
      Phy_EnterIdle();
    }
	
    /* For BareMetal break the while(1) after 1 run */
    if( gUseRtos_c == 0 )
    {
//      break;
    }
	
			mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint8_t), NULL); 
	    if (!mid_MsgQueue) {
              ; // Message Queue object not created, handle failure
      } 
	   // AppToAspMessage_t *pMsg;
			//pMsg->msgType = aspMsgTypeTelecSendRawData_c;
			uint8_t ptr[]="12345";
			//uint8_t *data=p;
		//	*pMsg->msgData.aspTelecSendRawData.dataPtr = p;
			
		//	osMessageQueuePut (mid_MsgQueue,*pMsg, NULL, NULL);
		 // osMessageQueueGet(mid_MsgQueue, pMsg, NULL, NULL);  // wait for message
	  	osMessageQueueGet(mid_MsgQueue, ptr, NULL, NULL);
	    ASP_TelecSendRawData(ptr);
	    signal_func(tid_Phy_Task,0x00FFFFFF);
		
  }
}

/*---------------------------------------------------------------------------
 * Name: Phy_TaskInitialize()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Phy_TaskInitialize( void )
{
  phyLocal.flags.mask = 0x00;

  /* Prepare input queues */
  MSG_InitQueue( &phyLocal.macPhyInputQueue );

  PhyIsrPassRxParams( NULL );

  /* Clear indirect queue */
  phyLocal.phyUnavailableQueuePos = 0;
}

/*---------------------------------------------------------------------------
 * Name: Phy_RegisterSapHandlers()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_RegisterSapHandlers( PD_MAC_SapHandler_t pPD_MAC_SapHandler, 
                              PLME_MAC_SapHandler_t pPLME_MAC_SapHandler, 
                              instanceId_t instanceId )
{
  phyLocal.PD_MAC_SapHandler = pPD_MAC_SapHandler;
  phyLocal.PLME_MAC_SapHandler = pPLME_MAC_SapHandler;
}

/*---------------------------------------------------------------------------
 * Name: MAC_PD_SapHandler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t MAC_PD_SapHandler(macToPdDataMessage_t *pMsg, instanceId_t phyInstance)
{
  phyStatus_t result = gPhySuccess_c;

  if( NULL == pMsg )
  {
     return gPhyInvalidParameter_c;
  }
  
  switch( pMsg->msgType )
  {
    case gPdIndQueueInsertReq_c:
      result = PhyPp_IndirectQueueInsert( pMsg->msgData.indQueueInsertReq.index, pMsg->msgData.indQueueInsertReq.checksum, phyInstance );
    break;
    case gPdIndQueueRemoveReq_c:
      result = PhyPp_RemoveFromIndirect( pMsg->msgData.indQueueRemoveReq.index, phyInstance );
    break;
    case gPdDataReq_c:
      MSG_Queue(&phyLocal.macPhyInputQueue, pMsg);
      PHY_EventSet(gMAC_AsyncReq_c);
    break;
    default:
      result = gPhyInvalidPrimitive_c;
  }
  return result;  
}

/*---------------------------------------------------------------------------
 * Name: MAC_PLME_SapHandler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t MAC_PLME_SapHandler(macToPlmeMessage_t * pMsg, instanceId_t phyInstance)
{
  phyStatus_t result = gPhySuccess_c;

  if( NULL == pMsg )
  {
     return gPhyInvalidParameter_c;
  }

  switch( pMsg->msgType )
  {
    case gPlmeEdReq_c:
    case gPlmeCcaReq_c:
      MSG_Queue(&phyLocal.macPhyInputQueue, pMsg);
      PHY_EventSet(gMAC_AsyncReq_c);
      break;
    case gPlmeSetReq_c:      
        PhyPlmeForceTrxOffRequest();            
        result = PhyPlmeSetPIBRequest(pMsg->msgData.setReq.PibAttribute, pMsg->msgData.setReq.PibAttributeValue, 0, phyInstance);
        Radio_Phy_DummyEvent(phyInstance);
      break;
    case gPlmeGetReq_c:      
        result = PhyPlmeGetPIBRequest( pMsg->msgData.getReq.PibAttribute, pMsg->msgData.getReq.pPibAttributeValue, 0, phyInstance);
      break;
    case gPlmeSetTRxStateReq_c:      
        if(gPhySetRxOn_c == pMsg->msgData.setTRxStateReq.state)
        {               
          if( PhyIsIdleRx(0) )
          {
             PhyPlmeForceTrxOffRequest();
             phyLocal.flags.idleRx = 0;
          }
          else if( gIdle_c != PhyGetSeqState() )
          {
             return gPhyBusy_c;
          }

          Phy_SetSequenceTiming(pMsg->msgData.setTRxStateReq.startTime,
                                pMsg->msgData.setTRxStateReq.rxDuration,
                                gRX_c, phyInstance);

          result = PhyPlmeRxRequest(pMsg->msgData.setTRxStateReq.slottedMode, (phyRxParams_t *) &phyLocal.rxParams);
        }
        else if (gPhyForceTRxOff_c == pMsg->msgData.setTRxStateReq.state)
        {
           PhyPlmeForceTrxOffRequest();
        }      
      break;
    default:      
        result = gPhyInvalidPrimitive_c;     
      break;
  }

  return result;
}

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePdDataReq()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePdDataReq( macToPdDataMessage_t * pMsg )
{
  phyStatus_t status = gPhySuccess_c;  

  if( NULL == pMsg->msgData.dataReq.pPsdu )
  {
    return gPhyInvalidParameter_c;
  } 
   
  Phy_SetSequenceTiming( pMsg->msgData.dataReq.startTime, 
                         pMsg->msgData.dataReq.txDuration, 
                         ((pMsg->msgData.dataReq.CCABeforeTx != gPhyNoCCABeforeTx_c) ? gCCA_c : gTX_c),
                         0 );
  
  status = PhyPdDataRequest(&pMsg->msgData.dataReq , 
                            &phyLocal.rxParams, 
                            &phyLocal.txParams);

  if(gPhySuccess_c != status)
  {
    PhyTimeDisableEventTrigger();
    PhyTimeDisableEventTimeout();
  }

  return status;
}

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePlmeCcaReq()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePlmeCcaReq( macToPlmeMessage_t * pMsg )
{
  phyStatus_t status = gPhySuccess_c;
   
  Phy_SetSequenceTiming( gPhySeqStartAsap_c, gPhyPib.mPIBphyCCADuration, gRX_c, 0 );

  status = PhyPlmeCcaEdRequest(gPhyCCAMode1_c, gPhyContCcaDisabled);

  return status;
}

/*---------------------------------------------------------------------------
 * Name: Phy_HandlePlmeEdReq()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t Phy_HandlePlmeEdReq( macToPlmeMessage_t * pMsg )
{
  phyStatus_t status = gPhySuccess_c;  
  phyTime_t   startTime;  
  
  phyLocal.startTime = pMsg->msgData.edReq.startTime;
  
  if (pMsg->msgData.edReq.startTime != gPhySeqStartAsap_c) 
  {        
    startTime = pMsg->msgData.edReq.startTime;
   
    Phy_TimeMultiplicator(&startTime);
    startTime -= Phy_XCVRRxWarmupTime();   
    PhyTimeSetEventTrigger(startTime);       
  }
  status = PhyPlmeCcaEdRequest(gPhyEnergyDetectMode_c, gPhyContCcaDisabled);

  return status;
}

/*---------------------------------------------------------------------------
 * Name: Phy_SetSequenceTiming()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_SetSequenceTiming(phyTime_t startTime, uint32_t seqDuration, uint8_t nextState, instanceId_t instanceId)
{
  phyTime_t endTime;
  phyTime_t durationTicks = seqDuration;

  vPortEnterCritical();
  
  if( gPhySeqStartAsap_c == startTime )
  {
    phyLocal.startTime = startTime;
    PhyTimeReadClock(&endTime);
  }
  else
  {    
    /* When TSCH is enabled start time is passed in us instead of MAC symbols */
    if( phyLocal.flags.tschEnabled )
    {
      startTime = TIME_US_TO_TICKS(startTime);
      endTime = startTime; /* ticks */
    }
    else
    {
      endTime = startTime; /* MAC symbols */
      Phy_TimeMultiplicator(&startTime);
    }
    
    /* start time is stored in ticks here */
    phyLocal.startTime = startTime;
    
    startTime -= (nextState == gTX_c) ? Phy_XCVRTxWarmupTime() : Phy_XCVRRxWarmupTime();
    PhyTimeSetEventTrigger(startTime);    
  }

  if( phyLocal.flags.tschEnabled )
  {
    /* endTime is in ticks */
    if( gRX_c == nextState )
    {
      /* Rx duration is passed in us */
      endTime += TIME_US_TO_TICKS(durationTicks);
    }
    else
    {
      /* duration is in MAC symbols */
      Phy_TimeMultiplicator(&durationTicks);
      endTime += durationTicks;
    }
  }
  else
  {
    /* endTime and duration are in MAC symbols */
    endTime += seqDuration;    
    Phy_TimeMultiplicator(&endTime);
  }    

  endTime += gPhyEndtimeGuardTicks_c;
  
  if( gPhySeqStartAsap_c == startTime )
  {
    endTime += (nextState == gTX_c) ? Phy_XCVRTxWarmupTime() : Phy_XCVRRxWarmupTime();
  }
  
  PhyTimeSetEventTimeout(&endTime);
  
 portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PdDataConfirm()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PdDataConfirm(instanceId_t instanceId, bool_t framePending)
{  
  PhyTimeDisableEventTimeout();
  
  phyLocal.flags.rxFramePending = framePending;
  
  PHY_EventSet(gRadio_PdDataConfirm_c);
  
  if( framePending && (phyLocal.maxFrameWaitTime > 0) )
  {
    /* Restart Rx asap if an ACK with FP=1 is received */    
    Phy_SetSequenceTiming( gPhySeqStartAsap_c, phyLocal.maxFrameWaitTime, gRX_c, 0 );
    PhyPlmeRxRequest( gPhyUnslottedMode_c, (phyRxParams_t *) &phyLocal.rxParams );
  }
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PdDataIndication()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PdDataIndication(instanceId_t instanceId)
{
  PhyTimeDisableEventTimeout();
  
  PHY_EventSet(gRadio_PdDataIndication_c);
}

///*---------------------------------------------------------------------------
// * Name: Radio_Phy_TimeWaitTimeoutIndication()
// * Description: -
// * Parameters: -
// * Return: -
// *---------------------------------------------------------------------------*/
//void Radio_Phy_TimeWaitTimeoutIndication(instanceId_t instanceId)
//{
//  PHY_EventSet(gRadio_WaitTimeoutIndication_c);         
//}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_DummyEvent()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_DummyEvent(instanceId_t instanceId)
{
  PHY_EventSet(gRadio_DummyEvent_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_TimeRxTimeoutIndication()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_TimeRxTimeoutIndication(instanceId_t instanceId)
{
  PhyPlmeForceTrxOffRequest();
  
  PHY_EventSet(gRadio_RxTimeoutIndication_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_TimeStartEventIndication()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_TimeStartEventIndication(instanceId_t instanceId)
{
  PHY_EventSet(gRadio_StartEventIndication_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PlmeCcaConfirm()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PlmeCcaConfirm(phyStatus_t phyChannelStatus, instanceId_t instanceId)
{
  PhyTimeDisableEventTimeout();
  
  phyLocal.channelParams.channelStatus = phyChannelStatus;

  PHY_EventSet(gRadio_PlmeCcaConfirm_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PlmeEdConfirm()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PlmeEdConfirm(uint8_t energyLeveldB, instanceId_t instanceId)
{
  PhyTimeDisableEventTimeout();
  
  phyLocal.channelParams.energyLeveldB = energyLeveldB;
  
  PHY_EventSet(gRadio_PlmeEdConfirm_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PlmeRxSfdDetect()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PlmeRxSfdDetect(instanceId_t instanceId, uint32_t param)
{
  PHY_EventSet(gRadio_RxSfdDetect_c);
}

/*---------------------------------------------------------------------------
 * Name: Radio_Phy_PlmeFilterFailRx()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Radio_Phy_PlmeFilterFailRx(instanceId_t instanceId)
{
  PHY_EventSet(gRadio_FilterFailRx_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetCslRxEnabled()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPlmeSetCslRxEnabled( bool_t cslRxEnabled, instanceId_t instanceId )
{
  phyLocal.flags.cslRxEnabled = cslRxEnabled;   
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetTschEnabled()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPlmeSetTschEnabled( bool_t tschEnabled, instanceId_t instanceId )
{
  phyLocal.flags.tschEnabled = tschEnabled;   
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetFrameWaitTime()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPlmeSetFrameWaitTime( uint32_t time, instanceId_t instanceId )
{
  phyLocal.maxFrameWaitTime = time;
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetRxOnWhenIdle()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyPlmeSetRxOnWhenIdle( bool_t state, instanceId_t instanceId )
{
  phyLocal.flags.rxOnWhenIdle = state;
  if( PhyIsIdleRx(0) )        
  {
    PhyPlmeForceTrxOffRequest();
    phyLocal.flags.idleRx = 0;
  }
  PHY_EventSet(gMAC_AsyncReq_c);
}

/*---------------------------------------------------------------------------
 * Name: Phy_EnterIdle()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_EnterIdle( void )
{
  if( (TRUE == phyLocal.flags.rxOnWhenIdle) &&
      (FALSE == phyLocal.flags.cslRxEnabled) &&
      (FALSE == phyLocal.flags.tschEnabled) )
  {       
    phyLocal.flags.idleRx = 1;   
    
    Phy_SetSequenceTiming( gPhySeqStartAsap_c, mPhyMaxIdleRxDuration_c, gRX_c, 0 ); 
    (void)PhyPlmeRxRequest( gPhyUnslottedMode_c, (phyRxParams_t *) &phyLocal.rxParams );
  }
  else
  {   
    phyLocal.flags.idleRx = 0;    
  }
}

/*---------------------------------------------------------------------------
 * Name: PhyIsIdleRx()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PhyIsIdleRx( instanceId_t instanceId )
{
  if( phyLocal.flags.idleRx && (gRX_c == PhyGetSeqState()))
    return TRUE;

  return FALSE;
}

/*---------------------------------------------------------------------------
 * Name: PLME_MAC_SendMessage()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t PLME_MAC_SendMessage(plmeToMacMessage_t * msg)
{
  return phyLocal.PLME_MAC_SapHandler(msg, phyLocal.currentMacInstance);
}

/*---------------------------------------------------------------------------
 * Name: PD_MAC_SendMessage()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyStatus_t PD_MAC_SendMessage(pdDataToMacMessage_t * msg)
{
  return phyLocal.PD_MAC_SapHandler(msg, phyLocal.currentMacInstance);
}

#ifdef gListenSupported_c
/*---------------------------------------------------------------------------
 * Name: PhySetRxToListen()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyStatus_t PhySetRxToListen(bool_t listenOn, instanceId_t instanceId )
{
  if(gIdle_c != PhyGetSeqState())
    return gPhyBusy_c;
  
  phyLocal.flags.rxIsListen = listenOn;
  
  return gPhySuccess_c;
}
#endif
