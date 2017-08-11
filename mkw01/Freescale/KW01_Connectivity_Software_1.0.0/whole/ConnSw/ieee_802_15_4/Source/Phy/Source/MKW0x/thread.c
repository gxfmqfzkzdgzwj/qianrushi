#include "cmsis_os2.h"
#include "thread.h"
#include "Phy.h"
#include "AspInterface.h"
#include "stdint.h"

//#define MSGQUEUE_OBJECTS      16                                   // number of Message Queue Objects
osThreadId_t tid_Send_Command,tid_clock;
osMessageQueueId_t mid_MsgQueue; 
int main_thread (void) {
      tid_Send_Command = osThreadNew(Send_Command, NULL, NULL);
	    tid_clock =	osThreadNew(clock, NULL, NULL);    
      osThreadFlagsSet(tid_Send_Command, 0x0001);          /* set signal to tid_Send_Command thread   */
      osDelay(osWaitForever);      
}



void Send_Command(void *argument)
{
	for (;;) 
  {
			osThreadFlagsWait (0x00000001U, osFlagsWaitAny, osWaitForever);	/* wait for an event flag 0x0001 */
/*
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
			osMessageQueuePut (mid_MsgQueue,ptr, NULL, NULL);*/
	    signal_func(tid_Phy_Task,0x00FFFFFF);
      osThreadYield ();
	
	}
	
}
void signal_func (osThreadId_t tid,int32_t signal)  {
     osThreadFlagsSet(tid_clock, 0x0100);           /* set signal to clock thread    */
     osDelay(500);                             /* delay 500ms                   */
     osThreadFlagsSet(tid_clock, 0x0100);           /* set signal to clock thread    */
     osDelay(500);                             /* delay 500ms                   */
     osThreadFlagsSet(tid, signal);                 /* set signal to thread 'thread' */
     osDelay(500);                             /* delay 500ms                   */
}


