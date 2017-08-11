#ifndef __THREAD__
#define __THREAD__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"                   // ARM::CMSIS:RTOS:Keil RTX


extern osThreadId_t tid_Send_Command, tid_clock;
void clock (void *argument);
void Send_Command(void *argument);
void signal_func (osThreadId_t tid,int32_t signal) ;
int main_thread (void);
//void main_thread(void *argument);

#ifdef __cplusplus
}
#endif

#endif
