#include "AspInterface.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "PhyInterface.h"
#include "cmsis_os2.h" 
#include "system_MKW01Z4.h"// ARM::CMSIS:RTOS:Keil RTX
#include "thread.h"

/* KSDK */

int main()
{
	 SystemCoreClockUpdate();
	 osKernelInitialize();
   
	 Phy_Init();
	 Asp_EnableXCVRInterrupts();
  
//	 osThreadNew(main_thread, NULL, NULL);
	// 	 main_thread();
	 osKernelStart();	
	return 0;	
}





