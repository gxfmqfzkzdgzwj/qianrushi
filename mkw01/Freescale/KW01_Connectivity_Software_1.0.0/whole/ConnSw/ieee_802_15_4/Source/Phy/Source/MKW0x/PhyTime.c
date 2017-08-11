/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyTime.c
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
#include "FunctionLib.h"
#include "Phy.h"
#include "PhyTime.h"
#include "PhyPib.h"
#include "PhyISR.h"
#include "SX123xDrv.h"

#include "board.h"
#include "fsl_tpm_driver.h"


#include "portmacro.h"
//#include "fsl_os_abstraction.h"

/*****************************************************************************
 *                             PUBLIC MACROS, DEFINITIONS                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define gPhyTimeMinSetupTimeSym_c   (10)
   
#define tpmBaseAddr g_tpmBase[gPhyTime_TPM_instance_c]

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

extern phyPib_t gPhyPib;

void (*gpfPhyTimeNotify)(void) = NULL;

/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

static volatile phyEvent64BitTimer_t mPhyEventTimeoutTimer;
static volatile phyEvent64BitTimer_t mPhyEventWaitTimeoutTimer;
static volatile phyEvent64BitTimer_t mPhyEventRxTimeoutTimer;
static volatile phyEvent64BitTimer_t mPhyEventTriggerTimer;
static volatile phyEvent64BitTimer_t mPhyCurrentTime;

static phyTimeEvent_t mPhyTimers[gMaxPhyTimers_c];
static phyTimeEvent_t *pNextEvent;

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
 *                                PRIVATE FUNCTIONS                          *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

static phyTimeEvent_t* PhyTime_GetNextEvent( void );

/*---------------------------------------------------------------------------
 * Name: PhyTimerInit()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerInit(void)
{
    static bool_t   init = FALSE;
    IRQn_Type       tmpIrqId = g_tpmIrqId[gPhyTime_TPM_instance_c];
    
    tpm_general_config_t tpmCfg = 
    {
        .isDBGMode = 0,
        .isGlobalTimeBase = 0,
        .isTriggerMode = 0,
        .isStopCountOnOveflow = 0,
        .isCountReloadOnTrig = 0,
        .triggerSource = kTpmTrigSel0,    
    };
    
    if( init == FALSE )
    {
        init = TRUE;
                 
        TPM_DRV_Init(gPhyTime_TPM_instance_c, &tpmCfg);
        
        TPM_HAL_SetClockDiv(tpmBaseAddr, kTpmDividedBy16);

        //OSA_InstallIntHandler(tmpIrqId, PhyTime_ISR);
			  //INT_SYS_InstallHandler(tmpIrqId, PhyTime_ISR);//这个也是改过的
   			
        NVIC_SetPriority(tmpIrqId, gPhyTime_IRQ_Priority_c >> (8 - __NVIC_PRIO_BITS));
        
        TPM_HAL_EnableTimerOverflowInt(tpmBaseAddr);
        
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, gPhyEventTriggerCh_c, TPM_CnSC_MSA_MASK);
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, gPhyEventTimeoutCh_c, TPM_CnSC_MSA_MASK);
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, gPhyWaitTimeoutCh_c, TPM_CnSC_MSA_MASK);
        TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, gPhyRxTimeoutCh_c, TPM_CnSC_MSA_MASK);
        TPM_HAL_SetClockMode(tpmBaseAddr, kTpmClockSourceModuleClk);
    }
}
  
/*---------------------------------------------------------------------------
 * Name: PhyTimerStart()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerStart(void)
{
    TPM_HAL_SetClockMode(tpmBaseAddr, kTpmClockSourceModuleClk);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimerStop()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimerStop(void)
{
    TPM_HAL_SetClockMode(tpmBaseAddr, kTpmClockSourceNoneClk);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeReadClock()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeReadClock(phyTime_t *pRetClk)
{
    if( NULL == pRetClk )
    {
        return;
    }

    PhyTimeReadClockTicks(pRetClk);

    /* Return value in MAC symbols */
    Phy_TimeDivider(pRetClk);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeReadClockTicks()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeReadClockTicks(phyTime_t *pRetClk)
{
    uint16_t timerCnt;

    if( NULL == pRetClk )
        return;

    portENTER_CRITICAL();    
    timerCnt = (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);

    if( TPM_HAL_GetTimerOverflowStatus(g_tpmBase[gPhyTime_TPM_instance_c]) )
    {
        *pRetClk = mPhyCurrentTime.longAccess + (1 << gPhyTimeShift_c);
        *pRetClk += (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);
    }
    else
    {
        *pRetClk = mPhyCurrentTime.longAccess;
        *pRetClk += timerCnt;
    }

   portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeSyncClockTicks()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSyncClockTicks(phyTime_t ticks)
{
  portENTER_CRITICAL();
    
    mPhyCurrentTime.longAccess += ticks;

    /* Update compare values for active timers */
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyEventTriggerCh_c) )
    {
        PhyTimeSetEventTrigger(mPhyEventTriggerTimer.longAccess);
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyEventTimeoutCh_c) )
    {
        PhyTimeSetEventTimeout((phyTime_t*)&mPhyEventTimeoutTimer.longAccess);
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyWaitTimeoutCh_c) )
    {
        PhyTimeSetWaitTimeout((phyTime_t*)&mPhyEventWaitTimeoutTimer.longAccess);
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyRxTimeoutCh_c) )
    {
        PhyTimeSetRxTimeout((phyTime_t*)&mPhyEventRxTimeoutTimer.longAccess);
    }

   portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeSetEventTrigger()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSetEventTrigger(phyTime_t startTime)
{               
    phyTime_t currentTime;
    uint16_t  cntVal;
    
    mPhyEventTriggerTimer.longAccess = startTime;

  portENTER_CRITICAL();
    PhyTimeReadClockTicks(&currentTime);
    cntVal = (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);
   portEXIT_CRITICAL();    

    if( mPhyEventTriggerTimer.longAccess > currentTime )
    {
        cntVal += mPhyEventTriggerTimer.halfWordAccess[0] - 
                  ((phyEvent64BitTimer_t *)&currentTime)->halfWordAccess[0];
        
        TPM_HAL_SetChnCountVal(tpmBaseAddr, gPhyEventTriggerCh_c, cntVal);
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyEventTriggerCh_c);
        TPM_HAL_EnableChnInt(tpmBaseAddr, gPhyEventTriggerCh_c);
    }
    else
    {
        PhyTimeHandleEventTrigger();
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeDisableEventTrigger()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeDisableEventTrigger(void)
{    
    TPM_HAL_DisableChnInt(tpmBaseAddr, gPhyEventTriggerCh_c); 
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeSetEventTimeout()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSetEventTimeout(phyTime_t *pEndTime)
{             
    phyTime_t currentTime;
    uint16_t  cntVal;

    if( NULL == pEndTime )
        return;
    
    PhyTimeDisableEventTimeout();
    
    mPhyEventTimeoutTimer.longAccess = *pEndTime;

  portENTER_CRITICAL();
    PhyTimeReadClockTicks(&currentTime);
    cntVal = (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);
   portEXIT_CRITICAL();     

    if( mPhyEventTimeoutTimer.longAccess > currentTime )
    {
        cntVal += mPhyEventTimeoutTimer.halfWordAccess[0] - 
                  ((phyEvent64BitTimer_t *)&currentTime)->halfWordAccess[0];
        
        TPM_HAL_SetChnCountVal(tpmBaseAddr, gPhyEventTimeoutCh_c, cntVal);
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyEventTimeoutCh_c);
        TPM_HAL_EnableChnInt(tpmBaseAddr, gPhyEventTimeoutCh_c);
    }
    else
    {
        PhyTimeHandleEventTimeout();
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeDisableEventTimeout()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeDisableEventTimeout(void)
{
    TPM_HAL_DisableChnInt(tpmBaseAddr, gPhyEventTimeoutCh_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeSetWaitTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSetWaitTimeout(phyTime_t *pWaitTimeout)
{
    phyTime_t currentTime;
    uint16_t  cntVal;

    if( NULL == pWaitTimeout )
        return;
    
    PhyTimeDisableWaitTimeout();
    
    mPhyEventWaitTimeoutTimer.longAccess = *pWaitTimeout;

  portENTER_CRITICAL();
    PhyTimeReadClockTicks(&currentTime);
    cntVal = (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);
   portEXIT_CRITICAL();     

    if( mPhyEventWaitTimeoutTimer.longAccess > currentTime )
    {
        cntVal += mPhyEventWaitTimeoutTimer.halfWordAccess[0] - 
                  ((phyEvent64BitTimer_t *)&currentTime)->halfWordAccess[0];
        
        TPM_HAL_SetChnCountVal(tpmBaseAddr, gPhyWaitTimeoutCh_c, cntVal);
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyWaitTimeoutCh_c);
        TPM_HAL_EnableChnInt(tpmBaseAddr, gPhyWaitTimeoutCh_c);
    }
    else
    {
        PhyTimeHandleEventWaitTimeout(); 
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeDisableWaitTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeDisableWaitTimeout(void)
{
    TPM_HAL_DisableChnInt(tpmBaseAddr, gPhyWaitTimeoutCh_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeSetRxTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeSetRxTimeout(phyTime_t *pRxTimeout)
{
    phyTime_t currentTime;
    uint16_t  cntVal;
    
    if( NULL == pRxTimeout )
        return;

    PhyTimeDisableRxTimeout();
    
    mPhyEventRxTimeoutTimer.longAccess = *pRxTimeout;

  portENTER_CRITICAL();
    PhyTimeReadClockTicks(&currentTime);
    cntVal = (uint16_t)TPM_HAL_GetCounterVal(tpmBaseAddr);
   portEXIT_CRITICAL();     

    if( mPhyEventRxTimeoutTimer.longAccess > currentTime )
    {
        cntVal += mPhyEventRxTimeoutTimer.halfWordAccess[0] - 
                  ((phyEvent64BitTimer_t *)&currentTime)->halfWordAccess[0];

        TPM_HAL_SetChnCountVal(tpmBaseAddr, gPhyRxTimeoutCh_c, cntVal);
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyRxTimeoutCh_c);
        TPM_HAL_EnableChnInt(tpmBaseAddr, gPhyRxTimeoutCh_c);
    }
    else
    {
        PhyTimeHandleEventRxTimeout();
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeDisableRxTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeDisableRxTimeout(void)
{
    TPM_HAL_DisableChnInt(tpmBaseAddr, gPhyRxTimeoutCh_c);
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeGetEventTimeout
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyTime_t PhyTimeGetEventTimeout(void)
{
    return mPhyEventTimeoutTimer.longAccess;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_TimerInit()
 * Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
phyTimeStatus_t PhyTime_TimerInit( void (*cb)(void) )
{
    if( gpfPhyTimeNotify )
    {
        return gPhyTimeError_c;
    }

    gpfPhyTimeNotify = cb;
    FLib_MemSet(mPhyTimers, 0, sizeof(mPhyTimers));

    return gPhyTimeOk_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_GetTimestamp
 * Description: -
 * Parameters: -
 * Return: 64bit timestamp value expressed in us
 *---------------------------------------------------------------------------*/
phyTime_t PhyTime_GetTimestampUs(void)
{
    phyTime_t time = 0;
    PhyTimeReadClockTicks(&time);
    return TIME_TICKS_TO_US(time);
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_GetTimestamp
 * Description: -
 * Parameters: -
 * Return: 64bit timestamp value to be used by the MAC Layer
 *---------------------------------------------------------------------------*/
phyTime_t PhyTime_GetTimestamp(void)
{
    phyTime_t time = 0;  
    PhyTimeReadClock((phyTime_t*)&time);
    return time;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_ScheduleEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyTimeTimerId_t PhyTime_ScheduleEvent( phyTimeEvent_t *pEvent )
{
    phyTimeTimerId_t tmr;

    /* Parameter validation */
    if( NULL == pEvent->callback )
    {
        return gInvalidTimerId_c;
    }

    /* Search for a free slot (slot 0 is reserved for the Overflow calback) */
  portENTER_CRITICAL();
    for( tmr=0; tmr<gMaxPhyTimers_c; tmr++ )
    {
        if( mPhyTimers[tmr].callback == NULL )
        {
            mPhyTimers[tmr] = *pEvent;
            break;
        }
    }
   portEXIT_CRITICAL();

    if( tmr >= gMaxPhyTimers_c )
        return gInvalidTimerId_c;

    /* Program the next event */
    if((NULL == pNextEvent) ||
       (NULL != pNextEvent  && mPhyTimers[tmr].timestamp < pNextEvent->timestamp))
    {
        PhyTime_Maintenance();
    }

    return tmr;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_CancelEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyTimeStatus_t PhyTime_CancelEvent( phyTimeTimerId_t timerId )
{
    if( (timerId >= gMaxPhyTimers_c) || 
        (NULL == mPhyTimers[timerId].callback) )
    {
        return gPhyTimeNotFound_c;
    }

  portENTER_CRITICAL();
    if( pNextEvent == &mPhyTimers[timerId] )
        pNextEvent = NULL;

    mPhyTimers[timerId].callback = NULL;
   portEXIT_CRITICAL();

    return gPhyTimeOk_c;
}


/*---------------------------------------------------------------------------
 * Name: PhyTime_CancelEventsWithParam
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyTimeStatus_t PhyTime_CancelEventsWithParam ( uint32_t param )
{
    uint32_t i;
    phyTimeStatus_t status = gPhyTimeNotFound_c;

    portENTER_CRITICAL();
    for( i=0; i<gMaxPhyTimers_c; i++ )
    {
        if( mPhyTimers[i].callback && (param == mPhyTimers[i].parameter) )
        {
            status = gPhyTimeOk_c;
            mPhyTimers[i].callback = NULL;
            if( pNextEvent == &mPhyTimers[i] )
                pNextEvent = NULL;
        }
    }
   portEXIT_CRITICAL();

    return status;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_RunCallback
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTime_RunCallback( void )
{
    uint32_t param;
    phyTimeCallback_t cb;

    if( pNextEvent )
    {
      portENTER_CRITICAL();

        param = pNextEvent->parameter;
        cb = pNextEvent->callback;
        pNextEvent->callback = NULL;
        pNextEvent = NULL;

       portEXIT_CRITICAL();

        cb(param);
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_Maintenance
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTime_Maintenance( void )
{
    phyTime_t       currentTime;
    phyTime_t       tempTimestamp;
    phyTimeEvent_t  *pEv;

    PhyTimeDisableWaitTimeout();

    while(1)
    {
      portENTER_CRITICAL();
        
        pEv = PhyTime_GetNextEvent();
        currentTime = PhyTime_GetTimestamp();
        
        /* Program next event if exists */
        if( pEv )
        {
            pNextEvent = pEv;
            
            if( pEv->timestamp > (currentTime + gPhyTimeMinSetupTimeSym_c) )
            {
                tempTimestamp = pEv->timestamp;
                Phy_TimeMultiplicator(&tempTimestamp);
                PhyTimeSetWaitTimeout(&tempTimestamp);
                pEv = NULL;
            }
        }

       portEXIT_CRITICAL();

        if( !pEv )
            break;

        PhyTime_RunCallback();
    }
    
}



/*---------------------------------------------------------------------------
 * Name: PhyTime_GetNextEvent
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static phyTimeEvent_t* PhyTime_GetNextEvent( void )
{
    phyTimeEvent_t *pEv = NULL;
    uint32_t i;

    /* Search for the next event to be serviced */
    for( i=0; i<gMaxPhyTimers_c; i++ )
    {
        if( NULL != mPhyTimers[i].callback )
        {
            if( NULL == pEv )
            {
                pEv = &mPhyTimers[i];
            }
            /* Check which event expires first */
            else if( mPhyTimers[i].timestamp < pEv->timestamp )
            {
                pEv = &mPhyTimers[i];
            }
        }
    }

    return pEv;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_ISR()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
//void PhyTime_ISR()
void TPM0_IRQHandler()	
{  
    if( TPM_HAL_GetTimerOverflowStatus(tpmBaseAddr) )
    {
        TPM_HAL_ClearTimerOverflowFlag(tpmBaseAddr);
        mPhyCurrentTime.longAccess += 1 << gPhyTimeShift_c;
    }
 
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyEventTriggerCh_c) && 
        TPM_HAL_GetChnStatus(tpmBaseAddr, gPhyEventTriggerCh_c) )  
    {
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyEventTriggerCh_c);
        PhyTimeEventTrigger_Handler();    
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyEventTimeoutCh_c) && 
        TPM_HAL_GetChnStatus(tpmBaseAddr, gPhyEventTimeoutCh_c) )  
    {
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyEventTimeoutCh_c);
        PhyTimeEventTimeout_Handler();    
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyWaitTimeoutCh_c) && 
        TPM_HAL_GetChnStatus(tpmBaseAddr, gPhyWaitTimeoutCh_c) )  
    {
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyWaitTimeoutCh_c);
        PhyTimeEventWaitTimeout_Handler();    
    }
    
    if( TPM_HAL_IsChnIntEnabled(tpmBaseAddr, gPhyRxTimeoutCh_c) && 
        TPM_HAL_GetChnStatus(tpmBaseAddr, gPhyRxTimeoutCh_c) )  
    {
        TPM_HAL_ClearChnInt(tpmBaseAddr, gPhyRxTimeoutCh_c);
        PhyTimeEventRxTimeout_Handler();    
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventTrigger_Handler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventTrigger_Handler(void)
{  
    phyTime_t currentTime;

    PhyTimeReadClockTicks(&currentTime);
    if( currentTime >= mPhyEventTriggerTimer.longAccess )
    {    
        PhyTimeDisableEventTrigger();
        PhyTimeHandleEventTrigger();
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventTimeout_Handler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventTimeout_Handler(void)
{
    phyTime_t currentTime;

    PhyTimeReadClockTicks(&currentTime);
    if( currentTime >= mPhyEventTimeoutTimer.longAccess )
    {
        PhyTimeDisableEventTimeout();
        PhyTimeHandleEventTimeout();   
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventWaitTimeout_Handler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventWaitTimeout_Handler(void)
{
    phyTime_t currentTime;

    PhyTimeReadClockTicks(&currentTime);
    if( currentTime >= mPhyEventWaitTimeoutTimer.longAccess )
    {
        PhyTimeDisableWaitTimeout();
        PhyTimeHandleEventWaitTimeout();   
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTimeEventRxTimeout_Handler()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PhyTimeEventRxTimeout_Handler(void)
{
    phyTime_t currentTime;
  
    PhyTimeReadClockTicks(&currentTime);
    if( currentTime >= mPhyEventRxTimeoutTimer.longAccess )
    {
        PhyTimeDisableRxTimeout();
        PhyTimeHandleEventRxTimeout();       
    }
}

/*---------------------------------------------------------------------------
 * Name: Phy_TimeMultiplicator()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_TimeMultiplicator(phyTime_t *time) 
{ 
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        TIME_MULTIPLICATOR_x20(*time);
    break;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        TIME_MULTIPLICATOR_x20(*time);
        break;
    case gPhyMode2_c:
        TIME_MULTIPLICATOR_x10(*time);       
        break;   
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        TIME_MULTIPLICATOR_x20(*time);
        break;
    case gPhyMode2_c:       
        TIME_MULTIPLICATOR_x6(*time);       
        break;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        TIME_MULTIPLICATOR_x20(*time);
        break;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        TIME_MULTIPLICATOR_x10(*time);       
        break;
    case gPhyMode3ARIB_c:
        TIME_MULTIPLICATOR_x5(*time);
        break;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        TIME_MULTIPLICATOR_x83(*time);
        break;
    case gPhyMode2_c:       
        TIME_MULTIPLICATOR_x50(*time);       
        break;
    case gPhyMode3_c:       
        TIME_MULTIPLICATOR_x10(*time);       
        break;        
#endif
    default:    
        break;     
    }
}

/*---------------------------------------------------------------------------
 * Name: Phy_TimeDivider()
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void Phy_TimeDivider(phyTime_t *time) 
{
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        TIME_DIVIDER_by20(*time);
        break;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        TIME_DIVIDER_by20(*time);
        break;
    case gPhyMode2_c:
        TIME_DIVIDER_by10(*time);       
        break;   
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        TIME_DIVIDER_by20(*time);
        break;
    case gPhyMode2_c:
        TIME_DIVIDER_by6(*time);       
        break;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        TIME_DIVIDER_by20(*time);
        break;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        TIME_DIVIDER_by10(*time);       
        break;
    case gPhyMode3ARIB_c:
        TIME_DIVIDER_by5(*time);
        break;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        TIME_DIVIDER_by83(*time);
        break;
    case gPhyMode2_c:
        TIME_DIVIDER_by50(*time);       
        break;
    case gPhyMode3_c:
        TIME_DIVIDER_by10(*time);       
        break;        
#endif
    default:    
        break;     
    }
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_GetSymbolsToUs
 * Description: -
 * Parameters: - 64bit timestamp value expressed in MAC symbols
 * Return: 64bit timestamp value expressed in us
 *---------------------------------------------------------------------------*/
phyTime_t PhyTime_GetSymbolsToUs(phyTime_t tsSymbols)
{
    phyTime_t tsUs = 0;
  
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        tsUs = tsSymbols * 20;
        break;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        tsUs = tsSymbols * 20;
        break;
    case gPhyMode2_c:
        tsUs = tsSymbols * 10;       
        break;   
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        tsUs = tsSymbols * 20;
        break;
    case gPhyMode2_c:
        tsUs = tsSymbols * 20 / 3;       
        break;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        tsUs = tsSymbols * 20;
        break;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        tsUs = tsSymbols * 10;       
        break;
    case gPhyMode3ARIB_c:
        tsUs = tsSymbols * 5;
        break;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        tsUs = tsSymbols * 2500 / 3;
        break;
    case gPhyMode2_c:
        tsUs = tsSymbols * 50;       
        break;   
    case gPhyMode3_c:
        tsUs = tsSymbols * 10;       
        break;        
#endif
    default:    
        break;     
    }
    
    return tsUs;
}

/*---------------------------------------------------------------------------
 * Name: PhyTime_GetUsToSymbols
 * Description: -
 * Parameters: - 64bit timestamp value expressed in us
 * Return: 64bit timestamp value expressed in MAC symbols
 *---------------------------------------------------------------------------*/
phyTime_t PhyTime_GetUsToSymbols(phyTime_t tsUs)
{
    phyTime_t tsSymbols = 0;
  
    switch( gPhyPib.mPIBphyCurrentMode )
    {
#if (gFreqBand_470__510MHz_d || gFreqBand_779__787MHz_d)
    case gPhyMode1_c:
        tsSymbols = tsUs / 20;
        break;    
#elif gFreqBand_863__870MHz_d
    case gPhyMode1_c:
        tsSymbols = tsUs / 20;
        break;
    case gPhyMode2_c:
        tsSymbols = tsUs / 10;       
        break;   
#elif gFreqBand_902__928MHz_d
    case gPhyMode1_c:
        tsSymbols = tsUs / 20;
        break;
    case gPhyMode2_c:
        tsSymbols = tsUs * 3 / 20;       
        break;
#elif gFreqBand_920__928MHz_d
    case gPhyMode1_c:
    case gPhyMode1ARIB_c:
        tsSymbols = tsUs / 20;
        break;
    case gPhyMode2_c:
    case gPhyMode2ARIB_c:
        tsSymbols = tsUs / 10;       
        break;
    case gPhyMode3ARIB_c:
        tsSymbols = tsUs / 5;
        break;
#elif gFreqBand_865__867MHz_d
    case gPhyMode1_c:
        tsSymbols = tsUs * 3 / 2500;
        break;
    case gPhyMode2_c:
        tsSymbols = tsUs / 50;       
        break;
    case gPhyMode3_c:
        tsSymbols = tsUs / 10;
        break;        
#endif
    default:    
        break;     
    }
    
    return tsSymbols;
}
