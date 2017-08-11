/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWR_Platform.c
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
#include "PhyTypes.h"
#include "PhyTime.h"
#include "PWR_Configuration.h"
#include "PWRLib.h"
#include "PWR_Interface.h"

#include "TimersManager.h"
#include "SX123xDrv.h"
#include "Keyboard.h"
#include "SerialManager.h"

#include "fsl_smc_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_llwu_hal.h"

#include "fsl_lptmr_driver.h"
#include "TMR_Adapter.h"

#include "board.h"
#include "fsl_osa_ext.h"
#include "PWR_Platform.h"

/*****************************************************************************
 *                        PRIVATE TYPE DEFINITIONS                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the data types definitions: stuctures, unions,    *
 * enumerations, typedefs ...                                                *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
typedef void (*xcvr_ptr)(void);

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
extern const clock_manager_user_config_t g_defaultClockConfigRun;

/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
static uint8_t gWakeUpSource = 0;
static smc_power_mode_config_t mPWRP_Config;
static uint16_t gu16WakeUpTime = 0;
static const clock_manager_user_config_t g_defaultClockConfigVlprPWRP =
{
    .mcgConfig =
    {
        .mcg_mode           = kMcgModeBLPI,   // Work in BLPI mode.
        .irclkEnable        = true,  // MCGIRCLK enable.
        .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgIrcFast, // Select IRC4M.
        .fcrdiv             = 0U,    // FCRDIV is 0.

        .frdiv   = 0U,
        .drs     = kMcgDcoRangeSelLow,  // Low frequency range
        .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%

        .pll0EnableInFllMode        = false,  // PLL0 disable
        .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
        .prdiv0            = 0U,
        .vdiv0             = 0U,
    },
    .simConfig =
    {
        .pllFllSel = kClockPllFllSelFll, // PLLFLLSEL select FLL.
        .er32kSrc  = kClockEr32kSrcLpo,     // ERCLK32K selection, use LPO.
        .outdiv1   = 1U,
        .outdiv4   = 1U,
    },
    .oscerConfig =
    {
        .enable       = true,  // OSCERCLK enable.
        .enableInStop = false, // OSCERCLK disable in STOP mode.
    }
};

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
static xcvr_ptr pSendRadioToMode = NULL;
static void Disable_WakeUp(bool_t);
static void Enable_WakeUp(bool_t);
static void Enter_LPMode(power_modes_t mode, smc_stop_submode_t submode);
static void XCVR_Enter_Sleep ( void );
static void XCVR_Enter_Listen ( void );
static void XCVR_Enter_Standby ( void );
static void XCVR_Enter_Synthesizer ( void );
static void XCVR_Enter_RX ( void );
static void XCVR_Enter_TX ( void );

/*****************************************************************************
 *                           PUBLIC FUNCTIONS PROTOTYPES                     *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
extern void PWRLib_LLWU_Isr(void);


/*---------------------------------------------------------------------------
 * Name: PWRP_ConfigureXCVRMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRP_ConfigureXCVRMode(pwrp_xcvr_modes_t mode)
{
  switch(mode)
  {
  case pwrp_xcvr_rx:
    pSendRadioToMode = XCVR_Enter_RX;
    break;
  case pwrp_xcvr_tx:
    pSendRadioToMode = XCVR_Enter_TX;
    break;
  case pwrp_xcvr_sleep:
    pSendRadioToMode = XCVR_Enter_Sleep;
    break;
  case pwrp_xcvr_listen:
    pSendRadioToMode = XCVR_Enter_Listen;
    break;
  case pwrp_xcvr_synth:
    pSendRadioToMode = XCVR_Enter_Synthesizer;
    break;
  case pwrp_xcvr_standby:
    pSendRadioToMode = XCVR_Enter_Standby;
    break;
  default:
    pSendRadioToMode = NULL;
    break;
  }
}

/*---------------------------------------------------------------------------
 * Name: PWRP_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRP_Init(void)
{
  /*Init LPTMR*/
  lptmr_prescaler_user_config_t prescaler_config;
    
  LPTMR_Init(NULL);
  
  prescaler_config.prescalerBypass = 1;
  
  prescaler_config.prescalerClockSelect = (lptmr_prescaler_clock_select_t)LPTMR_SOURCE_LPO1KHZ;
  
  LPTMR_HAL_SetPrescalerMode(g_lptmrBase[gLptmrInstance_c], prescaler_config);  
  
  LPTMR_HAL_Disable(g_lptmrBase[gLptmrInstance_c]);
  
  /*Un-register modules from LLWU*/
  LLWU_HAL_SetExternalInputPinMode(LLWU, kLlwuExternalPinDisabled, gPWRLib_LLWU_WakeupPin_PTD6_c);
  LLWU_HAL_SetInternalModuleCmd(LLWU, (llwu_wakeup_module_t)gPWRLib_LLWU_WakeupModule_LPTMR_c, false);
  
  /* Clear external pins wakeup interrupts */
  LLWU_F1 = LLWU_F1; 
  LLWU_F2 = LLWU_F2; 
  
  /* LPTMR is wakeup source */
  if( LLWU_HAL_GetInternalModuleWakeupFlag(LLWU, (llwu_wakeup_module_t)gPWRLib_LLWU_WakeupModule_LPTMR_c) )
  {
    /* Clear LPTMR interrupt */
    LPTMR_HAL_ClearIntFlag(g_lptmrBase[gLptmrInstance_c]);
  }
  OSA_EXT_InstallIntHandler(LLWU_IRQn, PWRLib_LLWU_Isr);
  NVIC_SetPriority(LLWU_IRQn, 0x80 >> (8 - __NVIC_PRIO_BITS));
  NVIC_EnableIRQ(LLWU_IRQn);
}

/*---------------------------------------------------------------------------
 * Name: PWRP_UpdateLPTMRCount
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRP_UpdateLPTMRCount(uint16_t u16TimeInMs)
{
  gu16WakeUpTime = u16TimeInMs;
}

/*---------------------------------------------------------------------------
 * Name: PWRP_SetWakeUpSource
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t PWRP_SetWakeUpSource(uint8_t u8WUSource)
{
  if ( u8WUSource != LPTMR_WAKE_UP && u8WUSource != UART_WAKE_UP && u8WUSource != GPIO_WAKE_UP )
  {
    return PWRP_ERR;
  }
  gWakeUpSource = u8WUSource;
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: PWRP_GetWakeUpReason
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PWRP_GetWakeUpReason(void)
{
  if(PWRLib_MCU_WakeupReason.Bits.FromKeyBoard && gWakeUpSource == GPIO_WAKE_UP)
  {
    return GPIO_WAKE_UP;
  }
  if(PWRLib_MCU_WakeupReason.Bits.FromLPTMR && gWakeUpSource == LPTMR_WAKE_UP)
  {
    return LPTMR_WAKE_UP;
  }
  if(PWRLib_MCU_WakeupReason.Bits.FromUART && gWakeUpSource == UART_WAKE_UP)
  {
    return UART_WAKE_UP;
  }
  return UNKNOWN_WAKE_UP;
}
/*---------------------------------------------------------------------------
 * Name: Enter_LPMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Enter_LPMode(power_modes_t mode, smc_stop_submode_t submode)
{
  bool_t bLLWUWakeUp = FALSE;
  uint32_t temp = 0;
  
  OSA_EXT_InterruptDisable();
  
  /*disable systick*/
  temp = SysTick->CTRL & (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
  SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
  
  mPWRP_Config.powerModeName = mode;
  if(mode == kPowerModeLls || mode == kPowerModeVlls)
  {
    mPWRP_Config.stopSubMode = submode;
    bLLWUWakeUp = TRUE;
  }
  else
  {
    (void)submode;
    bLLWUWakeUp = FALSE;
  }
  /*enables wake up source based on previously applied configuration*/
  Enable_WakeUp(bLLWUWakeUp);
  /* configure MCG in BLPI mode */      
  CLOCK_SYS_SetConfiguration(&g_defaultClockConfigVlprPWRP);
  /* send radio in configured mode*/
  if(pSendRadioToMode != NULL)
  {
    pSendRadioToMode();
  }
  
  SMC_HAL_SetMode(SMC, &mPWRP_Config);
  
  /* checks source of wakeup and disables it */
  Disable_WakeUp(bLLWUWakeUp);
  /* configure Radio in idle mode */
  XCVR_Enter_Standby();
  
  /*if previous mode was vlpw then MCU is running in vlpr
  must switch back to run*/
  if(mode == kPowerModeVlpw)
  {
    mPWRP_Config.powerModeName = kPowerModeRun;
    SMC_HAL_SetMode(SMC, &mPWRP_Config);
  }
  /* configure MCG in PEE mode*/
  CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
  
  /* restore the state of SysTick */
  SysTick->CTRL |= temp;
  
  OSA_EXT_InterruptEnable();
}

/*---------------------------------------------------------------------------
 * Name: Enable_WakeUp
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Enable_WakeUp(bool_t bWakeFromLLWU)
{
  if(bWakeFromLLWU)
  {
    if(gWakeUpSource == LPTMR_WAKE_UP)
    {
      /* enable LPTMR as wakeup source for LLWU module */
      LLWU_HAL_SetInternalModuleCmd(LLWU, (llwu_wakeup_module_t)gPWRLib_LLWU_WakeupModule_LPTMR_c, true);
      LPTMR_HAL_SetCompareValue(g_lptmrBase[gLptmrInstance_c], gu16WakeUpTime);
      LPTMR_HAL_Enable(g_lptmrBase[gLptmrInstance_c]);
    }
    else if(gWakeUpSource == GPIO_WAKE_UP)
    {
      LLWU_HAL_SetExternalInputPinMode(LLWU, kLlwuExternalPinChangeDetect, gPWRLib_LLWU_WakeupPin_PTD6_c);
    }
  }
  else
  {
    if(gWakeUpSource == UART_WAKE_UP)
    {
      Serial_EnableLowPowerWakeup(gSerialMgrLpsci_c);
    }
    else if(gWakeUpSource == LPTMR_WAKE_UP)
    {
      LPTMR_HAL_SetCompareValue(g_lptmrBase[gLptmrInstance_c], gu16WakeUpTime);
      LPTMR_HAL_Enable(g_lptmrBase[gLptmrInstance_c]);
    }
    else if(gWakeUpSource == GPIO_WAKE_UP)
    {
      
    }
  }
}

/*---------------------------------------------------------------------------
 * Name: Disable_WakeUp
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static void Disable_WakeUp(bool_t bWakeFromLLWU)
{
  if(bWakeFromLLWU)
  {
    PWRLib_LLWU_UpdateWakeupReason();
    
    if(PWRLib_MCU_WakeupReason.Bits.FromKeyBoard)
    {
      LLWU_HAL_SetExternalInputPinMode(LLWU, kLlwuExternalPinDisabled, gPWRLib_LLWU_WakeupPin_PTD6_c);
    }
    else if(PWRLib_MCU_WakeupReason.Bits.FromLPTMR)
    {
      LPTMR_HAL_Disable(g_lptmrBase[gLptmrInstance_c]);
      LLWU_HAL_SetInternalModuleCmd(LLWU, (llwu_wakeup_module_t)gPWRLib_LLWU_WakeupModule_LPTMR_c, false);
    }
  }
  else
  {
    if(KBD_IsWakeUpSource())
    {
      PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = TRUE;
    }
    if(Serial_IsWakeUpSource(gSerialMgrLpsci_c))
    {
      PWRLib_MCU_WakeupReason.Bits.FromUART = TRUE;
      Serial_DisableLowPowerWakeup(gSerialMgrLpsci_c);
    }
    else if (gWakeUpSource == UART_WAKE_UP)
    {
      Serial_DisableLowPowerWakeup(gSerialMgrLpsci_c);
    }
    if(LPTMR_HAL_IsIntPending(g_lptmrBase[gLptmrInstance_c]))
    {
      LPTMR_HAL_ClearIntFlag(g_lptmrBase[gLptmrInstance_c]);
      LPTMR_HAL_Disable(g_lptmrBase[gLptmrInstance_c]);
      PWRLib_MCU_WakeupReason.Bits.FromLPTMR = TRUE;
    }
    else if(gWakeUpSource == LPTMR_WAKE_UP)
    {
      LPTMR_HAL_ClearIntFlag(g_lptmrBase[gLptmrInstance_c]);
      LPTMR_HAL_Disable(g_lptmrBase[gLptmrInstance_c]);
    }

  }
}

/*---------------------------------------------------------------------------
 * Name: Enter_RUN
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_RUN (void)
{
  OSA_EXT_InterruptDisable();
  
  XCVR_Enter_Standby();
  
  mPWRP_Config.powerModeName = kPowerModeRun;
  SMC_HAL_SetMode(SMC, &mPWRP_Config);
  
  CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
  OSA_EXT_InterruptEnable();
  
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_VLPR
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_VLPR ( void )
{
  OSA_EXT_InterruptDisable();
  
  CLOCK_SYS_SetConfiguration(&g_defaultClockConfigVlprPWRP);
  
  if(pSendRadioToMode)
  {
    pSendRadioToMode();
  }
  mPWRP_Config.powerModeName = kPowerModeVlpr;
  SMC_HAL_SetMode(SMC, &mPWRP_Config);
  
  OSA_EXT_InterruptEnable();
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_Wait
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_Wait ( void )
{
  PWRLib_MCU_WakeupReason.AllBits = 0;
  
  if(gWakeUpSource == NO_WAKE_UP)
  {
    return PWRP_ERR;
  }
  Enter_LPMode(kPowerModeWait, (smc_stop_submode_t)0);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_Stop
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_Stop ( void )
{
  if(gWakeUpSource == NO_WAKE_UP)
  {
    return PWRP_ERR;
  }
  Enter_LPMode(kPowerModeStop, (smc_stop_submode_t)0);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_VLPW
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_VLPW ( void )
{
  if(gWakeUpSource == NO_WAKE_UP)
  {
    return PWRP_ERR;
  }
  Enter_LPMode(kPowerModeVlpw, (smc_stop_submode_t)0);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_VLPS
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_VLPS ( void )
{
  if(gWakeUpSource == NO_WAKE_UP)
  {
    return PWRP_ERR;
  }
  Enter_LPMode(kPowerModeVlps, kSmcStopSub1);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_LLS
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_LLS  ( void )
{
  PWRLib_MCU_WakeupReason.AllBits = 0;
  
  if(gWakeUpSource == NO_WAKE_UP || gWakeUpSource == UART_WAKE_UP)
  {
    return PWRP_ERR;
  }
  
  Enter_LPMode(kPowerModeLls, kSmcStopSub1);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: Enter_VLLS
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
pwrp_status_t Enter_VLLS (uint8_t u8SubMode)
{
  PWRLib_MCU_WakeupReason.AllBits = 0;
  
  if(gWakeUpSource == NO_WAKE_UP || gWakeUpSource == UART_WAKE_UP)
  {
    return PWRP_ERR;
  }
  Enter_LPMode(kPowerModeVlls, (smc_stop_submode_t)u8SubMode);
  return PWRP_OK;
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_Sleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_Sleep ( void )
{
  XCVRDrv_RadioSleepReq();
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_Listen
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_Listen ( void )
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, (OpMode_Listen_On | OpMode_StandBy));
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_Standby
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_Standby ( void )
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, (OpMode_Listen_Abort | OpMode_StandBy));
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, OpMode_StandBy);
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_Synthesizer
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_Synthesizer ( void )
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, OpMode_FreqSynt);
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_RX
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_RX ( void )
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, OpMode_Receiver);
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Enter_TX
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVR_Enter_TX ( void )
{
  XCVRDrv_WriteRegister(XCVR_Reg_OpMode, OpMode_Transmitter);
}