/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : pin_mux.c
**     Project     : USB_KW019032_K22F
**     Processor   : MKW01Z128CHN4
**     Component   : PinSettings
**     Version     : Component 1.2.0, Driver 1.4, CPU db: 3.00.000
**     Repository  : KSDK 1.3.0
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-28, 14:54, # CodeGen: 1
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file pin_mux.c
** @version 1.4
** @brief
**
*/         
/*!
**  @addtogroup pin_mux_module pin_mux module documentation
**  @{
*/         

/* MODULE pin_mux. */

#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_sim_hal.h"
#include "pin_mux.h"


/*FUNCTION**********************************************************************
*
* Function Name : configure_gpio_pins
* Description   : GPIO method sets registers according routing settings.
* Call this method code to route desired pins.
*END**************************************************************************/
void configure_gpio_pins(uint32_t instance)
{
  switch(instance) {
    case PORTA_IDX:                     /* PORTA_IDX */
      /* Affects PORTA_PCR12 register */
      PORT_HAL_SetMuxMode(PORTA,12UL,kPortMuxAsGpio);
      /* Affects PORTA_PCR13 register */
      PORT_HAL_SetMuxMode(PORTA,13UL,kPortMuxAsGpio);
      break;
    default:
      break;
  }
}
/*FUNCTION**********************************************************************
*
* Function Name : configure_i2c_pins
* Description   : I2C method sets registers according routing settings.
* Call this method code to route desired pins.
*END**************************************************************************/
void configure_i2c_pins(uint32_t instance)
{
  switch(instance) {
    case I2C0_IDX:                      /* I2C0_IDX */
      PORT_HAL_SetMuxMode(PORTD,2UL,kPortMuxAlt7);
      PORT_HAL_SetOpenDrainCmd(PORTD,2UL,true);
      PORT_HAL_SetMuxMode(PORTD,3UL,kPortMuxAlt7);
      PORT_HAL_SetOpenDrainCmd(PORTD,3UL,true);
      break;
    default:
      break;
  }
}
/*FUNCTION**********************************************************************
*
* Function Name : configure_spi_pins
* Description   : SPI method sets registers according routing settings.
* Call this method code to route desired pins.
*END**************************************************************************/
void configure_spi_pins(uint32_t instance)
{
  switch(instance) {
    case SPI0_IDX:                      /* SPI0_IDX */
      PORT_HAL_SetMuxMode(PORTD,0UL,kPortMuxAlt2);
      PORT_HAL_SetMuxMode(PORTD,1UL,kPortMuxAlt2);
      PORT_HAL_SetMuxMode(PORTC,6UL,kPortMuxAlt2);
      PORT_HAL_SetMuxMode(PORTC,7UL,kPortMuxAlt2);
      break;
    default:
      break;
  }
}
/*FUNCTION**********************************************************************
*
* Function Name : configure_uart_pins
* Description   : UART method sets registers according routing settings.
* Call this method code to route desired pins.
*END**************************************************************************/
void configure_uart_pins(uint32_t instance)
{
  switch(instance) {
    case UART1_IDX:                     /* UART1_IDX */
      /* Affects PORTC_PCR2 register */
      PORT_HAL_SetMuxMode(PORTC,2UL,kPortMuxAlt3);
      /* Affects PORTC_PCR1 register */
      PORT_HAL_SetMuxMode(PORTC,1UL,kPortMuxAlt3);
      /* Affects PORTC_PCR3 register */
      PORT_HAL_SetMuxMode(PORTC,3UL,kPortMuxAlt3);
      /* Affects PORTC_PCR4 register */
      PORT_HAL_SetMuxMode(PORTC,4UL,kPortMuxAlt3);
      break;
    default:
      break;
  }
}

void configure_lpuart_pins(uint32_t instance)
{

}

/* END pin_mux. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
