/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : pin_mux.c
**     Project     : MRB-KW019032NA
**     Processor   : MKW01Z128CHN4
**     Component   : PinSettings
**     Version     : Component 1.2.0, Driver 1.4, CPU db: 3.00.000
**     Repository  : KSDK 1.3.0
**     Compiler    : GNU C Compiler
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
#include "fsl_gpio_hal.h"
#include "pin_mux.h"

void configure_gpio_pins(uint32_t instance)
{
  switch(instance) {    
    case PORTA_IDX:
      break;
    case PORTB_IDX:
      PORT_HAL_SetMuxMode(PORTB, 0u, kPortMuxAsGpio);
      break;
    case PORTC_IDX: 
      break;
	case PORTD_IDX:
      PORT_HAL_SetMuxMode(PORTD, 6u, kPortMuxAsGpio);
	  break;
    case PORTE_IDX:						
      break;	
    default:
      break;
  }
}

void configure_spi_pins(uint32_t instance)
{
  switch(instance) {    
    case SPI0_IDX:       
      /* SPI0 CS */
      PORT_HAL_SetMuxMode(PORTD, 0u, kPortMuxAsGpio);
      GPIO_HAL_SetPinDir(GPIOD, 0u, kGpioDigitalOutput);
      /* SPI0 CLK */
      PORT_HAL_SetMuxMode(PORTC, 5u, kPortMuxAlt2);
      /* SPI0 MOSI */
      PORT_HAL_SetMuxMode(PORTC, 6u, kPortMuxAlt2);
      /* SPI0 MISO */
      PORT_HAL_SetMuxMode(PORTC, 7u, kPortMuxAlt2);
      break;
    case SPI1_IDX:       
      /* SPI1 CS */
      PORT_HAL_SetMuxMode(PORTD, 4u, kPortMuxAlt2);
      /* SPI1 CLK */
      PORT_HAL_SetMuxMode(PORTD, 5u, kPortMuxAlt2);
      /* SPI1 MOSI */
      PORT_HAL_SetMuxMode(PORTE, 1u, kPortMuxAlt2);
      /* SPI1 MISO */
      PORT_HAL_SetMuxMode(PORTE, 0u, kPortMuxAlt2);
      break;      
    default:
      break;
  }
}

void configure_lpsci_pins(uint32_t instance)
{
  switch(instance) {    
    case UART0_IDX:
      /* UART0 RX */
      PORT_HAL_SetMuxMode(PORTA, 1u, kPortMuxAlt2);
      /* UART0 TX */
      PORT_HAL_SetMuxMode(PORTA, 2u, kPortMuxAlt2); 
      break;
    default:
      break;
  }
}

void configure_uart_pins(uint32_t instance)
{

}

void configure_i2c_pins(uint32_t instance)
{
  switch(instance) {
    case I2C0_IDX:                      /* I2C0_IDX */
      PORT_HAL_SetMuxMode(PORTE,18UL,kPortMuxAlt4);
      //PORT_HAL_SetOpenDrainCmd(PORTE,18UL,true);
      PORT_HAL_SetMuxMode(PORTE,19UL,kPortMuxAlt4);
      //PORT_HAL_SetOpenDrainCmd(PORTE,19UL,true);
      break;
    default:
      break;
  }
}

/* Setup SPI pins to communicate with wireless modem */
void configure_spi_pins_for_modem(uint32_t instance)
{
  switch(instance) {
    case SPI0_IDX:                          /* SPI0 */
      /* PORTE_PCR19 */
      PORT_HAL_SetMuxMode(PORTC,6u,kPortMuxAlt2); /* MISO */
      /* PORTE_PCR18 */
      PORT_HAL_SetMuxMode(PORTC,7u,kPortMuxAlt2); /* MOSI */
      /* PORTE_PCR17 */
      PORT_HAL_SetMuxMode(PORTC,5u,kPortMuxAlt2); /* SCK */
      /* PORTE_PCR16 */
      PORT_HAL_SetMuxMode(PORTD,0u,kPortMuxAlt2); /* PCS0 */
      break;
    default:
      break;
  }
}

void configure_xcvr_pins(void)
{
  /* XCVR DIO0 */
  PORT_HAL_SetMuxMode(PORTC,3u,kPortMuxAsGpio);
  GPIO_HAL_SetPinDir(GPIOC,3u,kGpioDigitalInput);

  /* XCVR DIO1 */
  PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAsGpio);
  GPIO_HAL_SetPinDir(GPIOC,4u,kGpioDigitalInput);

  /* XCVR DIO4 */
  PORT_HAL_SetMuxMode(PORTA, 19u, kPortMuxAsGpio);
  GPIO_HAL_SetPinDir(GPIOA, 19u, kGpioDigitalInput);  

  /* XCVR CLKIN */
  PORT_HAL_SetMuxMode(PORTA,18u,kPortMuxAsGpio);
  GPIO_HAL_SetPinDir(GPIOA,18u,kGpioDigitalInput);

  /* XCVR RESET */
  PORT_HAL_SetMuxMode(PORTE,30u,kPortMuxAsGpio);
  GPIO_HAL_SetPinDir(GPIOE,30u,kGpioDigitalOutput);
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
