/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SX123xDrv.c
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
#include "SX123xDrv.h" 
#include "EmbeddedTypes.h"
#include "Phy.h"
#include "SPI.h" 
//#include "fsl_os_abstraction.h"
#include "portmacro.h"//改过的
/*****************************************************************************
 *                      PUBLIC MEMORY DECLARATIONS                           *
 *****************************************************************************/   
uint8_t const XCVRDefaultRegisterValues[] = 
{
    /* Radio operation mode initialization @0x01*/
    XCVR_Reg_OpMode, (uint8_t) ( OpMode_Sequencer_On | OpMode_Listen_Off | OpMode_StandBy ) ,

    /* Radio Data mode and modulation initialization @0x02*/
    XCVR_Reg_DataModul, (uint8_t) ( DataModul_DataMode_Packet | DataModul_Modulation_Fsk | DataModul_ModulationShaping_NoShaping ) ,

    /* Radio bit rate initialization @0x03-0x04*/
    XCVR_Reg_BitrateMsb, (uint8_t) ( BitrateMsb_100000 ) ,
    XCVR_Reg_BitrateLsb, (uint8_t) ( BitrateLsb_100000 ) ,

    /* Radio frequency deviation initialization @0x05-0x06*/
    XCVR_Reg_FdevMsb, (uint8_t) ( FdevMsb_50000 ) ,
    XCVR_Reg_FdevLsb, (uint8_t) ( FdevLsb_50000 ) ,

    /* Radio RF frequency initialization @0x07-0x09*/
    /*@CMA, Default Frequencies*/
    XCVR_Reg_FrfMsb, (uint8_t) ( FrfMsb ) ,
    XCVR_Reg_FrfMid, (uint8_t) ( FrfMid ) ,
    XCVR_Reg_FrfLsb, (uint8_t) ( FrfLsb ) ,

    /* Radio RegAfcCtrl initialization @0x0B*/
    XCVR_Reg_AfcCtrl, (uint8_t) AfcCtrl_AfcLowBeta_Off ,

    /* Radio output power initialization @0x11*/
    XCVR_Reg_PaLevel, (uint8_t) ( PaLevel_Pa0_On | PaLevel_Pa1_Off | PaLevel_Pa2_Off | 0x1F ) ,

    /* Radio Rise/Fall time of ramp up/down in FSK initialization @0x12*/
    XCVR_Reg_PaRamp, (uint8_t) (PaRamp_40) ,

    /* Radio overload current protection for PA initialization 0x13*/
    XCVR_Reg_Ocp, (uint8_t) ( Ocp_Ocp_On | 0x0C) ,

    /* Radio LNA gain and input impedance initialization @0x18*/
    XCVR_Reg_Lna, (uint8_t) ( Lna_LnaZin_200 | Lna_LnaGain_Agc ) ,

    /* Radio channel filter bandwidth initialization @0x19*/
    XCVR_Reg_RxBw, (uint8_t) ( DccFreq_5 | RxBw_166700 ) ,

    /* Radio channel filter bandwidth for AFC operation initialization @0x1A*/
    XCVR_Reg_AfcBw, (uint8_t) ( DccFreq_5 | RxBw_166700 ) ,

    /* Radio automatic frequency control initialization @0x1E*/
    XCVR_Reg_AfcFei, (uint8_t) ( AfcFei_AfcAutoClear_On | AfcFei_AfcAuto_Off ) ,

    /* Radio Rssi threshold initialization @0x29*/
    // RSSIthres = [-174 + NF +10*log(2*RxBw) + DemodSNR] dBm
    // NF = 7dB
    // DemodSnr = 8dB
    // RxBw depends on frequency bands and profiles
    XCVR_Reg_RssiThresh, (uint8_t) (0xDC) , // -101 dBm for 333.3 Khz singleside channel filter bandwith

    /* Radio RegTimeoutRxStart initialization @0x2A*/
    /* Radio RegTimeoutRssiThresh initialization @0x2B*/
    XCVR_Reg_RxTimeout1, (uint8_t) 0x00 , //disable timeout rx start
    XCVR_Reg_RxTimeout2, (uint8_t) 0x00 , //disable timeout rx start

    /* XCVR preamble size initialization @0x2C-0x2D*/
    XCVR_Reg_PreambleMsb, (uint8_t) 0x00,
    XCVR_Reg_PreambleLsb, (uint8_t) 0x0E,

    /* Radio sync word control and value initialization @0x2E-0x30*/
    XCVR_Reg_SyncConfig, (uint8_t) (SyncConfig_Sync_On | SyncConfig_FifioFill_ifSyncAddres | SyncConfig_SyncSize_4) ,
    XCVR_Reg_SyncValue1, (uint8_t) (0x55) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue2, (uint8_t) (0x55) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue3, (uint8_t) (0x90) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue4, (uint8_t) (0x4E) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue5, (uint8_t) (0x90) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue6, (uint8_t) (0x4E) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue7, (uint8_t) (0x90) , //SFD value for uncoded with phySUNMRFSKSFD = 0
    XCVR_Reg_SyncValue8, (uint8_t) (0x4E) , //SFD value for uncoded with phySUNMRFSKSFD = 0  

    /* FIFO threshold */
    XCVR_Reg_FifoThresh, (uint8_t) (0x00) , // Tx start - the number of bytes in the FIFO exceeds FifoThreshold = 0

    /* Radio packet mode config */
    XCVR_Reg_PacketConfig1, (uint8_t) 0x08,
    XCVR_Reg_PacketConfig2, (uint8_t) 0x00,
    
    /* Radio Listen mode config */
    XCVR_Reg_Listen1,       (uint8_t) 0x50,
    XCVR_Reg_Listen2,       (uint8_t) 0x01,
    XCVR_Reg_Listen3,       (uint8_t) 0x01,
    
    /* Radio payload length initialization */
    XCVR_Reg_PayloadLength, (uint8_t) 0x00,  //max length in rx

    XCVR_Reg_TestPLL_BW,    (uint8_t) 0x08,
    XCVR_Reg_TestLna,       (uint8_t) 0x2D,
    XCVR_Reg_TestDagc,      (uint8_t) 0x30
};

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
 * Name: XCVRDrv_Reset
 * Description: Make a pulse on Radio hardware reset PIN
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t XCVRDrv_Reset(void)
{
    //@AC RESET sequence from SX1233 datasheet:
    // RESET high Z
    // RESET = 1, then wait 100us
    // RESET = 0 (high Z), then wait 5ms before using the Radio. 
    volatile uint16_t delay = 40;

    gXcvrAssertReset;
    while(--delay);
    delay = 1700;
    gXcvrDeassertReset;   

    while( !gXcvrClkoutReady )
    {
        delay--;
        if( !delay )
        {
            return FALSE;
        }
    }
    
    return TRUE;  
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadRegister
 * Description: Read one 8bit data-location from Radio register "addr"
 *              May be called from any context.
 * Parameters: register address
 * Return: register value
 *---------------------------------------------------------------------------*/
uint8_t XCVRDrv_ReadRegister(uint8_t addr) 
{
    uint8_t txData = addr;
    uint8_t rxData;

    portENTER_CRITICAL();//所有的这个都是从OSA_EnterCritical改变而来  改过

    gXcvrAssertCS_d();
    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, &rxData, sizeof(rxData));
    gXcvrDeassertCS_d();

    portEXIT_CRITICAL();

    return rxData;
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadFifo
 * Description: Read one 8bit data-location from RegFifo register
 *              May be called from any context.
 * Parameters: -
 * Return: register value
 *---------------------------------------------------------------------------*/ 
uint8_t XCVRDrv_ReadFifo(void) 
{
    return XCVRDrv_ReadRegister(XCVR_Reg_Fifo);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ReadBytesFromFifoLSB
 * Description: Reads a buffer from Radio at Fifo address LSB first
 *              May be called from any context.
 * Parameters: value
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ReadBytesFromFifoLSB(uint8_t* buffer, uint8_t size)
{  
    uint8_t txData = XCVR_Reg_Fifo;
    
    if( NULL == buffer )
        return;
    
    portENTER_CRITICAL();
    
    /* Serialize LSB first */
    spi_master_configure_serialization_lsb(gXcvrSpiInstance_c);

    gXcvrAssertCS_d();
    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, buffer, size);
    gXcvrDeassertCS_d();    

    /* Restore serialization to MSB */
    spi_master_configure_serialization_msb(gXcvrSpiInstance_c);
    
    portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteRegister
 * Description: Write one 8bit data into Radio at address "addr"
 *              May be called from any context.
 * Parameters: register address, register value
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteRegister(uint8_t addr, uint8_t val) 
{		
    uint16_t txData = (val << 8) | (addr | 0x80);

    portENTER_CRITICAL();

    gXcvrAssertCS_d();
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t *)&txData, NULL, sizeof(txData));
    gXcvrDeassertCS_d();  
        
  portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteBytesToFifoLSB
 * Description: Writes a buffer into Radio at Fifo address LSB first
 *              May be called from any context.
 * Parameters: value
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteBytesToFifoLSB(uint8_t* buffer, uint8_t size)
{  
    uint8_t txData = (XCVR_Reg_Fifo | 0x01);
    
    if( NULL == buffer )
        return;
    
    portENTER_CRITICAL();
    
    /* Serialize LSB first */
    spi_master_configure_serialization_lsb(gXcvrSpiInstance_c);

    gXcvrAssertCS_d();
    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, buffer, NULL, size);
    gXcvrDeassertCS_d();    

    /* Restore serialization to MSB */
    spi_master_configure_serialization_msb(gXcvrSpiInstance_c);
    
  portEXIT_CRITICAL();
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteFifo
 * Description: Write one 8bit data into Radio at Fifo address
 *              May be called from any context.
 * Parameters: register address, register value
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteFifo(uint8_t val) 
{          
    XCVRDrv_WriteRegister(XCVR_Reg_Fifo | 0x80, val);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_WriteFifoLSB
 * Description: Write one 8bit data into Radio at Fifo address LSB first
 *              May be called from any context.
 * Parameters: value
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_WriteFifoLSB(uint8_t val) 
{    
    uint16_t txData = (val << 8) | (XCVR_Reg_Fifo | 0x01);
  
    portENTER_CRITICAL();
    
    /* Serialize LSB first */
    spi_master_configure_serialization_lsb(gXcvrSpiInstance_c);

    gXcvrAssertCS_d();
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t *)&txData, NULL, sizeof(txData));
    gXcvrDeassertCS_d();
    
    /* Restore serialization to MSB */
    spi_master_configure_serialization_msb(gXcvrSpiInstance_c);
        
  portEXIT_CRITICAL();    
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RFdefaultInit
 * Description: Initialize Radio transceiver with default registers value
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RFdefaultInit(void)
{
    for(uint32_t idx = 0; idx < (sizeof(XCVRDefaultRegisterValues)/sizeof(XCVRDefaultRegisterValues[0])); idx += 2)
    {
        XCVRDrv_WriteRegister(XCVRDefaultRegisterValues[idx], XCVRDefaultRegisterValues[idx+1]);
    }
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ConfigureDIOPins
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ConfigureDIOPins(uint8_t regDio1, uint8_t regDio2)
{
    uint8_t reg2;
    XCVRDrv_WriteRegister(XCVR_Reg_DioMapping1, regDio1);
    reg2 = XCVRDrv_ReadRegister(XCVR_Reg_DioMapping2);
    reg2 = (uint8_t) (reg2 & 0x07);
    reg2 = (uint8_t) (reg2 | (regDio2 & 0xF0) );
    XCVRDrv_WriteRegister(XCVR_Reg_DioMapping2, reg2);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_ConfigureCLKO
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_ConfigureCLKO(uint8_t fxOscDiv)
{
    uint8_t regDio2;
    regDio2 = XCVRDrv_ReadRegister(XCVR_Reg_DioMapping2);
    regDio2 = (uint8_t)(regDio2 & 0xF8);
    XCVRDrv_WriteRegister(XCVR_Reg_DioMapping2, (uint8_t) (regDio2 | fxOscDiv));
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_SetOperatingMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_SetOperatingMode(uint8_t mode)
{
    uint8_t regOpMode;
    regOpMode = XCVRDrv_ReadRegister(XCVR_Reg_OpMode);
    regOpMode = (regOpMode & 0xE3) | mode;
    XCVRDrv_WriteRegister(XCVR_Reg_OpMode, regOpMode);
    while ((XCVRDrv_ReadRegister(XCVR_Reg_IrqFlags1) & IrqFlags1_ModeReady) == 0x00);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RadioSleepReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RadioSleepReq(void)
{
    XCVRDrv_SetOperatingMode(OpMode_Sleep);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_RadioWakeUpReq
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void XCVRDrv_RadioWakeUpReq(void)
{
    XCVRDrv_SetOperatingMode(OpMode_StandBy);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO0_Enable(uint8_t interruptCfg)             
{    
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio0)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio0), 
                           (port_interrupt_config_t)interruptCfg); 
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO1_Enable(uint8_t interruptCfg)             
{
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio1)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio1), 
                           (port_interrupt_config_t)interruptCfg); 
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Enable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO4_Enable(uint8_t interruptCfg)
{
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio4)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio4), 
                           (port_interrupt_config_t)interruptCfg);     
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO0_Disable(void)
{
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio0)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio0), 
                           kPortIntDisabled);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO1_Disable(void)
{
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio1)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio1), 
                           kPortIntDisabled);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Disable
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO4_Disable(void)
{
    PORT_HAL_SetPinIntMode(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio4)], 
                           GPIO_EXTRACT_PIN(kGpioXcvrDio4), 
                           kPortIntDisabled); 
}

/*---------------------------------------------------------------------------
 * Name: XCVR_IRQ_DIO0_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline bool_t XCVRDrv_IRQ_DIO0_Detected(void)
{
    return PORT_HAL_IsPinIntPending(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio0)], 
                                    GPIO_EXTRACT_PIN(kGpioXcvrDio0));
}

/*---------------------------------------------------------------------------
 * Name: XCVR_IRQ_DIO1_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline bool_t XCVRDrv_IRQ_DIO1_Detected(void)
{
    return PORT_HAL_IsPinIntPending(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio1)], 
                                    GPIO_EXTRACT_PIN(kGpioXcvrDio1));
}

/*---------------------------------------------------------------------------
 * Name: XCVR_IRQ_DIO4_Detected
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline bool_t XCVRDrv_IRQ_DIO4_Detected(void)
{
    return PORT_HAL_IsPinIntPending(g_portBase[GPIO_EXTRACT_PORT(kGpioXcvrDio4)], 
                                    GPIO_EXTRACT_PIN(kGpioXcvrDio4));
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO0_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO0_Clear(void)
{
    GPIO_DRV_ClearPinIntFlag(kGpioXcvrDio0);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO1_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO1_Clear(void)
{
    GPIO_DRV_ClearPinIntFlag(kGpioXcvrDio1);
}

/*---------------------------------------------------------------------------
 * Name: XCVRDrv_IRQ_DIO4_Clear
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVRDrv_IRQ_DIO4_Clear(void)
{
    GPIO_DRV_ClearPinIntFlag(kGpioXcvrDio4);
}

/*---------------------------------------------------------------------------
 * Name: XCVR_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
inline void XCVR_Init(void)
{
    configure_xcvr_pins();

    /* Keep Radio in reset during SPI init */
    gXcvrAssertReset; 

    spi_master_init(gXcvrSpiInstance_c);   
    gXcvrDeassertCS_d();

    XCVRDrv_Reset();

    /* CLKOUT set at FXOSC - 30/32 MHz */
    XCVRDrv_ConfigureCLKO(ClkOutFxOsc_Div1);       
}
