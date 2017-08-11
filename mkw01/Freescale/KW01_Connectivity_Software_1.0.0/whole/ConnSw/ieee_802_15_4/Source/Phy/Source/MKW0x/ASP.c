/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ASP.c
* This is the source file for the ASP module.
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

#include "EmbeddedTypes.h"
#include "AspInterface.h"

#include "SX123xDrv.h"
/*#include "PhyPib.h"
#include "PhyExtended.h"
#include "Phy.h"
#include "PhyInterface.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciCommunication.h"
#endif
#include "MemManager.h"
#include "FunctionLib.h"
#include "Panic.h"
#include "board.h"
#ifdef gSmacSupported
#include "SMAC_Interface.h"
#endif
*/
#if gAspCapability_d

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
/*#ifdef gSmacSupported
#undef PhyPlmeForceTrxOffRequest
#define PhyPlmeForceTrxOffRequest() MLMEPhySoftReset()
#endif
*/
#define SetupContinuousDrivePin()     do { \
                                         PORT_PCR_REG(APP_DIO2_DATA_DRIVE_PORT,APP_DIO2_DATA_DRIVE_PIN) = \
                                         PORT_PCR_MUX(1); GPIO_PDDR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) |= \
                                                          (uint32_t) (1 << APP_DIO2_DATA_DRIVE_PIN); \
                                      } while(0)
#define SetContinuousDrivePin() (GPIO_PSOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) |= (1 << APP_DIO2_DATA_DRIVE_PIN))
#define ClearContinuousDrivePin() (GPIO_PCOR_REG(APP_DIO2_DATA_DRIVE_GPIO_PORT) |= (1 << APP_DIO2_DATA_DRIVE_PIN))
/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
void Asp_XCVRContReset(void); //moved from SMAC
void Asp_XCVRRestart(void);   //moved from SMAC
void Asp_EnableXCVRInterrupts(void); //moved from SMAC
void Asp_DisableXCVRInterrupts(void);// moved from SMAC
AspStatus_t Asp_EnablePABoost(uint8_t u8PABoostCfg); //moved from SMAC
AspStatus_t Asp_SetClockRate(clkoFrequency_t); // moved from SMAC
AspStatus_t Asp_ListenRequest( listenResolution_t listenResolution, uint8_t listenCoef); //moved from SMAC
AspStatus_t Asp_SetStandByModeRequest(void); //moved from SMAC
AspStatus_t Asp_SetSleepModeRequest(void); //moved from SMAC
AspStatus_t Asp_EnableAfc(bool_t, bool_t);
AspStatus_t Asp_DisableAfc(void);
/************************************************************************************
*************************************************************************************
* Private functions prototype
*************************************************************************************
************************************************************************************/
AspStatus_t Asp_XcvrWriteReq (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData);
AspStatus_t Asp_XcvrReadReq  (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData);
AspStatus_t Asp_SetPowerLevel(uint8_t powerLevel);
uint8_t     Asp_GetPowerLevel(void);

AspStatus_t Asp_SetActivePromState(bool_t state);
AspStatus_t Asp_SetLQIMode(bool_t mode);
uint8_t     Asp_GetRSSILevel(void);

AspStatus_t ASP_TelecSetFreq    (uint8_t channel);
AspStatus_t ASP_TelecSendRawData(uint8_t* dataPtr);
AspStatus_t ASP_TelecTest       (uint8_t mode);

#if gFsciIncluded_c
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId);
static void AspSapMonitor(void *pData, void* param, uint32_t interfaceId);
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint8_t u8Freq_DevMsb;
static uint8_t u8Freq_DevLsb;
static bool_t  gEnabledAfc = FALSE;

#if gFsciIncluded_c
static uint8_t mAspFsciBinding[gPhyInstancesCnt_c];
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

#undef mFuncId_c
#define mFuncId_c 0
void ASP_Init( instanceId_t phyInstance, uint8_t interfaceId )
{
#if gFsciIncluded_c 
    if( phyInstance < gPhyInstancesCnt_c )
    {
        mAspFsciBinding[phyInstance] = interfaceId;
        FSCI_RegisterOpGroup( gFSCI_AppAspOpcodeGroup_c, gFsciMonitorMode_c, fsciAspReqHandler, NULL, gAspInterfaceId);
        FSCI_RegisterOpGroup( gFSCI_AspSapId_c,          gFsciMonitorMode_c, AspSapMonitor,     NULL, gAspInterfaceId);
    }
#endif
#ifdef gSmacSupported
    SetupContinuousDrivePin();
#endif
}

#undef mFuncId_c
#define mFuncId_c 1
AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t instanceId)
{
    AspStatus_t status = gAspSuccess_c;
#if gFsciIncluded_c    
    FSCI_Monitor( gFSCI_AspSapId_c, 
                  pMsg, 
                  NULL,
                  gAspInterfaceId );
#endif
    switch( pMsg->msgType )
    {
    case aspMsgTypeXcvrWriteReq_c:
        status = Asp_XcvrWriteReq( pMsg->msgData.aspXcvrData.mode,
                                   pMsg->msgData.aspXcvrData.addr,
                                   pMsg->msgData.aspXcvrData.len,
                                   pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeXcvrReadReq_c:
        status = Asp_XcvrReadReq( pMsg->msgData.aspXcvrData.mode,
                                  pMsg->msgData.aspXcvrData.addr,
                                  pMsg->msgData.aspXcvrData.len,
                                  pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeSetFADState_c:
        status = Asp_SetFADState(pMsg->msgData.aspFADState);
        break;
    case aspMsgTypeSetFADThreshold_c:
        status = Asp_SetFADThreshold(pMsg->msgData.aspFADThreshold);
        break;
    case aspMsgTypeSetANTXState_c:
        status = Asp_SetANTXState(pMsg->msgData.aspANTXState);
        break;
    case aspMsgTypeGetANTXState_c:
        *((uint8_t*)&status) = Asp_GetANTXState();
        break;
    case aspMsgTypeSetPowerLevel_c:
        status = Asp_SetPowerLevel(pMsg->msgData.aspSetPowerLevelReq.powerLevel);
        break;
    case aspMsgTypeGetPowerLevel_c:
        *((uint8_t*)&status) = Asp_GetPowerLevel(); //remove compiler warning
        break;
    case aspMsgTypeTelecSetFreq_c:
        status = ASP_TelecSetFreq(pMsg->msgData.aspTelecsetFreq.channel);
        break;
    case aspMsgTypeTelecSendRawData_c:
        status = ASP_TelecSendRawData((uint8_t*)&pMsg->msgData.aspTelecSendRawData);
        break;
    case aspMsgTypeTelecTest_c:
        status = ASP_TelecTest(pMsg->msgData.aspTelecTest.mode);
        break;
    case aspMsgTypeSetLQIMode_c:
        status = Asp_SetLQIMode(pMsg->msgData.aspLQIMode);
        break;
    case aspMsgTypeGetRSSILevel_c:
        *((uint8_t*)&status) = Asp_GetRSSILevel(); //remove compiler warning
        break;
    default:
        status = gAspInvalidRequest_c;// OR gAspInvalidParameter_c
        break;
    }
#if gFsciIncluded_c
    FSCI_Monitor( gFSCI_AspSapId_c, 
                  pMsg, 
                  (void*)&status, 
                  gAspInterfaceId );
#endif
    return status;
}

#undef mFuncId_c
#define mFuncId_c 2
//Write len bytes to registers starting at addr, from pData 
AspStatus_t Asp_XcvrWriteReq (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData)
{
    (void)mode;
    uint8_t addrIncrement = 0;
    for(addrIncrement = 0; addrIncrement < len; addrIncrement++)
      XCVRDrv_WriteRegister(addrIncrement + addr, pData[addrIncrement]);
    
    return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 3
//Read len bytes to registers starting at addr, into pData 
AspStatus_t Asp_XcvrReadReq  (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData)
{
    (void)mode;
    uint8_t addrIncrement = 0;
    for(addrIncrement = 0; addrIncrement < len; addrIncrement++)
      pData[addrIncrement] = XCVRDrv_ReadRegister(addr + addrIncrement);
    
    return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 4
//Sets power level for MKW2x. Can be used for XCVR but different power levels required
AspStatus_t Asp_SetPowerLevel( uint8_t powerLevel )
{
    if(powerLevel > gAspPowerLevel_16dBm)
        return gAspInvalidParameter_c;
    {
        uint8_t res;
     
        res = PhyPib_SetTransmitPower(powerLevel);
        if( res == gPhySuccess_c )
        {
            return gAspSuccess_c;
        }
        else
        {
            return gAspDenied_c;
        }
    }
}

#undef mFuncId_c
#define mFuncId_c 5
//get power level for MKW0x
uint8_t Asp_GetPowerLevel()
{
    return (XCVRDrv_ReadRegister(XCVR_Reg_PaLevel) & 0x1F);
}
#undef mFuncId_c
#define mFuncId_c 6
//not used for MKW0x
AspStatus_t Asp_SetActivePromState(bool_t state)
{
    //PhySetActivePromiscuous(state);
    return gAspDenied_c;
}

#undef mFuncId_c
#define mFuncId_c 7
//not used for MKW0x
AspStatus_t Asp_SetFADState(bool_t state)
{
//    if( gPhySuccess_c != PhyPlmeSetFADStateRequest(state) )
//    {
//        return gAspDenied_c;
//    }
    return gAspDenied_c;
}

#undef mFuncId_c
#define mFuncId_c 8
//not used for MKW0x
AspStatus_t Asp_SetFADThreshold(uint8_t thresholdFAD)
{
//    if( gPhySuccess_c != PhyPlmeSetFADThresholdRequest(thresholdFAD) )
//    {
//        return gAspDenied_c;
//    }
    return gAspDenied_c;
}

#undef mFuncId_c
#define mFuncId_c 9
//not used for MKW0x
AspStatus_t Asp_SetANTXState(bool_t state)
{
//    if( gPhySuccess_c != PhyPlmeSetANTXStateRequest(state) )
//    {
//        return gAspDenied_c;
//    }
    return gAspDenied_c;
}

#undef mFuncId_c
#define mFuncId_c 10
//not used for MKW0x
uint8_t Asp_GetANTXState(void)
{
  return gAspDenied_c;//PhyPlmeGetANTXStateRequest();
}

#undef mFuncId_c
#define mFuncId_c 11
//not used for MKW0x
AspStatus_t Asp_SetLQIMode(bool_t mode)
{
   return gAspDenied_c;
}

#undef mFuncId_c
#define mFuncId_c 12
//not used for MKW0x
uint8_t Asp_GetRSSILevel(void)
{
  return Phy_GetRssi();
}






/*+-----------------------------------------------------------------------+
|                            TELEC Functions                            |
+-----------------------------------------------------------------------+*/
#undef mFuncId_c
#define mFuncId_c 13
//Sets Carrier frequency in respect to frequency band, mode and channel
AspStatus_t ASP_TelecSetFreq(uint8_t channel)
{
    PhyPlmeForceTrxOffRequest();
    if( gPhySuccess_c != PhyPib_SetCurrentChannel(channel) )
    {
        return gAspInvalidParameter_c;
    }
    
    return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 14
//Synchronous function that sends a buffer OTA
AspStatus_t ASP_TelecSendRawData(uint8_t* dataPtr) 
{
  uint8_t irqFlags2;
  phyStatus_t status;
  
  pdDataReq_t dataReq;
  phyTxParams_t txParam;
  
  PhyPlmeForceTrxOffRequest(); //force XCVR off
  //prepare data to send. Phy Transmission Param Config
  dataReq.ackRequired = gPhyNoAckRqd_c;
  dataReq.CCABeforeTx = gPhyNoCCABeforeTx_c;
  dataReq.macDataIndex = 0;
  dataReq.pPsdu = dataPtr + 1;
  //buffer[0] contains buffer length
  dataReq.psduLength = dataPtr[0];
  dataReq.slottedTx  = gPhyUnslottedMode_c;
  dataReq.startTime  = gPhySeqStartAsap_c;
  dataReq.txDuration = 0xFFFFFFFF;
  
  txParam.numOfCca    = 0;
  txParam.ackRequired = gPhyNoAckRqd_c;
  //program sequence start time and duration
  Phy_SetSequenceTiming((&dataReq)->startTime, 
                         (&dataReq)->txDuration, 
                         gTX_c,
                         0 );
  //make data request
  status = PhyPdDataRequest(&dataReq, NULL, &txParam);
  if(status != gPhySuccess_c)
    return gAspDenied_c;
  //disable DIO0 so that Packet Sent event won't fire
  //and Phy State Machine will be kept Idle.
  XCVRDrv_IRQ_DIO0_Disable();
  
  do
  {
    irqFlags2 = XCVRDrv_ReadRegister(0x28);
  }
  //wait until packet sent flag is set or FifoNotEmpty flag is cleared.
  while(!(irqFlags2 & IrqFlags2_PacketSent) && (irqFlags2 & IrqFlags2_FifoNotEmpty)); 
  
  //restore everything to idle
  PhyPlmeForceTrxOffRequest();
  //if fifo became empty but no packet sent flag is set then
  //return error code.
  if(!(irqFlags2 & IrqFlags2_PacketSent))
  {
   return gAspDenied_c;
  }
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 15
//implements test modes performed on MKW0x
AspStatus_t ASP_TelecTest(uint8_t mode)
{
  switch(mode)
  {
  case gTestForceIdle_c:
    
    PhyPlmeForceTrxOffRequest();
    //in case there was previously a Tx Test Mode, the Phy State machine is bypassed,
    //so XCVR might need to be set to stand by manually;
    (void)Phy_SetOperationMode(OpMode_StandBy);
    (void)PhySetState(gIdle_c);
    
    if(u8Freq_DevMsb || u8Freq_DevLsb)                          //if previously been to tx unmodulated
    {
      XCVRDrv_WriteRegister(XCVR_Reg_FdevMsb, u8Freq_DevMsb);
      XCVRDrv_WriteRegister(XCVR_Reg_FdevLsb, u8Freq_DevLsb);
      u8Freq_DevLsb = 0;
      u8Freq_DevMsb = 0;
    }
    Phy_SetDataMode(DataModul_DataMode_Packet);
    SetContinuousDrivePin();
    break;
    
  case gTestContinuousTxModOne_c:
    SetContinuousDrivePin();
    /*Set Data Mode*/
    Phy_SetDataMode(DataModul_DataMode_Continous);
    /* Put transceiver in TX mode */
    (void)Phy_SetOperationMode(OpMode_Transmitter);// Set XCVR in TX;
    break;

  case gTestContinuousTxModZero_c:
    ClearContinuousDrivePin();
    /*Set Data Mode*/
    Phy_SetDataMode(DataModul_DataMode_Continous);
    /* Put transceiver in TX mode */
    (void)Phy_SetOperationMode(OpMode_Transmitter);// Set XCVR in TX;
    break;

  case gTestContinuousTxContPN9_c:
    PhySetState(gPN9_c);
    XCVRDrv_ConfigureDIOPins((DIO0_CM_TxPllLock | DIO1_CM_TxDclk
                             | DIO2_CM_TxData | DIO3_CM_TxTxReady), (DIO4_CM_TxTxReady
                             | DIO5_CM_TxClkOut)); 
    Asp_EnableXCVRInterrupts();
    /*Set Data Mode*/
    Phy_SetDataMode(DataModul_DataMode_Continous);
    /* Put transceiver in TX mode */
    (void)Phy_SetOperationMode(OpMode_Transmitter);
    break;
    
  case gTestContinuousTxNoMod_c:
    u8Freq_DevMsb = XCVRDrv_ReadRegister(XCVR_Reg_FdevMsb);
    u8Freq_DevLsb = XCVRDrv_ReadRegister(XCVR_Reg_FdevLsb);
    
    XCVRDrv_WriteRegister(XCVR_Reg_FdevMsb, 0x00);
    XCVRDrv_WriteRegister(XCVR_Reg_FdevLsb, 0x00);
    /*Set Data Mode*/
    Phy_SetDataMode(DataModul_DataMode_Continous);                         
    /* Put transceiver in TX mode */
    (void)Phy_SetOperationMode(OpMode_Transmitter);
    break;
  case gTestContinuousRx_c:
    Phy_SetDataMode(DataModul_DataMode_Continous); //continuous mode with bit synchronizer
   //DIO1 Dclk and DIO2 Data
   XCVRDrv_ConfigureDIOPins((DIO0_CM_RxSyncAddress | DIO1_CM_RxDclk
                             | DIO2_CM_RxData | DIO3_CM_RxRssi), (DIO4_CM_RxTimeout
                             | DIO5_CM_RxClkOut));
    //disable all Radio Interrupts
    Asp_DisableXCVRInterrupts();
    //start RX
    Phy_SetOperationMode(OpMode_Receiver);
    break;
  default:
    return gAspInvalidParameter_c;
  }
  return gAspSuccess_c;
}

/******************************************************************************/
#if gFsciIncluded_c
#undef mFuncId_c
#define mFuncId_c 16
static uint32_t getPhyInstance( uint32_t interfaceId )
{
    uint32_t i;
    
    for( i=0; i<gPhyInstancesCnt_c; i++ )
        if( mAspFsciBinding[i] == interfaceId )
            return i;
            
    return 0;
}

#undef mFuncId_c
#define mFuncId_c 17
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId)
{
    clientPacket_t *pClientPacket = ((clientPacket_t*)pData);
    uint8_t *pMsg = pClientPacket->structured.payload;
    
    pMsg -= sizeof(AppAspMsgType_t);
    ((AppToAspMessage_t*)pMsg)->msgType = (AppAspMsgType_t)pClientPacket->structured.header.opCode;
    
    APP_ASP_SapHandler( (AppToAspMessage_t*)pMsg, getPhyInstance( interfaceId ) );
    MEM_BufferFree(pData);    
}

#undef mFuncId_c
#define mFuncId_c 18
static void AspSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
    clientPacket_t *pFsciPacket = MEM_BufferAlloc( sizeof(clientPacket_t) );
    AppToAspMessage_t *pReq = (AppToAspMessage_t*)pData;
    uint8_t *p;
    
    if( NULL == pFsciPacket )
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }
    
    p = pFsciPacket->structured.payload;
    
    if( NULL == param )
    {
        pFsciPacket->structured.header.opGroup = gFSCI_AppAspOpcodeGroup_c;
        pFsciPacket->structured.header.opCode = pReq->msgType;
        
        switch( pReq->msgType )
        {
        case aspMsgTypeXcvrWriteReq_c:
        case aspMsgTypeXcvrReadReq_c:
            *p++ = pReq->msgData.aspXcvrData.mode;
            FLib_Memcpy(p, pReq->msgData.aspXcvrData.addr, sizeof(uint16_t));
            p += sizeof(uint16_t);
            *p++ = pReq->msgData.aspXcvrData.len;
            if( pReq->msgType == aspMsgTypeXcvrWriteReq_c )
            {
                FLib_MemCpy( p, pReq->msgData.aspXcvrData.data, 
                             pReq->msgData.aspXcvrData.len );
                p += pReq->msgData.aspXcvrData.len;
            }
            break;
       /* case aspMsgTypeSetFADState_c:
            FLib_MemCpy( p, &pReq->msgData.aspFADState, sizeof(pReq->msgData.aspFADState) );
            p += sizeof(pReq->msgData.aspFADState);
            break;
       case aspMsgTypeSetFADThreshold_c:
            FLib_MemCpy( p, &pReq->msgData.aspFADThreshold, sizeof(pReq->msgData.aspFADThreshold) );
            p += sizeof(pReq->msgData.aspFADThreshold);
            break;*/
        case aspMsgTypeSetANTXState_c:
            FLib_MemCpy( p, &pReq->msgData.aspANTXState, sizeof(pReq->msgData.aspANTXState) );
            p += sizeof(pReq->msgData.aspANTXState);
            break;
        case aspMsgTypeGetANTXState_c:
            /* Nothing to do here */
            break;
            
        case aspMsgTypeSetPowerLevel_c:
            FLib_MemCpy( p, &pReq->msgData.aspSetPowerLevelReq, sizeof(pReq->msgData.aspSetPowerLevelReq) );
            p += sizeof(pReq->msgData.aspSetPowerLevelReq);
            break;
        case aspMsgTypeGetPowerLevel_c:
            /* Nothing to do here */
            break;
        case aspMsgTypeTelecSetFreq_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecsetFreq, sizeof(pReq->msgData.aspTelecsetFreq) );
            p += sizeof(pReq->msgData.aspTelecsetFreq);
            break;
        case aspMsgTypeTelecSendRawData_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecSendRawData, sizeof(pReq->msgData.aspTelecSendRawData) );
            p += sizeof(pReq->msgData.aspTelecSendRawData);
            break;
        case aspMsgTypeTelecTest_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecTest, sizeof(pReq->msgData.aspTelecTest) );
            p += sizeof(pReq->msgData.aspTelecTest);
            break;
        case aspMsgTypeSetLQIMode_c:
            FLib_MemCpy(p, &pReq->msgData.aspLQIMode, sizeof(pReq->msgData.aspLQIMode) );
            p += sizeof(pReq->msgData.aspLQIMode);
            break;
        case aspMsgTypeGetRSSILevel_c:
            /* Nothing to do here */
            break;
        }
    }
    else
    {
        pFsciPacket->structured.header.opGroup = gFSCI_AspAppOpcodeGroup_c;
        pFsciPacket->structured.header.opCode = pReq->msgType;
        
        *p++ = *((uint8_t*)param);/* copy status */
        
        switch( pReq->msgType )
        {
        case aspMsgTypeXcvrReadReq_c:
            *p++ = pReq->msgData.aspXcvrData.len; /* copy length */
            FLib_MemCpy( p, pReq->msgData.aspXcvrData.data, pReq->msgData.aspXcvrData.len );
            p += pReq->msgData.aspXcvrData.len;
            break;
        }
        
    }
    
    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
}

#endif /* gFsciIncluded_c */

/**************************************************************************************/
/********************Ported from SMAC. XCVR Specific Functions*************************/
/**************************************************************************************/
#undef mFuncId_c
#define mFuncId_c 19
void Asp_XCVRContReset(void) //moved from SMAC
{
  /* asset transceiver reset line */
  gXcvrAssertReset;
}

#undef mFuncId_c
#define mFuncId_c 20
void Asp_XCVRRestart(void)   //moved from SMAC
{
  /* Deassert transceiver reset line */
  gXcvrDeassertReset;
}

#undef mFuncId_c
#define mFuncId_c 21
void Asp_EnableXCVRInterrupts(void) //moved from SMAC
{
  //XCVRxDrv_IRQ_PortConfig();                                                   //At this point only DIO0, DIO1, DIO4 are enabled (No edge select nor Pin Interrupt enabled)Can Not cause interrupt yet
  XCVRDrv_IRQ_DIO1_Enable(9);                                                    //DIO0 and DIO1, Rising Edge Enabled
  XCVRDrv_IRQ_DIO0_Enable(9);
  XCVRDrv_IRQ_DIO4_Enable(9);
}

#undef mFuncId_c
#define mFuncId_c 22
void Asp_DisableXCVRInterrupts(void)// moved from SMAC
{
  XCVRDrv_IRQ_DIO0_Disable();                                                    //DIO0 and DIO1, Rising Edge Enabled
  XCVRDrv_IRQ_DIO1_Disable();
  XCVRDrv_IRQ_DIO4_Disable();
}

#undef mFuncId_c
#define mFuncId_c 23
AspStatus_t Asp_EnablePABoost(uint8_t u8PABoostCfg) //moved from SMAC
{
  PhyPlmeForceTrxOffRequest();
  
  if ( gDisablePA_Boost_c == u8PABoostCfg ||
      gEnablePA2_Boost_c == u8PABoostCfg || 
        gEnablePA1_Boost_c == u8PABoostCfg || 
          gEnablePABoth_Boost_c == u8PABoostCfg )
  {
    Phy_PABoostCfg (u8PABoostCfg);
    return gAspSuccess_c;
  }
  else
    return gAspInvalidParameter_c;
}

#undef mFuncId_c
#define mFuncId_c 24
AspStatus_t Asp_DisablePABoost(void)            //moved from SMAC
{
  PhyPlmeForceTrxOffRequest();
  Phy_PABoostCfg (gDisablePA_Boost_c);
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 25
AspStatus_t Asp_SetClockRate(clkoFrequency_t clkFreq)       //moved from SMAC
{
  if(clkFreq >= gAspClkoOutOfRange_c)
    return gAspInvalidParameter_c;
  
  PhyPlmeForceTrxOffRequest();
  XCVRDrv_ConfigureCLKO(clkFreq);
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 26
AspStatus_t Asp_SetLnaGainAdjust(lnaGainValues_t gainValue)     //moved from SMAC
{
  if(gainValue > gAspLnaGain_7_c)
    return gAspInvalidParameter_c;
  
  PhyPlmeForceTrxOffRequest();
  Phy_SetLnaGain((uint8_t)gainValue);
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 27
AspStatus_t Asp_ListenRequest( listenResolution_t listenResolution, uint8_t listenCoef) //moved from SMAC
{	
  if(gIdle_c != PhyPpGetState())
  {
    return gAspDenied_c;
  }
  Phy_SetListenResRx((uint8_t)listenResolution);
  Phy_SetListenCoefRx(listenCoef);
  
  Phy_SetOperationMode(OpMode_StandBy);
  Phy_SetOperationMode(OpMode_Listen_On); 
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 28
AspStatus_t Asp_SetStandByModeRequest(void)                    //moved from SMAC
{
  if(PhyPpGetState() != gIdle_c)
    return gAspDenied_c;
  
  Phy_SetOperationMode(OpMode_StandBy);
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 29
AspStatus_t Asp_SetSleepModeRequest(void)                      //moved from SMAC
{
  if(PhyPpGetState() != gIdle_c)
    return gAspDenied_c;
  
  Phy_SetOperationMode(OpMode_Sleep);
  
  return gAspSuccess_c;
}

#undef mFuncId_c
#define mFuncId_c 30
//enables AFC feature and/or control low beta and autoclear enablement
//this means that if AFC is already enabled, it just controls low beta and autoclear enablement
AspStatus_t Asp_EnableAfc(bool_t lowBetaOn, bool_t autoClearOn)
{ 
  if(PhyPpGetState() != gIdle_c)
    return gAspDenied_c;
  
  if(!gEnabledAfc)
  {
    PhyPib_EnableAFC(TRUE);
    gEnabledAfc = TRUE;
  }
  Phy_EnableAfcLowBeta(lowBetaOn);
  Phy_EnableAfcAutoclear(autoClearOn);
  
  return gAspSuccess_c;
  
}

#undef mFuncId_c
#define mFuncId_c 31
//disable Afc Feature
AspStatus_t Asp_DisableAfc()
{
  if(PhyPpGetState() != gIdle_c)
    return gAspDenied_c;
  
  PhyPib_EnableAFC(FALSE);
  gEnabledAfc = FALSE;
  
  return gAspSuccess_c;
}
#endif /* gAspCapability_d */
