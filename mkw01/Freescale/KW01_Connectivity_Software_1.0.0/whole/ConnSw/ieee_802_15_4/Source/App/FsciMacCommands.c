/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file FsciMacCommands.c
* This is a source file which implements the FSCI commands received from the host.
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
#include "FunctionLib.h"
#include "MemManager.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciMacCommands.h"
#include "FsciCommunication.h"
#endif

#include "MacInterface.h"
#if !gFsciHostMacSupport_c
#include "PhyInterface.h"
#endif

#include "fsl_os_abstraction.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/


/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
#if !gFsciHostMacSupport_c 
extern uint16_t mlmeGetSizeOfPIB(pibId_t  pib);
#endif

void fsciMlmeAsyncReqHandler(void *pData, void* param, uint32_t interfaceId);

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
uint8_t fsciToMacBinding[gMacInstancesCnt_c];
#endif

#if gFsciIncluded_c && gFsciHostMacSupport_c
FsciHostMacInterface_t fsciHostMacInterfaces[gMacInstancesCnt_c];

extern clientPacket_t *pFsciHostSyncRsp;
extern uint8_t         gFsciHostWaitingSyncRsp;
extern opGroup_t       gFsciHostWaitingOpGroup;
extern opCode_t        gFsciHostWaitingOpCode;

#if gFsciHostSyncUseEvent_c
extern event_t         gFsciHostSyncRspEvent;
#endif
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief   This function registers the MAC SAP monitors and commands handlers
*
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciRegisterMac( instanceId_t macInterface, uint32_t fsciInterfaceId )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    
    if( macInterface >= gMacInstancesCnt_c )
        return;

    /* Bind MAC instance to interfaceId */
    fsciToMacBinding[macInterface] = fsciInterfaceId;

    /* Register Handler for requests coming from the serial interface */
    FSCI_RegisterOpGroup( gFSCI_NwkMlmeOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMlmeReqHandler, 
                          NULL, 
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_NwkMcpsOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMcpsReqHandler, 
                          NULL, 
                          fsciInterfaceId );

    /* Register SAP Monitor Handler */
    FSCI_RegisterOpGroup( gFSCI_McpsSapId_c,
                          gFsciMonitorMode_c,
                          McpsSapMonitor,
                          NULL,
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_MlmeSapId_c,
                          gFsciMonitorMode_c,
                          MlmeSapMonitor,
                          NULL,
                          fsciInterfaceId );
#endif /* gFsciIncluded_c && gFSCI_IncludeMacCommands_c */
}

/*! *********************************************************************************
* \brief   This function registers the MAC to Host SAP monitors and commands handlers
*
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciRegisterMacToHost( instanceId_t macInterface, uint32_t fsciInterfaceId )
{
#if gFsciIncluded_c && gFsciHostMacSupport_c
    
    if( macInterface >= gMacInstancesCnt_c )
        return;

    /* Bind MAC instance to interfaceId */
    fsciHostMacInterfaces[macInterface].fsciInterfaceId = fsciInterfaceId;

    /* Register Handler for confirms and indications coming from the serial interface */
    FSCI_RegisterOpGroup( gFSCI_MlmeNwkOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMlmeCnfHandler, 
                          NULL, 
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_McpsNwkOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMcpsCnfHandler, 
                          NULL, 
                          fsciInterfaceId );

    /* Register SAP Monitor Handler */
    FSCI_RegisterOpGroup( gFSCI_McpsSapId_c,
                          gFsciMonitorMode_c,
                          McpsSapMonitor,
                          NULL,
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_MlmeSapId_c,
                          gFsciMonitorMode_c,
                          MlmeSapMonitor,
                          NULL,
                          fsciInterfaceId );
#endif /* gFsciIncluded_c && gFsciHostMacSupport_c */
}

void MAC_Monitor(uint8_t sapId, void *pMsg, void* param, uint8_t instanceId)
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
   if(gFsciSAPHook_c == FSCI_Monitor(sapId, pMsg, param, fsciGetMacInterfaceId(instanceId)) )
   {
       
   }
#endif
}

#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
/*! *********************************************************************************
* \brief   This is the handler function for the MCPS Requests received over the
           serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMcpsReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    nwkToMcpsMessage_t *pMcpsReq;
   
    if( mFsciNwkMcpsDataReq_c == pClientPacket->structured.header.opCode )
    {
        pMcpsReq = MEM_BufferAlloc(sizeof(nwkToMcpsMessage_t) + gMaxPHYPacketSize_c);
    }
    else if( mFsciNwkMcpsPurgeReq_c == pClientPacket->structured.header.opCode )
    {
        pMcpsReq = MEM_BufferAlloc(sizeof(nwkToMcpsMessage_t));
    }
    else
    {
        MEM_BufferFree( pData );
        FSCI_Error( gFsciUnknownOpcode_c, interfaceId );
        return;
    }

    if( NULL == pMcpsReq )
    {
        MEM_BufferFree( pData );
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    switch( pClientPacket->structured.header.opCode ) {
    case mFsciNwkMcpsDataReq_c:
        {
            uint8_t *p = &pClientPacket->structured.payload[0];

            pMcpsReq->msgType = gMcpsDataReq_c;

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.dstAddr, p,
                        sizeof(pMcpsReq->msgData.dataReq.dstAddr) );
            p += sizeof(pMcpsReq->msgData.dataReq.dstAddr);

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.dstPanId, p,
                         sizeof(pMcpsReq->msgData.dataReq.dstPanId));
            p += sizeof(pMcpsReq->msgData.dataReq.dstPanId);

            pMcpsReq->msgData.dataReq.dstAddrMode   = (addrModeType_t)*p++;

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.srcAddr, p,
                        sizeof(pMcpsReq->msgData.dataReq.srcAddr) );
            p += sizeof(pMcpsReq->msgData.dataReq.srcAddr);

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.srcPanId, p,
                         sizeof(pMcpsReq->msgData.dataReq.srcPanId));
            p += sizeof(pMcpsReq->msgData.dataReq.srcPanId);

            pMcpsReq->msgData.dataReq.srcAddrMode   = (addrModeType_t)*p++;
            pMcpsReq->msgData.dataReq.msduLength    = *p++;
            pMcpsReq->msgData.dataReq.msduHandle    = *p++;
            pMcpsReq->msgData.dataReq.txOptions     = (macTxOptions_t)*p++;
            pMcpsReq->msgData.dataReq.securityLevel = (macSecurityLevel_t)*p++;
            pMcpsReq->msgData.dataReq.keyIdMode     = (keyIdModeType_t)*p++;

            FLib_MemCpy(&pMcpsReq->msgData.dataReq.keySource, p,
                        sizeof(pMcpsReq->msgData.dataReq.keySource));
            p += sizeof(pMcpsReq->msgData.dataReq.keySource);
            pMcpsReq->msgData.dataReq.keyIndex = *p++;
            pMcpsReq->msgData.dataReq.pMsdu = (uint8_t*)&pMcpsReq->msgData.dataReq.pMsdu +
                        sizeof(pMcpsReq->msgData.dataReq.pMsdu);
            FLib_MemCpy(pMcpsReq->msgData.dataReq.pMsdu, p,
                        pMcpsReq->msgData.dataReq.msduLength);
        }
        break;

    case mFsciNwkMcpsPurgeReq_c:
        pMcpsReq->msgType = gMcpsPurgeReq_c;
        pMcpsReq->msgData.purgeReq.msduHandle = pClientPacket->structured.payload[0];
        break;
    }

    /* To reduce peak memory usage, free the FSCI request before calling MAC SAPs */
    MEM_BufferFree( pData );
    NWK_MCPS_SapHandler( pMcpsReq, fsciGetMacInstanceId(interfaceId) );
#undef pClientPacket
}

#if gFsciIncluded_c && gFsciHostMacSupport_c
/*! *********************************************************************************
* \brief   This is the handler function for the MCPS Confirms and indications 
           received over the serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMcpsCnfHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    mcpsToNwkMessage_t *pMcpsCnf;
    uint8_t            *p = &pClientPacket->structured.payload[0];
    uint8_t             macInstance;
    uint8_t             fsciInterface;

    switch( pClientPacket->structured.header.opCode ) {
    case mFsciMcpsNwkDataCnf_c:
    case mFsciMcpsNwkPurgeCnf_c:
        pMcpsCnf = MEM_BufferAlloc(sizeof(mcpsToNwkMessage_t));
        break;
    case mFsciMcpsNwkDataInd_c:
    case mFsciMcpsNwkPromInd_c:
        pMcpsCnf = MEM_BufferAlloc(sizeof(mcpsToNwkMessage_t) + gMaxPHYPacketSize_c);
        break;
    default:
        MEM_BufferFree( pData );
        FSCI_Error( gFsciUnknownOpcode_c, interfaceId );
        return;        
    }

    if( NULL == pMcpsCnf )
    {
        MEM_BufferFree( pData );
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
        return;
    }
    
    switch( pClientPacket->structured.header.opCode ) {
    case mFsciMcpsNwkDataCnf_c:
        pMcpsCnf->msgType = gMcpsDataCnf_c;
        pMcpsCnf->msgData.dataCnf.msduHandle = *p++;
        pMcpsCnf->msgData.dataCnf.status = (resultType_t)*p++;
        break;
        
    case mFsciMcpsNwkPurgeCnf_c:
        pMcpsCnf->msgType = gMcpsPurgeCnf_c;
        pMcpsCnf->msgData.dataCnf.msduHandle = *p++;
        pMcpsCnf->msgData.dataCnf.status = (resultType_t)*p++;        
        break;
        
    case mFsciMcpsNwkDataInd_c:
        pMcpsCnf->msgType = gMcpsDataInd_c;
        
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.dstAddr, p,
                    sizeof(pMcpsCnf->msgData.dataInd.dstAddr));
        p += sizeof(pMcpsCnf->msgData.dataInd.dstAddr);
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.dstPanId, p,
                    sizeof(pMcpsCnf->msgData.dataInd.dstPanId));
        p += sizeof(pMcpsCnf->msgData.dataInd.dstPanId);
        pMcpsCnf->msgData.dataInd.dstAddrMode = (addrModeType_t)*p++;
          
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.srcAddr, p,
                    sizeof(pMcpsCnf->msgData.dataInd.srcAddr));
        p += sizeof(pMcpsCnf->msgData.dataInd.srcAddr);
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.srcPanId, p,
                    sizeof(pMcpsCnf->msgData.dataInd.srcPanId));
        p += sizeof(pMcpsCnf->msgData.dataInd.srcPanId);
        pMcpsCnf->msgData.dataInd.srcAddrMode = (addrModeType_t)*p++;
    
        pMcpsCnf->msgData.dataInd.msduLength = *p++;
        pMcpsCnf->msgData.dataInd.mpduLinkQuality = *p++;
        pMcpsCnf->msgData.dataInd.dsn = *p++;
        
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.timestamp, p,
                    sizeof(pMcpsCnf->msgData.dataInd.timestamp));
        p += sizeof(pMcpsCnf->msgData.dataInd.timestamp);
        
        pMcpsCnf->msgData.dataInd.securityLevel = (macSecurityLevel_t)*p++;
        pMcpsCnf->msgData.dataInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMcpsCnf->msgData.dataInd.keySource, p,
                    sizeof(pMcpsCnf->msgData.dataInd.keySource));
        p += sizeof(pMcpsCnf->msgData.dataInd.keySource);
        pMcpsCnf->msgData.dataInd.keyIndex = *p++;
        
        pMcpsCnf->msgData.dataInd.pMsdu = (uint8_t*)&pMcpsCnf->msgData.dataInd.pMsdu + 
                                          sizeof(pMcpsCnf->msgData.dataInd.pMsdu);
        FLib_MemCpy(pMcpsCnf->msgData.dataInd.pMsdu, p,
                    pMcpsCnf->msgData.dataInd.msduLength);
        break;
        
    case mFsciMcpsNwkPromInd_c:
        pMcpsCnf->msgType = gMcpsPromInd_c;
        pMcpsCnf->msgData.promInd.mpduLinkQuality = *p++;
        
        FLib_MemCpy(&pMcpsCnf->msgData.promInd.timeStamp, p,
                        sizeof(pMcpsCnf->msgData.promInd.timeStamp));
        p += sizeof(pMcpsCnf->msgData.promInd.timeStamp);
        
        pMcpsCnf->msgData.promInd.msduLength = *p++;
        pMcpsCnf->msgData.promInd.pMsdu = (uint8_t*)&pMcpsCnf->msgData.promInd.pMsdu + 
                                          sizeof(pMcpsCnf->msgData.promInd.pMsdu);
        FLib_MemCpy(pMcpsCnf->msgData.promInd.pMsdu, p,
                    pMcpsCnf->msgData.promInd.msduLength);
        break;       
    }

    macInstance = fsciHostGetMacInstanceId(interfaceId);
    fsciInterface = fsciGetMacInterfaceId(macInstance);
    
    /* Check if there is another FSCI interface for this MAC instance 
       and forwared FSCI message received on FSCI Host interface */
    if( mFsciInvalidInterface_c != fsciInterface )
    {
        FSCI_transmitFormatedPacket(pData, fsciInterface);
    }
    else
    {
        MEM_BufferFree(pData);
    }

    fsciHostMacInterfaces[macInstance].pfMCPS_NWK_SapHandler(pMcpsCnf, fsciHostMacInterfaces[macInstance].upperLayerId);
#undef pClientPacket    
}

/*! *********************************************************************************
* \brief   This is the handler function for the MLME Confirms and indications 
           received over the serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMlmeCnfHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    nwkMessage_t *pMlmeCnf;
    uint8_t      *p = &pClientPacket->structured.payload[0];
    uint8_t       macInstance;
    uint8_t       fsciInterface;
    
    switch( pClientPacket->structured.header.opCode ) {
    case mFsciMlmeNwkAssociateInd_c:
    case mFsciMlmeNwkAssociateCnf_c:
    case mFsciMlmeNwkDisassociateInd_c:
    case mFsciMlmeNwkDisassociateCnf_c:
    //case mFsciMlmeNwkGetCnf_c:
    case mFsciMlmeNwkGtsInd_c:
    case mFsciMlmeNwkGtsCnf_c:
    case mFsciMlmeNwkOrphanInd_c:
    //case mFsciMlmwNwkResetCnf_c:
    case mFsciMlmeNwkRxEnableCnf_c:
    case mFsciMlmeNwkScanCnf_c:
    case mFsciMlmeNwkCommStatusInd_c:
    //case mFsciMlmeNwkSetCnf_c:
    case mFsciMlmeNwkStartCnf_c:
    case mFsciMlmeNwkSyncLossInd_c:
    case mFsciMlmeNwkPollCnf_c:
    //case mFsciMlmeNwkErrorCnf_c:
    //case mFsciMlmeNwkBeaconStartInd_c:
    //case mFsciMlmeNwkMaintenanceScanCnf_c:
    case mFsciMlmeNwkPollNotifyInd_c:
    //case mFsciMlmeNwkSetSlotframeCnf_c:
    //case mFsciMlmeNwkSetLinkCnf_c:
    //case mFsciMlmeNwkTschModeCnf_c:
    //case mFsciMlmeNwkKeepAliveCnf_c:
    case mFsciMlmeNwkBeaconCnf_c:
        pMlmeCnf = MEM_BufferAlloc(sizeof(nwkMessage_t));
        break;
    case mFsciMlmeNwkBeaconNotifyInd_c:
        pMlmeCnf = MEM_BufferAlloc(sizeof(nwkMessage_t) + sizeof(panDescriptor_t));
        if( NULL == pMlmeCnf )
        {
            MEM_BufferFree( pData );
            FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
            return;
        }
        /* Allocate buffer for beacon payload separately, always freed later */
        pMlmeCnf->msgData.beaconNotifyInd.pBufferRoot = MEM_BufferAlloc(gMaxPHYPacketSize_c);
        if( NULL == pMlmeCnf->msgData.beaconNotifyInd.pBufferRoot )
        {
            MEM_BufferFree( pData );
            FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
            return;
        }
        break;        
    default:
        MEM_BufferFree( pData );
        FSCI_Error( gFsciUnknownOpcode_c, interfaceId );
        return;        
    }

    if( NULL == pMlmeCnf )
    {
        MEM_BufferFree( pData );
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
        return;
    }
    
    switch( pClientPacket->structured.header.opCode ) {
    case mFsciMlmeNwkAssociateInd_c:
        pMlmeCnf->msgType = gMlmeAssociateInd_c;
        FLib_MemCpy(&pMlmeCnf->msgData.associateInd.deviceAddress, p, 8);
        p += 8;
        pMlmeCnf->msgData.associateInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.associateInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.associateInd.keySource, p, 8);
        p += 8;
        pMlmeCnf->msgData.associateInd.keyIndex = *p++;
        pMlmeCnf->msgData.associateInd.capabilityInfo = (macCapabilityInfo_t)*p++;
        break;
            
    case mFsciMlmeNwkAssociateCnf_c:
        pMlmeCnf->msgType = gMlmeAssociateCnf_c;
        FLib_MemCpy(&pMlmeCnf->msgData.associateCnf.assocShortAddress, p, 2);
        p += 2;
        pMlmeCnf->msgData.associateCnf.status = (resultType_t)*p++;
        pMlmeCnf->msgData.associateCnf.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.associateCnf.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.associateCnf.keySource, p, 8);
        p += 8;
        pMlmeCnf->msgData.associateCnf.keyIndex = *p++;      
        break;
        
    case mFsciMlmeNwkDisassociateInd_c:
        pMlmeCnf->msgType = gMlmeDisassociateInd_c;
        FLib_MemCpy(&pMlmeCnf->msgData.disassociateInd.deviceAddress, p, 8);
        p += 8;
        pMlmeCnf->msgData.disassociateInd.disassociateReason = (macDisassociateReason_t)*p++;
        pMlmeCnf->msgData.disassociateInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.disassociateInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.disassociateInd.keySource, p, 8);
        p += 8;
        pMlmeCnf->msgData.disassociateInd.keyIndex = *p++;
        break;
        
    case mFsciMlmeNwkDisassociateCnf_c:
        pMlmeCnf->msgType = gMlmeDisassociateCnf_c;
        FLib_MemCpy(&pMlmeCnf->msgData.disassociateCnf.deviceAddress, p, 8);
        p += 8;
        FLib_MemCpy(&pMlmeCnf->msgData.disassociateCnf.devicePanId, p, 2);
        p += 2;
        pMlmeCnf->msgData.disassociateCnf.deviceAddrMode = (addrModeType_t)*p++;
        pMlmeCnf->msgData.disassociateCnf.status = (resultType_t)*p++;
        break;
        
    //case mFsciMlmeNwkGetCnf_c:
    //    break;
        
    case mFsciMlmeNwkGtsInd_c:
        pMlmeCnf->msgType = gMlmeGtsInd_c;
        FLib_MemCpy(&pMlmeCnf->msgData.gtsInd.deviceAddress, p, 2);
        p += 2;        
        *(uint8_t*)(&pMlmeCnf->msgData.gtsInd.gtsCharacteristics) = *p++;
        pMlmeCnf->msgData.gtsInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.gtsInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.gtsInd.keySource, p, 8);
        p += 8;        
        pMlmeCnf->msgData.gtsInd.keyIndex = *p++;
        break;
        
    case mFsciMlmeNwkGtsCnf_c:
        pMlmeCnf->msgType = gMlmeGtsCnf_c;
        pMlmeCnf->msgData.gtsCnf.status = (resultType_t)*p++;
        *(uint8_t*)(&pMlmeCnf->msgData.gtsCnf.gtsCharacteristics) = *p++;
        break;
        
    case mFsciMlmeNwkOrphanInd_c:
        pMlmeCnf->msgType = gMlmeOrphanInd_c;
        FLib_MemCpy(&pMlmeCnf->msgData.orphanInd.orphanAddress, p, 8);
        p += 8;         
        pMlmeCnf->msgData.orphanInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.orphanInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.orphanInd.keySource, p, 8);
        p += 8;        
        pMlmeCnf->msgData.orphanInd.keyIndex = *p++;
        break;
        
//    case mFsciMlmwNwkResetCnf_c:
//        break;
    
    case mFsciMlmeNwkRxEnableCnf_c:
        pMlmeCnf->msgType = gMlmeRxEnableCnf_c;
        pMlmeCnf->msgData.rxEnableCnf.status = (resultType_t)*p++;
        break;
        
    case mFsciMlmeNwkCommStatusInd_c:
        pMlmeCnf->msgType = gMlmeCommStatusInd_c;
        FLib_MemCpy(&pMlmeCnf->msgData.commStatusInd.srcAddress, p, 8);
        p += 8;
        FLib_MemCpy(&pMlmeCnf->msgData.commStatusInd.panId, p, 2);
        p += 2;         
        pMlmeCnf->msgData.commStatusInd.srcAddrMode = (addrModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.commStatusInd.destAddress, p, 8);
        p += 8;
        pMlmeCnf->msgData.commStatusInd.destAddrMode = (addrModeType_t)*p++;
        pMlmeCnf->msgData.commStatusInd.status = (resultType_t)*p++;
        pMlmeCnf->msgData.commStatusInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.commStatusInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.commStatusInd.keySource, p, 8);
        p += 8;
        pMlmeCnf->msgData.commStatusInd.keyIndex = *p++;
        break;
        
    //case mFsciMlmeNwkSetCnf_c:
    //    break;
        
    case mFsciMlmeNwkStartCnf_c:
        pMlmeCnf->msgType = gMlmeStartCnf_c;
        pMlmeCnf->msgData.startCnf.status = (resultType_t)*p++;        
        break;
        
    case mFsciMlmeNwkSyncLossInd_c:
        pMlmeCnf->msgType = gMlmeSyncLossInd_c;
        pMlmeCnf->msgData.syncLossInd.lossReason = (resultType_t)*p++;         
        FLib_MemCpy(&pMlmeCnf->msgData.syncLossInd.panId, p, 2);
        p += 2;        
        pMlmeCnf->msgData.syncLossInd.logicalChannel = (logicalChannelId_t)*p++;
#ifdef gPHY_802_15_4g_d        
        pMlmeCnf->msgData.syncLossInd.channelPage = gChannelPageId9_c;
#else
        pMlmeCnf->msgData.syncLossInd.channelPage = gChannelPageId0_c;
#endif
        pMlmeCnf->msgData.syncLossInd.securityLevel = (macSecurityLevel_t)*p++;
        pMlmeCnf->msgData.syncLossInd.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlmeCnf->msgData.syncLossInd.keySource, p, 8);
        p += 8;
        pMlmeCnf->msgData.syncLossInd.keyIndex = *p++;        
        break;
        
    case mFsciMlmeNwkPollCnf_c:
        pMlmeCnf->msgType = gMlmePollCnf_c;
        pMlmeCnf->msgData.pollCnf.status = (resultType_t)*p++;          
        break;
        
    //case mFsciMlmeNwkErrorCnf_c:
    //    break;
        
    //case mFsciMlmeNwkBeaconStartInd_c:
    //    break;
        
    //case mFsciMlmeNwkMaintenanceScanCnf_c:
    //    break;
        
    case mFsciMlmeNwkPollNotifyInd_c:
        pMlmeCnf->msgType = gMlmePollNotifyInd_c;
        pMlmeCnf->msgData.pollNotifyInd.srcAddrMode = (addrModeType_t)*p++; 
        FLib_MemCpy(&pMlmeCnf->msgData.pollNotifyInd.srcAddr, p, 8);
        p += 8;
        FLib_MemCpy(&pMlmeCnf->msgData.pollNotifyInd.srcPanId, p, 2);
        p += 2;
        break;
        
    //case mFsciMlmeNwkSetSlotframeCnf_c:
    //    break;
        
    //case mFsciMlmeNwkSetLinkCnf_c:
    //    break;
        
    //case mFsciMlmeNwkTschModeCnf_c:
    //    break;
        
    //case mFsciMlmeNwkKeepAliveCnf_c:
    //    break;
        
    case mFsciMlmeNwkBeaconCnf_c:
        pMlmeCnf->msgType = gMlmeBeaconCnf_c;
        pMlmeCnf->msgData.beaconCnf.status = (resultType_t)*p++;
        break;
        
    case mFsciMlmeNwkBeaconNotifyInd_c:
        {
            uint8_t pendAddrSize;
            
            pMlmeCnf->msgType = gMlmeBeaconNotifyInd_c;
            pMlmeCnf->msgData.beaconNotifyInd.bsn = *p++;
            pMlmeCnf->msgData.beaconNotifyInd.pendAddrSpec = *p++;
            pMlmeCnf->msgData.beaconNotifyInd.sduLength = *p++;
            
            pendAddrSize = 2 * (pMlmeCnf->msgData.beaconNotifyInd.pendAddrSpec & 0x07) +
                           8 * ((pMlmeCnf->msgData.beaconNotifyInd.pendAddrSpec >> 4) & 0x07);
            
            /* Pending address list points at start of pBufferRoot */
            pMlmeCnf->msgData.beaconNotifyInd.pAddrList = pMlmeCnf->msgData.beaconNotifyInd.pBufferRoot;
            
            /* Put pending short and extended addresses at start of pBufferRoot */
            FLib_MemCpy(pMlmeCnf->msgData.beaconNotifyInd.pAddrList, p, pendAddrSize);
            p += pendAddrSize;        

            /* PAN descriptor points at the end of the NWK message */
            pMlmeCnf->msgData.beaconNotifyInd.pPanDescriptor = (panDescriptor_t*)((uint8_t*)pMlmeCnf + sizeof(nwkMessage_t));        
            /* Fill PAN descriptor fields all at once as struct is packed */
            FLib_MemCpy(pMlmeCnf->msgData.beaconNotifyInd.pPanDescriptor, p, 
                        sizeof(panDescriptor_t) );
            p += sizeof(panDescriptor_t);
            
            /* Beacon payload points in pBufferRoot at the end of the pending address list */
            pMlmeCnf->msgData.beaconNotifyInd.pSdu = (uint8_t*)pMlmeCnf->msgData.beaconNotifyInd.pBufferRoot + pendAddrSize;
            /* Copy beacon payload */
            FLib_MemCpy(pMlmeCnf->msgData.beaconNotifyInd.pSdu, p, 
                        pMlmeCnf->msgData.beaconNotifyInd.sduLength );
            p += pMlmeCnf->msgData.beaconNotifyInd.sduLength;

#if gTschSupport_d            
            pMlmeCnf->msgData.beaconNotifyInd.ebsn = *p++;
            pMlmeCnf->msgData.beaconNotifyInd.beaconType = (beaconType_t)*p++;
#endif
        }    
        break;
        
    case mFsciMlmeNwkScanCnf_c:
        pMlmeCnf->msgType = gMlmeScanCnf_c;
        pMlmeCnf->msgData.scanCnf.status = (resultType_t)*p++;
        pMlmeCnf->msgData.scanCnf.scanType = (macScanType_t)*p++;
        pMlmeCnf->msgData.scanCnf.resultListSize = *p++;
        FLib_MemCpy(&pMlmeCnf->msgData.scanCnf.unscannedChannels, p, sizeof(channelMask_t));
        p += sizeof(channelMask_t);
        pMlmeCnf->msgData.scanCnf.resList.pEnergyDetectList = NULL;

        if( pMlmeCnf->msgData.scanCnf.resultListSize )
        {
            switch( pMlmeCnf->msgData.scanCnf.scanType ){
            case gScanModeED_c:
            case gScanModeFastED_c:
                /* Allocate buffer for ED list */
                pMlmeCnf->msgData.scanCnf.resList.pEnergyDetectList = MEM_BufferAlloc(pMlmeCnf->msgData.scanCnf.resultListSize);
                if( NULL == pMlmeCnf->msgData.scanCnf.resList.pEnergyDetectList )
                {
                    /* Free serial buffer and previously allocated NWK message */
                    MEM_BufferFree( pData );
                    MEM_BufferFree( pMlmeCnf );
                    FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
                    return;
                }
                
                FLib_MemCpy(pMlmeCnf->msgData.scanCnf.resList.pEnergyDetectList, p, 
                            pMlmeCnf->msgData.scanCnf.resultListSize);
                p += pMlmeCnf->msgData.scanCnf.resultListSize;              
                break;
            case gScanModeActive_c:
            case gScanModePassive_c:
                /* Allocate buffer for first PAN descriptor block */
                pMlmeCnf->msgData.scanCnf.resList.pPanDescriptorBlockList = (panDescriptorBlock_t*)MEM_BufferAlloc(sizeof(panDescriptorBlock_t));
                if( NULL == pMlmeCnf->msgData.scanCnf.resList.pPanDescriptorBlockList )
                {
                    /* Free serial buffer and previously allocated NWK message */
                    MEM_BufferFree( pData );
                    MEM_BufferFree( pMlmeCnf );
                    FSCI_Error( gFsciOutOfMessages_c, interfaceId );     
                    return;
                }                
                else
                {
                    panDescriptorBlock_t *pCrtPDBlock = pMlmeCnf->msgData.scanCnf.resList.pPanDescriptorBlockList;
                    panDescriptorBlock_t *pNewPDBlock = NULL;
                    
                    pCrtPDBlock->panDescriptorCount = 0;
                    pCrtPDBlock->pNext = NULL;
                    
                    for( uint32_t resIdx =0; resIdx<pMlmeCnf->msgData.scanCnf.resultListSize; resIdx++ )
                    {
                        /* Allocate a new block if current one is full */
                        if( pCrtPDBlock->panDescriptorCount >= gScanResultsPerBlock_c )
                        {
                            pNewPDBlock = (panDescriptorBlock_t*)MEM_BufferAlloc(sizeof(panDescriptorBlock_t));
                            if( NULL == pNewPDBlock )
                            {
                                /* Update number of results and stop parsing serial message */
                                pMlmeCnf->msgData.scanCnf.resultListSize = resIdx;
                                break;
                            }
                            else
                            {
                                pNewPDBlock->panDescriptorCount = 0;
                                pNewPDBlock->pNext = NULL;

                                pCrtPDBlock->pNext = pNewPDBlock;
                                pCrtPDBlock = pNewPDBlock;
                            } 
                        }
                        
                        /* Record PanDescriptor */
                        FLib_MemCpy(&pCrtPDBlock->panDescriptorList[pCrtPDBlock->panDescriptorCount], p, 
                                    sizeof(panDescriptor_t));
                        p += sizeof(panDescriptor_t);
                        pCrtPDBlock->panDescriptorCount++;
                    }
                }
                break;
            }
        }
        
        break;       
    }    
    
    macInstance = fsciHostGetMacInstanceId(interfaceId);
    fsciInterface = fsciGetMacInterfaceId(macInstance);
    
    /* Check if there is another FSCI interface for this MAC instance 
       and forwared FSCI message received on FSCI Host interface */
    if( mFsciInvalidInterface_c != fsciInterface )
    {
        FSCI_transmitFormatedPacket(pData, fsciInterface);
    }
    else
    {
        MEM_BufferFree(pData);
    }
    
    fsciHostMacInterfaces[macInstance].pfMLME_NWK_SapHandler(pMlmeCnf, fsciHostMacInterfaces[macInstance].upperLayerId);
#undef pClientPacket      
}

#endif /* gFsciIncluded_c && gFsciHostMacSupport_c */

/*! *********************************************************************************
* \brief   This is the handler function for All MLME Requests received over the
           serial interface
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMlmeReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    mlmeMessage_t  mlmeReq;    
    uint8_t        *p = &pClientPacket->structured.payload[0];

    switch (pClientPacket->structured.header.opCode) {
    case mFsciNwkMlmeGetReq_c:
        mlmeReq.msgType = gMlmeGetReq_c;
        mlmeReq.msgData.getReq.pibAttribute      = pClientPacket->structured.payload[0];
        mlmeReq.msgData.getReq.pibAttributeIndex = pClientPacket->structured.payload[1];
        mlmeReq.msgData.getReq.pibAttributeValue = &pClientPacket->structured.payload[2];
        break;

    case mFsciNwkMlmeSetReq_c:
        mlmeReq.msgType = gMlmeSetReq_c;
        mlmeReq.msgData.setReq.pibAttribute      = pClientPacket->structured.payload[0];
        mlmeReq.msgData.setReq.pibAttributeIndex = pClientPacket->structured.payload[1];
        mlmeReq.msgData.setReq.pibAttributeValue = &pClientPacket->structured.payload[2];
        break;

    case mFsciNwkMlmeResetReq_c:
        mlmeReq.msgType = gMlmeResetReq_c;
        mlmeReq.msgData.resetReq.setDefaultPIB = pClientPacket->structured.payload[0];
        break;

    case mFsciNwkMlmeSetSlotframeReq_c:
        mlmeReq.msgType = gMlmeSetSlotframeReq_c;
        mlmeReq.msgData.setSlotframeReq.slotframeHandle = *p++;
        mlmeReq.msgData.setSlotframeReq.operation = (macSetSlotframeOp_t)*p++;
        FLib_MemCpy(&mlmeReq.msgData.setSlotframeReq.size, p, sizeof(uint16_t));
        p += sizeof(uint16_t);
        break;
        
    case mFsciNwkMlmeSetLinkReq_c:
        mlmeReq.msgType = gMlmeSetLinkReq_c;
        mlmeReq.msgData.setLinkReq.operation = (macSetLinkOp_t)*p++;
        FLib_MemCpy(&mlmeReq.msgData.setLinkReq.linkHandle, p, sizeof(uint16_t));
        p += sizeof(uint16_t);       
        mlmeReq.msgData.setLinkReq.slotframeHandle = *p++;
        FLib_MemCpy(&mlmeReq.msgData.setLinkReq.timeslot, p, sizeof(uint16_t));
        p += sizeof(uint16_t);         
        FLib_MemCpy(&mlmeReq.msgData.setLinkReq.channelOffset, p, sizeof(uint16_t));
        p += sizeof(uint16_t);
        FLib_MemCpy(&mlmeReq.msgData.setLinkReq.linkOptions, p, sizeof(uint8_t));
        p += sizeof(uint8_t);        
        mlmeReq.msgData.setLinkReq.linkType = (macLinkType_t)*p++;
        FLib_MemCpy(&mlmeReq.msgData.setLinkReq.nodeAddr, p, sizeof(uint16_t));
        p += sizeof(uint16_t);    
        break;
        
    case mFsciNwkMlmeTschModeReq_c:
        mlmeReq.msgType = gMlmeTschModeReq_c;
        mlmeReq.msgData.tschModeReq.tschMode = (macTschMode_t)*p++;
        break;
        
    case mFsciNwkMlmeKeepAliveReq_c:
        mlmeReq.msgType = gMlmeKeepAliveReq_c;
        FLib_MemCpy(&mlmeReq.msgData.keepAliveReq.dstAddr, p, sizeof(uint16_t));
        p += sizeof(uint16_t);        
        FLib_MemCpy(&mlmeReq.msgData.keepAliveReq.keepAlivePeriod, p, sizeof(uint16_t));
        p += sizeof(uint16_t);
        break;        
        
    default:
        fsciMlmeAsyncReqHandler(pData, param, interfaceId);
        return;
    }

    MEM_BufferFree( pData );
    (void)NWK_MLME_SapHandler(&mlmeReq, fsciGetMacInstanceId(interfaceId));  
#undef pClientPacket
}

/*! *********************************************************************************
* \brief   This is the handler function for Asynchronous MLME Requests received
           over the serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMlmeAsyncReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    mlmeMessage_t  *pMlemReq = MEM_BufferAlloc(sizeof(mlmeMessage_t));
    uint8_t        *p = &pClientPacket->structured.payload[0];

    if (NULL == pMlemReq) {
        MEM_BufferFree( pData );
        FSCI_Error(gFsciOutOfMessages_c, interfaceId);
        return;
    }

    switch (pClientPacket->structured.header.opCode) {
    case mFsciNwkMlmeAssociateReq_c:
        pMlemReq->msgType = gMlmeAssociateReq_c;
        pMlemReq->msgData.associateReq.channelPage = gDefaultChannelPageId_c;
        FLib_MemCpy(&pMlemReq->msgData.associateReq.coordAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.associateReq.coordPanId,
                     p, sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.associateReq.coordAddrMode  = (addrModeType_t)*p++;
        pMlemReq->msgData.associateReq.logicalChannel = *p++;
        pMlemReq->msgData.associateReq.securityLevel  = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.associateReq.keyIdMode      = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.associateReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.associateReq.keyIndex = *p++;
        pMlemReq->msgData.associateReq.capabilityInfo = *p++;
        break;

    case mFsciNwkMlmeAssociateRes_c:
        pMlemReq->msgType = gMlmeAssociateRes_c;
        FLib_MemCpy(&pMlemReq->msgData.associateRes.deviceAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.associateRes.assocShortAddress, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.associateRes.securityLevel  = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.associateRes.keyIdMode      = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.associateRes.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.associateRes.keyIndex = *p++;
        pMlemReq->msgData.associateRes.status   = (resultType_t)*p++;
        break;

    case mFsciNwkMlmeDisassociateReq_c:
        pMlemReq->msgType = gMlmeDisassociateReq_c;
        FLib_MemCpy(&pMlemReq->msgData.disassociateReq.deviceAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.disassociateReq.devicePanId, p,
                     sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.disassociateReq.deviceAddrMode  = (addrModeType_t)*p++;
        pMlemReq->msgData.disassociateReq.disassociateReason = (macDisassociateReason_t)*p++;
        pMlemReq->msgData.disassociateReq.txIndirect = *p++;
        pMlemReq->msgData.disassociateReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.disassociateReq.keyIdMode     = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.disassociateReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.disassociateReq.keyIndex = *p++;
        break;

    case mFsciNwkMlmeGtsReq_c:
        pMlemReq->msgType = gMlmeGtsReq_c;
        *(uint8_t*)(&pMlemReq->msgData.gtsReq.gtsCharacteristics) = *p++;
        pMlemReq->msgData.gtsReq.securityLevel      = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.gtsReq.keyIdMode          = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.gtsReq.keySource, p,
                    sizeof(pMlemReq->msgData.gtsReq.keySource));
        p += sizeof(pMlemReq->msgData.gtsReq.keySource);
        pMlemReq->msgData.gtsReq.keyIndex = *p;
        break;

    case mFsciNwkMlmeOrphanRes_c:
        pMlemReq->msgType = gMlmeOrphanRes_c;
        FLib_MemCpy(&pMlemReq->msgData.orphanRes.orphanAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.orphanRes.shortAddress, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.orphanRes.associatedMember = *p++;
        pMlemReq->msgData.orphanRes.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.orphanRes.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.orphanRes.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.orphanRes.keyIndex = *p++;
        break;

    case mFsciNwkMlmeRxEnableReq_c:
        pMlemReq->msgType = gMlmeRxEnableReq_c;
        pMlemReq->msgData.rxEnableReq.deferPermit = *p++;
        pMlemReq->msgData.rxEnableReq.rxOnTime = 0;
        pMlemReq->msgData.rxEnableReq.rxOnDuration = 0;
        FLib_MemCpy(&pMlemReq->msgData.rxEnableReq.rxOnTime, p, 3);
        p += 3;
        FLib_MemCpy(&pMlemReq->msgData.rxEnableReq.rxOnDuration, p, 3);
#ifdef gMAC2011_d
        p += 3;
        pMlemReq->msgData.rxEnableReq.rangingRxControl = (macRangingRxControl_t)*p;
#endif
        break;

    case mFsciNwkMlmeScanReq_c:
        pMlemReq->msgType = gMlmeScanReq_c;
        pMlemReq->msgData.scanReq.scanType = (macScanType_t)*p++;
        FLib_MemCpy( &pMlemReq->msgData.scanReq.scanChannels, p, sizeof(channelMask_t) );
        p += sizeof(channelMask_t);
        pMlemReq->msgData.scanReq.scanDuration = *p++;
        pMlemReq->msgData.scanReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.scanReq.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.scanReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.scanReq.keyIndex = *p++;
        break;

    case mFsciNwkMlmeStartReq_c:
        pMlemReq->msgType = gMlmeStartReq_c;
        pMlemReq->msgData.startReq.channelPage = gDefaultChannelPageId_c;
        FLib_MemCpy( &pMlemReq->msgData.startReq.panId, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.startReq.logicalChannel = *p++;
        FLib_MemCpy( &pMlemReq->msgData.startReq.startTime, p, sizeof(uint32_t) );
        p += sizeof(uint32_t);
        pMlemReq->msgData.startReq.beaconOrder = *p++;
        pMlemReq->msgData.startReq.superframeOrder = *p++;
        pMlemReq->msgData.startReq.panCoordinator = *p++;
        pMlemReq->msgData.startReq.batteryLifeExtension = *p++;
        pMlemReq->msgData.startReq.coordRealignment = *p++;
        pMlemReq->msgData.startReq.coordRealignSecurityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.startReq.coordRealignKeyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.startReq.coordRealignKeySource, p, 8);
        p += 8;
        pMlemReq->msgData.startReq.coordRealignKeyIndex = *p++;
        pMlemReq->msgData.startReq.beaconSecurityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.startReq.beaconKeyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.startReq.beaconKeySource, p, 8);
        p += 8;
        pMlemReq->msgData.startReq.beaconKeyIndex = *p++;
        break;

    case mFsciNwkMlmeSyncReq_c:
        pMlemReq->msgType = gMlmeSyncReq_c;
        pMlemReq->msgData.syncReq.channelPage = gDefaultChannelPageId_c;
        pMlemReq->msgData.syncReq.logicalChannel = *p++;
        pMlemReq->msgData.syncReq.trackBeacon    = *p;
        break;

    case mFsciNwkMlmePollReq_c:
        pMlemReq->msgType = gMlmePollReq_c;
        FLib_MemCpy(&pMlemReq->msgData.pollReq.coordAddress, p, sizeof(pMlemReq->msgData.pollReq.coordAddress));
        p += sizeof(pMlemReq->msgData.pollReq.coordAddress);
        FLib_MemCpy( &pMlemReq->msgData.pollReq.coordPanId, p,
                     sizeof(pMlemReq->msgData.pollReq.coordPanId));
        p += sizeof(pMlemReq->msgData.pollReq.coordPanId);
        pMlemReq->msgData.pollReq.coordAddrMode = (addrModeType_t)*p++;
        pMlemReq->msgData.pollReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.pollReq.keyIdMode     = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.pollReq.keySource, p,
                    sizeof(pMlemReq->msgData.pollReq.keySource));
        p += sizeof(pMlemReq->msgData.pollReq.keySource);
        pMlemReq->msgData.pollReq.keyIndex = *p++;
        break;
        
    case mFsciNwkMlmeBeaconReq_c:
        pMlemReq->msgType = gMlmeBeaconReq_c;
        pMlemReq->msgData.beaconReq.beaconType = (beaconType_t)*p++;
        pMlemReq->msgData.beaconReq.channel = *p++;
        pMlemReq->msgData.beaconReq.channelPage = (channelPageId_t)*p++;
        pMlemReq->msgData.beaconReq.superframeOrder = *p++;
        pMlemReq->msgData.beaconReq.beaconSecurityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.beaconReq.beaconKeyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.beaconReq.beaconKeySource, p, sizeof(uint64_t));
        p += sizeof(uint64_t);
        pMlemReq->msgData.beaconReq.beaconKeyIndex = *p++;
        pMlemReq->msgData.beaconReq.dstAddrMode = (addrModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.beaconReq.dstAddr, p, sizeof(uint64_t));
        p += sizeof(uint64_t);
        pMlemReq->msgData.beaconReq.bsnSuppression = *p++;
        break;

    default:
        MEM_BufferFree( pData );
        FSCI_Error(gFsciUnknownOpcode_c, interfaceId);
        return;
    }
    /* To reduce peak memory usage, free the FSCI request before calling MAC SAPs */
    MEM_BufferFree( pData );
    if( gSuccess_c != NWK_MLME_SapHandler(pMlemReq, fsciGetMacInstanceId(interfaceId)) )
    {
        MEM_BufferFree( pMlemReq );
    }
#undef pClientPacket
}

/*! *********************************************************************************
* \brief   This function is called to monitor the MAC's MCPS SAP
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void McpsSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
#define pMscpCnf ((mcpsToNwkMessage_t*)pData)
#define pMcpsReq ((nwkToMcpsMessage_t*)pData)
    clientPacket_t     *pFsciPacket;
    uint8_t            *p;
    uint16_t            size = sizeof(clientPacketHdr_t) + sizeof(uint8_t);

    /* Determine the size needed to be allocated */
    switch( ((macMsgHeader_t*)pData)->msgType )
    {
    case gMcpsDataReq_c:
        {
            mcpsDataReq_t *pSrc = &pMcpsReq->msgData.dataReq;
            size += sizeof(mcpsDataReq_t) + pSrc->msduLength;
        }
        break;
    case gMcpsPurgeReq_c:
        size += sizeof(mcpsPurgeReq_t);
        break;
    case gMcpsDataInd_c:
        {
            mcpsDataInd_t *pSrc = &pMscpCnf->msgData.dataInd;
            size += sizeof(mcpsDataInd_t) + pSrc->msduLength;
        }
        break;
    case gMcpsDataCnf_c:
        size += sizeof(mcpsDataCnf_t);
        break;
    case gMcpsPurgeCnf_c:
        size += sizeof(mcpsPurgeCnf_t);
        break;
    case gMcpsPromInd_c:
        {
            mcpsPromInd_t *pSrc = &pMscpCnf->msgData.promInd;
            size += sizeof(mcpsPromInd_t) + pSrc->msduLength;
        }
        break;
    }
    
    pFsciPacket = MEM_BufferAlloc( size );
    if (NULL == pFsciPacket) {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    pFsciPacket->structured.header.opGroup = gFSCI_McpsNwkOpcodeGroup_c;
    p = pFsciPacket->structured.payload;

    switch( ((macMsgHeader_t*)pData)->msgType )
    {
/* MCPS requests */
    case gMcpsDataReq_c:         //87 00
        {
            mcpsDataReq_t *pSrc = &pMcpsReq->msgData.dataReq;

            pFsciPacket->structured.header.opCode = mFsciNwkMcpsDataReq_c;
            pFsciPacket->structured.header.opGroup = gFSCI_NwkMcpsOpcodeGroup_c;
            FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
            p += sizeof(pSrc->dstAddr);
            FLib_MemCpy( p, &pSrc->dstPanId, sizeof(pSrc->dstPanId) );
            p += sizeof(pSrc->dstPanId);
            *p++ = pSrc->dstAddrMode;

            FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
            p += sizeof(pSrc->srcAddr);
            FLib_MemCpy( p, &pSrc->srcPanId, sizeof(pSrc->srcPanId));
            p += sizeof(pSrc->srcPanId);
            *p++ = pSrc->srcAddrMode;

            *p++ = pSrc->msduLength;
            *p++ = pSrc->msduHandle;
            *p++ = pSrc->txOptions;
            *p++ = pSrc->securityLevel;
            *p++ = pSrc->keyIdMode;

            FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
            p += sizeof(pSrc->keySource);
            *p++ = pSrc->keyIndex;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;

    case gMcpsPurgeReq_c:        //87 01
        pFsciPacket->structured.header.opCode = mFsciNwkMcpsPurgeReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMcpsOpcodeGroup_c;
        *p++ = pMcpsReq->msgData.purgeReq.msduHandle;
        break;

/* MCPS confirms and indications */
    case gMcpsDataCnf_c:         //86 00
        pFsciPacket->structured.header.opCode = mFsciMcpsNwkDataCnf_c;
        *p++ = pMscpCnf->msgData.dataCnf.msduHandle;
        *p++ = pMscpCnf->msgData.dataCnf.status;
        break;

    case gMcpsDataInd_c:         //86 01
        {
            mcpsDataInd_t *pSrc = &pMscpCnf->msgData.dataInd;

            pFsciPacket->structured.header.opCode = mFsciMcpsNwkDataInd_c;
            FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
            p += sizeof(pSrc->dstAddr);
            FLib_MemCpy( p, &pSrc->dstPanId, sizeof(pSrc->dstPanId));
            p += sizeof(pSrc->dstPanId);
            *p++ = pSrc->dstAddrMode;

            FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
            p += sizeof(pSrc->srcAddr);
            FLib_MemCpy( p, &pSrc->srcPanId, sizeof(pSrc->srcPanId) );
            p += sizeof(pSrc->srcPanId);
            *p++ = pSrc->srcAddrMode;

            *p++ = pSrc->msduLength;
            *p++ = pSrc->mpduLinkQuality;
            *p++ = pSrc->dsn;
            FLib_MemCpy( p, &pSrc->timestamp, sizeof(uint32_t) );
            p += sizeof(uint32_t);
            *p++ = pSrc->securityLevel;
            *p++ = pSrc->keyIdMode;
            FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
            p += sizeof(pSrc->keySource);
            *p++ = pSrc->keyIndex;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;

    case gMcpsPurgeCnf_c:        //86 02
        pFsciPacket->structured.header.opCode = mFsciMcpsNwkPurgeCnf_c;
        *p++ = pMscpCnf->msgData.purgeCnf.msduHandle;
        *p++ = pMscpCnf->msgData.purgeCnf.status;
        break;

    case gMcpsPromInd_c:         //86 03
        {
            mcpsPromInd_t *pSrc = &pMscpCnf->msgData.promInd;

            pFsciPacket->structured.header.opCode = mFsciMcpsNwkPromInd_c;
            *p++ = pSrc->mpduLinkQuality;
            FLib_MemCpy( p, &pSrc->timeStamp, sizeof(uint32_t) );
            p += sizeof(uint32_t);
            *p++ = pSrc->msduLength;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;
    } /* switch( pMsg->msgType ) */

    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
#undef pMscpCnf
#undef pMcpsReq
}

/*! *********************************************************************************
* \brief   This function is called to monitor the MAC's MLME SAP
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void MlmeSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
#define pMlmeReq ((mlmeMessage_t*)pData)
#define pMlmeCnf ((nwkMessage_t*)pData)
    uint8_t        *p;
    clientPacket_t *pFsciPacket;
    uint16_t        size = sizeof(clientPacketHdr_t) + sizeof(uint8_t);

    /* Determine the size needed to be allocated */
    switch( pMlmeReq->msgType )
    {
    case gMlmeAssociateReq_c:
        size += sizeof(mlmeAssociateReq_t);
        break;
    case gMlmeAssociateRes_c:
        size += sizeof(mlmeAssociateRes_t);
        break;
    case gMlmeDisassociateReq_c:
        size += sizeof(mlmeDisassociateReq_t);
        break;
    case gMlmeGetReq_c:
        size += sizeof(mlmeGetReq_t);
        break;
    case gMlmeGtsReq_c:
        size += sizeof(mlmeGtsReq_t);
        break;
    case gMlmeOrphanRes_c:
        size += sizeof(mlmeOrphanRes_t);
        break;
    case gMlmeResetReq_c:
        size += sizeof(mlmeResetReq_t);
        break;
    case gMlmeRxEnableReq_c:
        size += sizeof(mlmeRxEnableReq_t);
        break;
    case gMlmeScanReq_c:
        size += sizeof(mlmeScanReq_t);
        break;
    case gMlmeSetReq_c:
#if gFsciHostMacSupport_c
        if( gMPibKey_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = sizeof(mlmeSetReq_t) + 16;
        }
#ifndef gMAC2011_d        
        else if( gMPibKeyIdLookupData_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = sizeof(mlmeSetReq_t) + 9;
        }
#endif
#if gTschSupport_d
        else if( gMPibTimeslotTemplate_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = sizeof(mlmeSetReq_t) + sizeof(macTimeslotTemplate_t);
        }
#endif
        else if( (gMPibBeaconPayload_c == pMlmeReq->msgData.setReq.pibAttribute) 
#if gTschSupport_d
         || (gMPibHoppingSequenceList_c == pMlmeReq->msgData.setReq.pibAttribute) 
#endif
          )
        {
            size = gFsciMaxPayloadLen_c - sizeof(mlmeSetReq_t);
        }
        else
        {
            size += sizeof(mlmeSetReq_t) + sizeof(uint64_t);
        }
#else      
        size += sizeof(mlmeSetReq_t) + mlmeGetSizeOfPIB(pMlmeReq->msgData.setReq.pibAttribute);
#endif  
        break;
    case gMlmeStartReq_c:
        size += sizeof(mlmeStartReq_t);
        break;
    case gMlmeSyncReq_c:
        size += sizeof(mlmeSyncReq_t);
        break;
    case gMlmePollReq_c:
        size += sizeof(mlmePollReq_t);
        break;
    case gMlmeSetSlotframeReq_c:
        size += sizeof(mlmeSetSlotframeReq_t);
        break;
    case gMlmeSetLinkReq_c:
        size += sizeof(mlmeSetLinkReq_t);
        break;
    case gMlmeTschModeReq_c:
        size += sizeof(mlmeTschModeReq_t);
        break;
    case gMlmeKeepAliveReq_c:
        size += sizeof(mlmeKeepAliveReq_t);
        break;
    case gMlmeBeaconReq_c:
        size += sizeof(mlmeBeaconReq_t);
        break;
        
    case gMlmeAssociateInd_c:
        size += sizeof(mlmeAssociateInd_t);
        break;
    case gMlmeAssociateCnf_c:
        size += sizeof(mlmeAssociateCnf_t);
        break;
    case gMlmeDisassociateInd_c:
        size += sizeof(mlmeDisassociateInd_t);
        break;
    case gMlmeDisassociateCnf_c:
        size += sizeof(mlmeDisassociateCnf_t);
        break;
    case gMlmeBeaconNotifyInd_c:
        size += gFsciMaxPayloadLen_c;
        break;
    case gMlmeGetCnf_c:
#if gFsciHostMacSupport_c
        if( (gMPibBeaconPayload_c == pMlmeReq->msgData.getReq.pibAttribute) 
#if gTschSupport_d
         || (gMPibHoppingSequenceList_c == pMlmeReq->msgData.getReq.pibAttribute) 
#endif
          )
        {
            size = gFsciMaxPayloadLen_c - sizeof(mlmeGetCnf_t);
        }
        else
        {
            size += sizeof(mlmeGetCnf_t) + sizeof(uint64_t);
        }
#else
        size += sizeof(mlmeGetCnf_t) + mlmeGetSizeOfPIB(pMlmeReq->msgData.getReq.pibAttribute);
#endif
        break;
    case gMlmeGtsInd_c:
        size += sizeof(mlmeGtsInd_t);
        break;
    case gMlmeGtsCnf_c:
        size += sizeof(mlmeGtsCnf_t);
        break;
    case gMlmeOrphanInd_c:
        size += sizeof(mlmeOrphanInd_t);
        break;
    case gMlmeResetCnf_c:
        size += sizeof(mlmeResetCnf_t);
        break;
    case gMlmeRxEnableCnf_c:
        size += sizeof(mlmeRxEnableCnf_t);
        break;
    case gMlmeScanCnf_c:
        size += gFsciMaxPayloadLen_c;
        break;
    case gMlmeCommStatusInd_c:
        size += sizeof(mlmeCommStatusInd_t);
        break;
    case gMlmeSetCnf_c:
        size += sizeof(mlmeSetCnf_t);
        break;
    case gMlmeStartCnf_c:
        size += sizeof(mlmeStartCnf_t);
        break;
    case gMlmeSyncLossInd_c:
        size += sizeof(mlmeSyncLossInd_t);
        break;
    case gMlmePollCnf_c:
        size += sizeof(mlmePollCnf_t);
        break;
    case gMlmePollNotifyInd_c:
        size += sizeof(mlmePollNotifyInd_t);
        break;
    case gMlmeSetSlotframeCnf_c:
        size += sizeof(mlmeSetSlotframeCnf_t);
        break;
    case gMlmeSetLinkCnf_c:
        size += sizeof(mlmeSetLinkCnf_t);
        break;
    case gMlmeTschModeCnf_c:
        size += sizeof(mlmeTschModeCnf_t);
        break;
    case gMlmeKeepAliveCnf_c:
        size += sizeof(mlmeKeepAliveCnf_t);
        break;
    case gMlmeBeaconCnf_c:
        size += sizeof(mlmeBeaconCnf_t);
        break;
    }

    pFsciPacket = MEM_BufferAlloc( size );
    if( NULL == pFsciPacket ) 
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    pFsciPacket->structured.header.opGroup = gFSCI_MlmeNwkOpcodeGroup_c;
    p = pFsciPacket->structured.payload;

    switch( pMlmeReq->msgType )
    {
/* MLME requests */
    case gMlmeAssociateReq_c:    //85 00
#define pSrc (&pMlmeReq->msgData.associateReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeAssociateReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->coordAddress, sizeof(pSrc->coordAddress));
        p += sizeof(pSrc->coordAddress);
        FLib_MemCpy( p, &pSrc->coordPanId, sizeof(pSrc->coordPanId) );
        p += sizeof(pSrc->coordPanId);
        *p++ = pSrc->coordAddrMode;
        *p++ = pSrc->logicalChannel;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->capabilityInfo;
#undef pSrc
        break;

    case gMlmeAssociateRes_c:    //85 01
#define pSrc (&pMlmeReq->msgData.associateRes)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeAssociateRes_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->assocShortAddress, sizeof(pSrc->assocShortAddress));
        p += sizeof(pSrc->assocShortAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->status;
#undef pSrc
        break;

    case gMlmeDisassociateReq_c: //85 02
#define pSrc (&pMlmeReq->msgData.disassociateReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeDisassociateReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->devicePanId, sizeof(pSrc->devicePanId) );
        p += sizeof(pSrc->devicePanId);
        *p++ = pSrc->deviceAddrMode;
        *p++ = pSrc->disassociateReason;
        *p++ = pSrc->txIndirect;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeGetReq_c:          //85 03
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeGetReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;

        *p++ = pMlmeReq->msgData.getReq.pibAttribute;
        *p++ = pMlmeReq->msgData.getReq.pibAttributeIndex;
        break;

    case gMlmeGtsReq_c:          //85 04
#define pSrc (&pMlmeReq->msgData.gtsReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeGtsReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = *((uint8_t*)&pSrc->gtsCharacteristics);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeOrphanRes_c:       //85 05
#define pSrc (&pMlmeReq->msgData.orphanRes)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeOrphanRes_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->orphanAddress, sizeof(pSrc->orphanAddress));
        p += sizeof(pSrc->orphanAddress);
        FLib_MemCpy( p, &pSrc->shortAddress, sizeof(pSrc->shortAddress) );
        p += sizeof(pSrc->shortAddress);
        *p++ = pSrc->associatedMember;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeResetReq_c:        //85 06
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeResetReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pMlmeReq->msgData.resetReq.setDefaultPIB;
        break;

    case gMlmeRxEnableReq_c:     //85 07
#define pSrc (&pMlmeReq->msgData.rxEnableReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeRxEnableReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->deferPermit;
        FLib_MemCpy(p, &pSrc->rxOnTime, 3);
        p += 3;
        FLib_MemCpy(p, &pSrc->rxOnDuration, 3);
        p += 3;
#undef pSrc
        break;

    case gMlmeScanReq_c:         //85 08
#define pSrc (&pMlmeReq->msgData.scanReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeScanReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->scanType;
        FLib_MemCpy(p, &pSrc->scanChannels, sizeof(channelMask_t));
        p += sizeof(channelMask_t);
        *p++ = pSrc->scanDuration;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeSetReq_c:          //85 09
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeSetReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pMlmeReq->msgData.setReq.pibAttribute;
        *p++ = pMlmeReq->msgData.setReq.pibAttributeIndex;
#if gFsciHostMacSupport_c
        if( gMPibKey_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = 16;
        }
#ifndef gMAC2011_d        
        else if( gMPibKeyIdLookupData_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = 9;
        }
#endif
#if gTschSupport_d
        else if( gMPibTimeslotTemplate_c == pMlmeReq->msgData.setReq.pibAttribute )
        {
            size = sizeof(macTimeslotTemplate_t);
        }
#endif        
        else if( (gMPibBeaconPayload_c == pMlmeReq->msgData.setReq.pibAttribute) 
#if gTschSupport_d
         || (gMPibHoppingSequenceList_c == pMlmeReq->msgData.setReq.pibAttribute) 
#endif
          )
        {
            size = gFsciMaxPayloadLen_c - sizeof(mlmeSetReq_t);
        }
        else
        {
            size = sizeof(uint64_t);
        }
#else        
        size = mlmeGetSizeOfPIB(pMlmeReq->msgData.setReq.pibAttribute);
#endif
        FLib_MemCpy(p, pMlmeReq->msgData.setReq.pibAttributeValue, size);
        p += size;
        break;

    case gMlmeStartReq_c:        //85 0A
#define pSrc (&pMlmeReq->msgData.startReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeStartReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy( p, &pSrc->panId, sizeof(pSrc->panId) );
        p += sizeof(uint16_t);
        *p++ = pSrc->logicalChannel;
        FLib_MemCpy( p, &pSrc->startTime, sizeof(uint32_t) );
        p += sizeof(uint32_t);
        *p++ = pSrc->beaconOrder;
        *p++ = pSrc->superframeOrder;
        *p++ = pSrc->panCoordinator;
        *p++ = pSrc->batteryLifeExtension;
        *p++ = pSrc->coordRealignment;
        *p++ = pSrc->coordRealignSecurityLevel;
        *p++ = pSrc->coordRealignKeyIdMode;
        FLib_MemCpy(p, &pSrc->coordRealignKeySource, sizeof(pSrc->coordRealignKeySource));
        p += sizeof(pSrc->coordRealignKeySource);
        *p++ = pSrc->coordRealignKeyIndex;
        *p++ = pSrc->beaconSecurityLevel;
        *p++ = pSrc->beaconKeyIdMode;
        FLib_MemCpy(p, &pSrc->beaconKeySource, sizeof(pSrc->beaconKeySource));
        p += sizeof(pSrc->beaconKeySource);
        *p++ = pSrc->beaconKeyIndex;
#undef pSrc
        break;

    case gMlmeSyncReq_c:         //85 0B
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeSyncReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pMlmeReq->msgData.syncReq.logicalChannel;
        *p++ = pMlmeReq->msgData.syncReq.trackBeacon;
        break;

    case gMlmePollReq_c:         //85 0C
#define pSrc (&pMlmeReq->msgData.pollReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmePollReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->coordAddress, sizeof(pSrc->coordAddress));
        p += sizeof(pSrc->coordAddress);
        FLib_MemCpy( p, &pSrc->coordPanId, sizeof(pSrc->coordPanId) );
        p += sizeof(pSrc->coordPanId);
        *p++ = pSrc->coordAddrMode;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeSetSlotframeReq_c: //85 0D
#define pSrc (&pMlmeReq->msgData.setSlotframeReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeSetSlotframeReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->slotframeHandle;
        *p++ = pSrc->operation;
        FLib_MemCpy(p, &pSrc->size, sizeof(pSrc->size));
        p += sizeof(pSrc->size);
#undef pSrc      
        break;
        
    case gMlmeSetLinkReq_c:      //85 0E
#define pSrc (&pMlmeReq->msgData.setLinkReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeSetLinkReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->operation;
        FLib_MemCpy(p, &pSrc->linkHandle, sizeof(pSrc->linkHandle));
        p += sizeof(pSrc->linkHandle);
        *p++ = pSrc->slotframeHandle;
        FLib_MemCpy(p, &pSrc->timeslot, sizeof(pSrc->timeslot));
        p += sizeof(pSrc->timeslot);
        FLib_MemCpy(p, &pSrc->channelOffset, sizeof(pSrc->channelOffset));
        p += sizeof(pSrc->channelOffset);
        FLib_MemCpy(p, &pSrc->linkOptions, sizeof(pSrc->linkOptions));
        p += sizeof(pSrc->linkOptions);
        *p++ = pSrc->linkType;
        FLib_MemCpy(p, &pSrc->nodeAddr, sizeof(pSrc->nodeAddr));
        p += sizeof(pSrc->nodeAddr);        
#undef pSrc 
        break;
        
    case gMlmeTschModeReq_c:     //85 0F
#define pSrc (&pMlmeReq->msgData.tschModeReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeTschModeReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->tschMode;
#undef pSrc 
        break;
        
    case gMlmeKeepAliveReq_c:    //85 10
#define pSrc (&pMlmeReq->msgData.keepAliveReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeKeepAliveReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
        p += sizeof(pSrc->dstAddr);
        FLib_MemCpy(p, &pSrc->keepAlivePeriod, sizeof(pSrc->keepAlivePeriod));
        p += sizeof(pSrc->keepAlivePeriod);                    
#undef pSrc 
        break;
        
    case gMlmeBeaconReq_c:       //85 11
#define pSrc (&pMlmeReq->msgData.beaconReq)
        pFsciPacket->structured.header.opCode = mFsciNwkMlmeBeaconReq_c;
        pFsciPacket->structured.header.opGroup = gFSCI_NwkMlmeOpcodeGroup_c;
        *p++ = pSrc->beaconType;
        *p++ = pSrc->channel;
        *p++ = pSrc->channelPage;
        *p++ = pSrc->superframeOrder;
        *p++ = pSrc->beaconSecurityLevel;
        *p++ = pSrc->beaconKeyIdMode;
        FLib_MemCpy(p, &pSrc->beaconKeySource, sizeof(pSrc->beaconKeySource));
        p += sizeof(pSrc->beaconKeySource);        
        *p++ = pSrc->beaconKeyIndex;
        *p++ = pSrc->dstAddrMode;
        FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
        p += sizeof(pSrc->dstAddr);
        *p++ = pSrc->bsnSuppression;
#undef pSrc 
        break;

/* MLME confirms and indications */
    case gMlmeAssociateInd_c:    //84 00
#define pSrc (&pMlmeCnf->msgData.associateInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkAssociateInd_c;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->capabilityInfo;
#undef pSrc
        break;

    case gMlmeAssociateCnf_c:    //84 01
#define pSrc (&pMlmeCnf->msgData.associateCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkAssociateCnf_c;
        
        FLib_MemCpy( p, &pSrc->assocShortAddress, sizeof(pSrc->assocShortAddress) );
        p += sizeof(pSrc->assocShortAddress);
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pSrc->status;
        }
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeDisassociateInd_c: //84 02
#define pSrc (&pMlmeCnf->msgData.disassociateInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkDisassociateInd_c;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        *p++ = pSrc->disassociateReason;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeDisassociateCnf_c: //84 03
#define pSrc (&pMlmeCnf->msgData.disassociateCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkDisassociateCnf_c;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->devicePanId, sizeof(pSrc->devicePanId) );
        p += sizeof(pSrc->devicePanId);
        *p++ = pSrc->deviceAddrMode;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint32_t)param;
        else
            *p++ = pSrc->status;
#undef pSrc
        break;

    case gMlmeBeaconNotifyInd_c: //84 04
#define pSrc (&pMlmeCnf->msgData.beaconNotifyInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkBeaconNotifyInd_c;
        *p++ = pSrc->bsn;
        *p++ = pSrc->pendAddrSpec;
        *p++ = pSrc->sduLength;
        //pending short address
        size = sizeof(uint16_t) * (pSrc->pendAddrSpec & 0x07);
        //pending extended address
        size += sizeof(uint64_t) * ((pSrc->pendAddrSpec >> 4) & 0x07);
        FLib_MemCpy(p, pSrc->pAddrList, size);
        p += size;

        FLib_MemCpy(p, pSrc->pPanDescriptor, sizeof(panDescriptor_t));
        p += sizeof(panDescriptor_t);
        FLib_MemCpy(p, pSrc->pSdu, pSrc->sduLength);
        p += pSrc->sduLength;
#if gTschSupport_d
        *p++ = pSrc->ebsn;
        *p++ = pSrc->beaconType;
#endif        
#undef pSrc
        break;

    case gMlmeGetCnf_c:          //84 05
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkGetCnf_c;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        *p++ = pMlmeReq->msgData.getReq.pibAttribute;
        *p++ = pMlmeReq->msgData.getReq.pibAttributeIndex;
#if gFsciHostMacSupport_c
        if( (gMPibBeaconPayload_c == pMlmeReq->msgData.getReq.pibAttribute) 
#if gTschSupport_d
         || (gMPibHoppingSequenceList_c == pMlmeReq->msgData.getReq.pibAttribute) 
#endif
          )
        {
            size = gFsciMaxPayloadLen_c - sizeof(mlmeGetCnf_t);
        }
        else
        {
            size = sizeof(uint64_t);
        }
#else          
        size = mlmeGetSizeOfPIB(pMlmeReq->msgData.getReq.pibAttribute);
#endif
        FLib_MemCpy( p, &size, sizeof(size) );
        p += sizeof(size);
        FLib_MemCpy(p, pMlmeReq->msgData.getReq.pibAttributeValue, size);
        p += size;
        break;

    case gMlmeGtsInd_c:          //84 06
#define pSrc (&pMlmeCnf->msgData.gtsInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkGtsInd_c;
        FLib_MemCpy( p, &pSrc->deviceAddress, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = *((uint8_t*)&pSrc->gtsCharacteristics);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy (p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeGtsCnf_c:          //84 07
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkGtsCnf_c;
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pMlmeCnf->msgData.gtsCnf.status;
        }
        *p++ = *((uint8_t*)&(pMlmeCnf->msgData.gtsCnf.gtsCharacteristics));
        break;

    case gMlmeOrphanInd_c:       //84 08
#define pSrc (&pMlmeCnf->msgData.orphanInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkOrphanInd_c;
        FLib_MemCpy(p, &pSrc->orphanAddress, sizeof(pSrc->orphanAddress));
        p += sizeof(pSrc->orphanAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeResetCnf_c:        //84 09
        pFsciPacket->structured.header.opCode = mFsciMlmwNwkResetCnf_c;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        break;

    case gMlmeRxEnableCnf_c:     //84 0A
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkRxEnableCnf_c;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint8_t)((uint32_t)param);
        else
            *p++ = pMlmeCnf->msgData.rxEnableCnf.status;
        break;

    case gMlmeScanCnf_c:         //84 0B
#define pSrc (&pMlmeCnf->msgData.scanCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkScanCnf_c;
        *p++ = pSrc->status;
        *p++ = pSrc->scanType;
        *p++ = pSrc->resultListSize;
        FLib_MemCpy( p, &pSrc->unscannedChannels, sizeof(channelMask_t) );
        p += sizeof(channelMask_t);
        
        if( pSrc->status == gSuccess_c )
        {
            if( (pSrc->scanType == gScanModeED_c) ||
                (pSrc->scanType == gScanModeFastED_c) )
            {
                size = pSrc->resultListSize * sizeof(pSrc->resList.pEnergyDetectList[0]);
                FLib_MemCpy(p, pSrc->resList.pEnergyDetectList, size);
                p += size;
            }
            else if( (pSrc->scanType == gScanModeActive_c) ||
                     (pSrc->scanType == gScanModePassive_c) )
            {
                uint16_t len = sizeof(pSrc->status) +
                               sizeof(pSrc->scanType) +
                               sizeof(pSrc->resultListSize) +
                               sizeof(pSrc->unscannedChannels);
                panDescriptorBlock_t* pCrtPDBlock = pSrc->resList.pPanDescriptorBlockList;
                uint8_t pdIdx = 0;
                uint8_t resIdx = 0;
                
                size = sizeof(panDescriptor_t);
                
                while ((pCrtPDBlock) &&
                       (resIdx < pSrc->resultListSize) &&
                       ( (len + size) < gFsciMaxPayloadLen_c ))
                {
                    FLib_MemCpy(p, &pCrtPDBlock->panDescriptorList[pdIdx], size);
                    p += size;
                    len += size;
                    resIdx++;
                    
                    if ( ++pdIdx >= gScanResultsPerBlock_c )
                    {
                        pCrtPDBlock = pCrtPDBlock->pNext;
                        pdIdx = 0;
                    }
                }
                
                /* store how many pan descriptors were put in buffer */
                pFsciPacket->structured.payload[2] = resIdx;
            }
        }
#undef pSrc
        break;

    case gMlmeCommStatusInd_c:   //84 0C
#define pSrc (&pMlmeCnf->msgData.commStatusInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkCommStatusInd_c;
        FLib_MemCpy(p, &pSrc->srcAddress, sizeof(pSrc->srcAddress));
        p += sizeof(pSrc->srcAddress);
        FLib_MemCpy( p, &pSrc->panId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = pSrc->srcAddrMode;
        FLib_MemCpy(p, &pSrc->destAddress, sizeof(pSrc->destAddress));
        p += sizeof(pSrc->destAddress);
        *p++ = pSrc->destAddrMode;
        
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint32_t)param;
        else
            *p++ = pSrc->status;
        
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeSetCnf_c:          //84 0D
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkSetCnf_c;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        *p++ = pMlmeReq->msgData.setReq.pibAttribute;
        *p++ = pMlmeReq->msgData.setReq.pibAttributeIndex;
        break;

    case gMlmeStartCnf_c:        //84 0E
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkStartCnf_c;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint8_t)((uint32_t)param);
        else
            *p++ = pMlmeCnf->msgData.startCnf.status;
        break;

    case gMlmeSyncLossInd_c:     //84 0F
#define pSrc (&pMlmeCnf->msgData.syncLossInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkSyncLossInd_c;
        *p++ = pSrc->lossReason;
        FLib_MemCpy( p, &pSrc->panId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = pSrc->logicalChannel;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmePollCnf_c:         //84 10
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkPollCnf_c;
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pMlmeCnf->msgData.pollCnf.status;
        }
        break;

    case gMlmePollNotifyInd_c:   //84 14
#define pSrc (&pMlmeCnf->msgData.pollNotifyInd)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkPollNotifyInd_c;
        *p++ = pSrc->srcAddrMode;
        FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
        p += sizeof(pSrc->srcAddr);
        FLib_MemCpy( p, &pSrc->srcPanId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
#undef pSrc
        break;
        
    case gMlmeSetSlotframeCnf_c: //84 15
#define pSrc (&pMlmeCnf->msgData.setSlotframeCnf)      
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkSetSlotframeCnf_c;
        *p++ = pSrc->slotframeHandle;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
#undef pSrc
        break;
        
    case gMlmeSetLinkCnf_c:      //84 16
#define pSrc (&pMlmeCnf->msgData.setLinkCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkSetLinkCnf_c;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        FLib_MemCpy(p, &pSrc->linkHandle, sizeof(pSrc->linkHandle));
        p += sizeof(pSrc->linkHandle);
        *p++ = pSrc->slotframeHandle;
#undef pSrc      
        break;
        
    case gMlmeTschModeCnf_c:     //84 17
#define pSrc (&pMlmeCnf->msgData.tschModeCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkTschModeCnf_c;
        *p++ = pSrc->tschMode;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
#undef pSrc 
        break;
        
    case gMlmeKeepAliveCnf_c:    //84 18
#define pSrc (&pMlmeCnf->msgData.keepAliveCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkKeepAliveCnf_c;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
#undef pSrc 
        break;
        
    case gMlmeBeaconCnf_c:       //84 19
#define pSrc (&pMlmeCnf->msgData.beaconCnf)
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkBeaconCnf_c;
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint8_t)((uint32_t)param);
        }
        else
        {
            *p++ = pSrc->status;
        }
#undef pSrc 
        break;       
        
/*
    case BeaconStartInd:         //84 12
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkBeaconStartInd_c;
        *p++ = source
        break;

    case MacMaintenanceScanCnf:  //84 13
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkMaintenanceScanCnf_c;
        *p++ = status
        break;
*/
    default:                     //84 11
        pFsciPacket->structured.header.opCode = mFsciMlmeNwkErrorCnf_c;
        //*p++ = error...
    }

    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
#undef pMlmeReq
#undef pMlmeCnf
}
#endif /* gFsciIncluded_c && gFSCI_IncludeMacCommands_c */

/*! *********************************************************************************
* \brief   This function determines the instance of the MAC registered on the 
*          specified interface
*
* \param[in] interfaceId
*
* \return The instance of the MAC
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
instanceId_t fsciGetMacInstanceId( uint32_t interfaceId )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    uint32_t i;

    for( i=0; i<gMacInstancesCnt_c; i++ )
    {
        if( interfaceId == fsciToMacBinding[i] )
        {
            return (instanceId_t)i;
        }
    }
#endif
    return gInvalidInstanceId_c;
}

/*! *********************************************************************************
* \brief   This function determines the interface Id of the specified MAC instance
*
* \param[in] macInstance
*
* \return The interface Id on which the MAC is registered
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
uint32_t fsciGetMacInterfaceId( instanceId_t macInstance )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    return fsciToMacBinding[macInstance];
#else
    return 0;
#endif
}

/*! *********************************************************************************
* \brief   This function determines the instance of the MAC registered on the 
*          specified interface on FSCI host
*
* \param[in] interfaceId
*
* \return The instance of the MAC
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
instanceId_t fsciHostGetMacInstanceId( uint32_t interfaceId )
{
#if gFsciIncluded_c && gFsciHostMacSupport_c
    uint32_t i;

    for( i=0; i<gMacInstancesCnt_c; i++ )
    {
        if( interfaceId == fsciHostMacInterfaces[i].fsciInterfaceId )
        {
            return (instanceId_t)i;
        }
    }
#endif
    return gInvalidInstanceId_c;
}

/*! *********************************************************************************
* \brief   This function determines the interface Id of the specified MAC 
*          instance registered on FSCI host
*
* \param[in] macInstance
*
* \return The interface Id on which the MAC is registered
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
uint32_t fsciHostGetMacInterfaceId( instanceId_t macInstance )
{
#if gFsciIncluded_c && gFsciHostMacSupport_c
    return fsciHostMacInterfaces[macInstance].fsciInterfaceId;
#else
    return 0;
#endif
}

#if gFsciIncluded_c && gFsciHostMacSupport_c
/*! *********************************************************************************
* \brief  This function will bind an instance of the upper layet to a MAC instance 
*
* \param[in]  nwkId instance of the upper layer
*
* \return  instanceId_t instance of the MAC
*
********************************************************************************** */
instanceId_t BindToMAC( instanceId_t nwkId )
{
    uint32_t i;
    for( i=0; i<gMacInstancesCnt_c; i++)
    {
        if( fsciHostMacInterfaces[i].upperLayerId == gInvalidInstanceId_c )
        {
            fsciHostMacInterfaces[i].upperLayerId = nwkId;
            return (instanceId_t)i;
        }
    }
    
    return gInvalidInstanceId_c;
}

/*! *********************************************************************************
* \brief  This function will unbind a MAC instance from the upper layer
*
* \param[in]  macId instance of the MAC
*
********************************************************************************** */
void UnBindFromMAC ( instanceId_t macId )
{
    fsciHostMacInterfaces[macId].upperLayerId = gInvalidInstanceId_c;
}

void Mac_RegisterSapHandlers( MCPS_NWK_SapHandler_t pMCPS_NWK_SapHandler,
                              MLME_NWK_SapHandler_t pMLME_NWK_SapHandler,
                              instanceId_t macInstanceId )
{
    fsciHostMacInterfaces[macInstanceId].pfMCPS_NWK_SapHandler = pMCPS_NWK_SapHandler;
    fsciHostMacInterfaces[macInstanceId].pfMLME_NWK_SapHandler = pMLME_NWK_SapHandler;
}

/*! *********************************************************************************
* \brief  Dummy SAP handler
*
* \param[in]  pMsg
* \param[in]  instanceId
*
* \return  None.
*
********************************************************************************** */
resultType_t Dumy_MCPS_NWK_SapHandler (mcpsToNwkMessage_t* pMsg, instanceId_t instanceId)
{
    MEM_BufferFree(pMsg);
    return gSuccess_c;
}

/*! *********************************************************************************
* \brief  Dummy SAP handler
*
* \param[in]  pMsg
* \param[in]  instanceId
*
* \return  None.
*
********************************************************************************** */
resultType_t Dumy_MLME_NWK_SapHandler (nwkMessage_t* pMsg, instanceId_t instanceId)
{
    MEM_BufferFree(pMsg);
    return gSuccess_c;
}

/*! *********************************************************************************
* \brief  This function handles MCPS requests
*
* \param[in] pMsg Message containing the message type and
*                          a pointer to the request
* \param[in] instanceId instance of the MAC
*
* \return  resultType_t.
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 7
resultType_t NWK_MCPS_SapHandler (nwkToMcpsMessage_t* pMsg, instanceId_t instanceId)
{
    if( instanceId >= gMacInstancesCnt_c )
    {
        return gInvalidParameter_c;
    }
    
    OSA_MutexLock(&fsciHostMacInterfaces[instanceId].fsciHostSAPMutex, OSA_WAIT_FOREVER);

    /* Monitor message from fsci interface */
    FSCI_Monitor(gFSCI_McpsSapId_c, pMsg, gSuccess_c, fsciGetMacInterfaceId(instanceId));
    
    /* Parse message and send request to fsci host interface */
    FSCI_Monitor(gFSCI_McpsSapId_c, pMsg, gSuccess_c, fsciHostGetMacInterfaceId(instanceId));
    
    /* Async request can be freed now */
    MEM_BufferFree(pMsg);
    
    OSA_MutexUnlock(&fsciHostMacInterfaces[instanceId].fsciHostSAPMutex);
    
    return gSuccess_c;
}

/*! *********************************************************************************
* \brief  This function handles MLME requests
*
* \param[in]  pMsg Message containing the message type and
*                           a pointer to the request
* \param[in] instanceId instance of the MAC
*
* \return  resultType_t.
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 8
resultType_t NWK_MLME_SapHandler (mlmeMessage_t* pMsg, instanceId_t instanceId)
{    
    uint8_t      *p = NULL;
    uint16_t     size = 0;
    resultType_t status = gSuccess_c;
    uint8_t      fsciInterface;
#if gFsciHostSyncUseEvent_c    
    event_flags_t fsciHostSyncRspEventFlags; 
#endif
  
    if( instanceId >= gMacInstancesCnt_c )
        return gInvalidParameter_c;

    OSA_MutexLock(&fsciHostMacInterfaces[instanceId].fsciHostSAPMutex, OSA_WAIT_FOREVER);    
    
    fsciInterface = fsciGetMacInterfaceId(instanceId);
    
    /* Monitor message from fsci interface */
    FSCI_Monitor(gFSCI_MlmeSapId_c, pMsg, gSuccess_c, fsciInterface);
    
    /* Parse message and send request to fsci host interface */
    FSCI_Monitor(gFSCI_MlmeSapId_c, pMsg, gSuccess_c, fsciHostGetMacInterfaceId(instanceId));

    /* Check for sync requests */
    switch( pMsg->msgType )
    {
    case gMlmeGetReq_c: 
        pMsg->msgType = gMlmeGetCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkGetCnf_c;
        break;

    case gMlmeResetReq_c:
        pMsg->msgType = gMlmeResetCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmwNwkResetCnf_c;
        break;

    case gMlmeSetReq_c:
        pMsg->msgType = gMlmeSetCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkSetCnf_c;
        break;

    case gMlmeSetSlotframeReq_c:
        pMsg->msgType = gMlmeSetSlotframeCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkSetSlotframeCnf_c;
        break;

    case gMlmeSetLinkReq_c:
        pMsg->msgType = gMlmeSetLinkCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkSetLinkCnf_c;
        break;

    case gMlmeTschModeReq_c:
        pMsg->msgType = gMlmeTschModeCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkTschModeCnf_c;
        break;

    case gMlmeKeepAliveReq_c:
        pMsg->msgType = gMlmeKeepAliveCnf_c;
        gFsciHostWaitingOpCode = mFsciMlmeNwkKeepAliveCnf_c;
        break;

    default:
        /* Async request can be freed now */
        MEM_BufferFree(pMsg);
        OSA_MutexUnlock(&fsciHostMacInterfaces[instanceId].fsciHostSAPMutex);
        return gSuccess_c;
    }

    gFsciHostWaitingSyncRsp = TRUE;
    gFsciHostWaitingOpGroup = gFSCI_MlmeNwkOpcodeGroup_c;
    pFsciHostSyncRsp = NULL;
    
    /* Wait for result from the serial interface */
    while( gFsciHostWaitingSyncRsp && !pFsciHostSyncRsp )
    {
#if gFsciHostSyncUseEvent_c
        OSA_EventWait(&gFsciHostSyncRspEvent, 0x00FFFFFF, FALSE, OSA_WAIT_FOREVER ,&fsciHostSyncRspEventFlags);
#else
        FSCI_receivePacket((void*)fsciHostGetMacInterfaceId(instanceId));
#endif
        
        if( NULL != pFsciHostSyncRsp )
        {
            p = pFsciHostSyncRsp->structured.payload;      
        }
        else
        {
            continue;
        }

        /* Check that response matches the sync request */
        switch( pMsg->msgType )
        {
        case gMlmeGetCnf_c:
            if( mFsciMlmeNwkGetCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            { 
                status = (resultType_t)*p++;
                /* Skip pib attribute and index */
                p += 2;
                FLib_MemCpy(&size, p, 2);
                p += 2;
                FLib_MemCpy(pMsg->msgData.getReq.pibAttributeValue, p, size);
                p += size;
            }
            break;

        case gMlmeResetCnf_c:
            if( mFsciMlmwNwkResetCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                status = (resultType_t)*p++;
            }            
            break;

        case gMlmeSetCnf_c:
            if( mFsciMlmeNwkSetCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                status = (resultType_t)*p++;
            }            
            break;

        case gMlmeSetSlotframeCnf_c:
            if( mFsciMlmeNwkSetSlotframeCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                /* Skip slotframe handle */
                p += 1;
                status = (resultType_t)*p++;
            }            
            break;

        case gMlmeSetLinkCnf_c:
            if( mFsciMlmeNwkSetLinkCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                status = (resultType_t)*p++;
            }            
            break;

        case gMlmeTschModeCnf_c:
            if( mFsciMlmeNwkTschModeCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                /* Skip tsch mode */
                p += 1;
                status = (resultType_t)*p++;
            }            
            break;

        case gMlmeKeepAliveCnf_c:
            if( mFsciMlmeNwkKeepAliveCnf_c == pFsciHostSyncRsp->structured.header.opCode )
            {
                status = (resultType_t)*p++;
            }            
            break;

        default:
            break;
        }        
    }
    
    gFsciHostWaitingSyncRsp = FALSE;
   
    /* Check if there is another FSCI interface for this MAC instance 
       and forwared FSCI message received on FSCI Host interface */
    if( mFsciInvalidInterface_c != fsciInterface )
    {
        FSCI_transmitFormatedPacket(pFsciHostSyncRsp, fsciInterface);
    }
    else
    {
        MEM_BufferFree(pFsciHostSyncRsp);
        pFsciHostSyncRsp = NULL;
    }
    
    OSA_MutexUnlock(&fsciHostMacInterfaces[instanceId].fsciHostSAPMutex);
    
    return status;
}

#endif /* gFsciIncluded_c && gFsciHostMacSupport_c */

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/************************************************************************************/
