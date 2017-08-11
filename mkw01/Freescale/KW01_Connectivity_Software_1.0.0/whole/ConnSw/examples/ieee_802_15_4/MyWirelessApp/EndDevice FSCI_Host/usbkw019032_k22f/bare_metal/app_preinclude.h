#ifndef __APP_PREINCLUDE_H__
#define __APP_PREINCLUDE_H__

/* 
 * KSDK configuration 
 */
#define CPU_MK22FN512VMP12              1

/* 
 * 802.15.4 MAC configuration 
 */
#define gMacFeatureSet_d gMacFeatureSet_06gM0_d
#define gPHY_802_15_4g_d 1

/* 
 * Connectivity Framework configuration 
 */
#define gFsciIncluded_c                 1
#define gFsciMaxInterfaces_c            1
#define gFsciHostMacSupport_c           1
#define gFSCI_IncludeMacCommands_c      1
#define gFSCI_IncludeLpmCommands_c      0
#define gFsciLenHas2Bytes_c             1
#define gFsciMaxPayloadLen_c            295
#define gSerialManagerMaxInterfaces_c   2
#define gSerialMgrUseUSB_c              1
/* Select the serial type used for inter-processor communication */
#define gSerialMgrUseUart_c             1
#define gSerialMgrUseSPI_c              0
#define gSerialMgrUseIIC_c              0

#define gKBD_KeysCount_c                1
#define gLEDsOnTargetBoardCnt_c         1
#define gEepromType_d                   gEepromDevice_None_c

#if gSerialMgrUseUart_c
    #define gAppFSCIHostInterfaceBaud_d     gUARTBaudRate115200_c
    #define gAppFSCIHostInterfaceType_d     gSerialMgrUart_c
    #define gAppFSCIHostInterfaceId_d       1
#elif gSerialMgrUseSPI_c
    #define gAppFSCIHostInterfaceBaud_d     gSPI_BaudRate_1000000_c
    #define gAppFSCIHostInterfaceType_d     gSerialMgrSPIMaster_c
    #define gAppFSCIHostInterfaceId_d       0
#elif gSerialMgrUseIIC_c
    #define gAppFSCIHostInterfaceBaud_d     gIIC_BaudRate_100000_c
    #define gAppFSCIHostInterfaceType_d     gSerialMgrIICMaster_c
    #define gAppFSCIHostInterfaceId_d       0
#endif

/* Memory pools */
#define PoolsDetails_c \
         _block_size_  64  _number_of_blocks_    8 _eol_  \
         _block_size_ 128  _number_of_blocks_    2 _eol_  \
         _block_size_ 320  _number_of_blocks_    6 _eol_
     
#endif /* __APP_PREINCLUDE_H__ */