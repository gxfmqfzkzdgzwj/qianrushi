#ifndef __APP_PREINCLUDE_H__
#define __APP_PREINCLUDE_H__

/* 
 * KSDK configuration 
 */
#define CPU_MKW01Z128CHN4 1
#define IAR 1

/*
 * SMAC
 */
#define gSmacSupported 1
#define CT_Feature_Calibration 1

/*
 * PHY
 */
#define gAspCapability_d 1

/* 
 * PHY Mode
 */
#define gPHY_802_15_4g_d 1
#define gPhyModeDefault_d gPhyMode1_c

/* 
 * PHY Frequency Band - Select only one frequency band
 */
#define gFreqBand_169_400__169_475MHz_d 0
#define gFreqBand_470__510MHz_d 0
#define gFreqBand_779__787MHz_d 0
#define gFreqBand_863__870MHz_d 0
#define gFreqBand_902__928MHz_d 1
#define gFreqBand_920__928MHz_d 0   
#define gFreqBand_865__867MHz_d 0

/*
 *Memory pools
 */
#define PoolsDetails_c \
         _block_size_  64  _number_of_blocks_    8 _eol_  \
         _block_size_ 128  _number_of_blocks_    2 _eol_  \
         _block_size_ 320  _number_of_blocks_    6 _eol_
     
/* 
 *FWK
 */        
#define gStackTimerInstance_c 1
#define gNvDisableIntCmdSeq_c 0
#define gRNG_UsePhyRngForInitialSeed_d 1
#define gSerialManagerMaxInterfaces_c 1
#define gKBD_KeysCount_c 1
#define gLEDsOnTargetBoardCnt_c 1

#endif /* __APP_PREINCLUDE_H__ */