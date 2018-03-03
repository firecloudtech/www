//hhh
//jjjjj
/** ###################################################################*/
/*!
** @file BCC1.c
** @version 01.00
** @brief
**         This component "BCC_MC3377x" encapsulates function of MC33771
**         Battery Cell Controller. It provides methods for SPI and
**         Twisted Pair Line (TPL) communication mode, voltage,
**         current and temperature measurement, fault handling and
**         implements diagnostic procedures.
*/         
/*!
**  @addtogroup BCC1_module BCC1 module documentation
**  @{
*/         

/* MODULE BCC1. */

#include "includes.h"

/*To configure the 33771 for Daisy Chain Fault output set the GPIO0 port as an input.
1.Set GPIO0 as an input GPIO0_CFG = 10.
2.Disable wake-up on GPIO0 with GPIO0_WU = 0.
3.Set GPIO0 to propagate signal to FAULT pin with GPIO0_FLT_ACT = 1.
To configure the 33771 to heart beat, the signal set the SYS_CFG1[FAULT_WAVE,WAVE_DC_BITx] to enable the heart beat and set the desired off time.
*/                                                                                                                      
#ifdef __cplusplus
extern "C" {
#endif 

uint8_t          TxBufPtr[BCC_MSG_SIZE];
uint8_t          Rc;                     /* Rolling counter value. */
uint16_t          TagID = 0;                  /* Tag ID value. */           //经过位数变换得到的
uint16_t         RegVal2, tagidcal, tagidback;    //进行ADC转换时写入ADC_CFG的值，包含TagID信息
boolean BalSet_Back[12][14];
uint8_t BCC1_DevCnt;
uint16_t NVM_CellUv, NVM_CellOv;
uint16_t     NVM_ChSelect[12];
int16_t NVM_CellUt[12][6], NVM_CellOt[12][6];


extern void delay(uint16 delayCount);
extern void delay_10us(uint16 delayCount);
extern void delay_ms(uint16 delayCount);

/* Initial configuration from Processor Expert properties. */
/* Note: INIT register is initialized automatically (no PEx properties dependency). */
/* Note: SYS_CFG_GLOBAL reg. contains only command GO2SLEEP (no initialization needed). */
/* Initial value of SYS_CFG1 register (configuration 1). */
#define BCC_CONF1_SYS_CFG1_VALUE ( \
  BCC_CYCLIC_TIMER_DISABLED | \
  BCC_DIAG_TIMEOUT_8S | \
  BCC_I_MEAS_DISABLED | \
  BCC_CB_AUTO_PAUSE_ENABLED | \
  BCC_CB_DRV_ENABLED | \
  BCC_DIAG_MODE_DISABLED | \
  BCC_CB_MAN_PAUSE_DISABLED | \
  BCC_SW_RESET_DISABLED | \
  BCC_FAULT_WAVE_DISABLED | \
  BCC_WAVE_DC_500US | \
  BCC_OSC_MON_ENABLED \
)

/* Initial value of SYS_CFG2 register (configuration 1). */

#define BCC_CONF1_SYS_CFG2_VALUE ( \
    BCC_FM_CP_ENABLED | \
    BCC_AUTO_SWITCH_SLEEP_ENABLED | \
    BCC_TIMEOUT_COMM_256MS | \
    BCC_EVEN_CELLS | \
    BCC_HAMM_ENCOD_ENABLED \
)

/* Initial value of SYS_DIAG register (configuration 1). */
#define BCC_CONF1_SYS_DIAG_VALUE ( \
  BCC_FAULT_PIN_PACK_CTRL | \
  BCC_IMUX_ISENSE | \
  BCC_ISENSE_OL_DIAG_DISABLED | \
  BCC_ANX_OL_DIAG_DISABLED | \
  BCC_ANX_DIAG_SW_OPEN | \
  BCC_DA_DIAG_NO_CHECK | \
  BCC_POL_NON_INVERTED | \
  BCC_CT_LEAK_DIAG_NORMAL | \
  BCC_CT_OV_UV_DISABLED | \
  BCC_CT_OL_ODD_OPEN | \
  BCC_CT_OL_EVEN_OPEN | \
  BCC_CB_OL_ODD_OPEN | \
  BCC_CB_OL_EVEN_OPEN \
)

/* Initial value of ADC_CFG register (configuration 1). */
#define BCC_CONF1_ADC_CFG_VALUE ( \
  /* Note: TAG_ID is zero. */ \
  /* Note: SOC is disable (i.e. do not initiate on-demand conversion now). */ \
  BCC_ADC2_PGA_AUTO | \
  /* Note: CC_RST is not set (do not reset CC now). */ \
  BCC_CHAR_COMP_ENABLED | \
  BCC_ADC1_A_RES_16BIT | \
  BCC_ADC1_B_RES_16BIT | \
  BCC_ADC2_RES_16BIT \
)

/* Initial value of ADC2_OFFSET_COMP register (configuration 1). */
#define BCC_CONF1_ADC2_OFFSET_COMP_VALUE (\
  BCC_READ_CC_RESET | \
  BCC_FREE_CC_CLAMP | \
  BCC_GET_ADC2_OFFSET(0) /* ADC2 offset compensation value. */ \
)

/* Initial value of OV_UV_EN register (configuration 1). */
#define BCC_CONF1_OV_UV_EN_VALUE ( \
  BCC_CTX_OV_TH_COMMON | \
  BCC_CTX_UV_TH_COMMON | \
  /* CTs OV and UV enable (bit is 1) or disable (bit is 0). */ \
  0x3FFFU \
)

/* Initial value of CELL_OV_FLT register (configuration 1). */
#define BCC_CONF1_CELL_OV_FLT_VALUE 0x0008U

/* Initial value of CELL_UV_FLT register (configuration 1). */
#define BCC_CONF1_CELL_UV_FLT_VALUE 0x0000U

/* Initial value of CB1_CFG register (configuration 1). */
#define BCC_CONF1_CB1_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB2_CFG register (configuration 1). */
#define BCC_CONF1_CB2_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB3_CFG register (configuration 1). */
#define BCC_CONF1_CB3_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB4_CFG register (configuration 1). */
#define BCC_CONF1_CB4_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB5_CFG register (configuration 1). */
#define BCC_CONF1_CB5_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB6_CFG register (configuration 1). */
#define BCC_CONF1_CB6_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB7_CFG register (configuration 1). */
#define BCC_CONF1_CB7_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB8_CFG register (configuration 1). */
#define BCC_CONF1_CB8_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB9_CFG register (configuration 1). */
#define BCC_CONF1_CB9_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB10_CFG register (configuration 1). */
#define BCC_CONF1_CB10_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB11_CFG register (configuration 1). */
#define BCC_CONF1_CB11_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB12_CFG register (configuration 1). */
#define BCC_CONF1_CB12_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB13_CFG register (configuration 1). */
#define BCC_CONF1_CB13_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB14_CFG register (configuration 1). */
#define BCC_CONF1_CB14_CFG_VALUE ( \
  BCC_CB_DISABLED | \
  0xFFU /* Cell balance timer in minutes. */ \
)
  
/* Initial value of CB_OPEN_FLT register (configuration 1). */
#define BCC_CONF1_CB_OPEN_FLT_VALUE 0x0000U

/* Initial value of CB_SHORT_FLT register (configuration 1). */
#define BCC_CONF1_CB_SHORT_FLT_VALUE 0x0000U

/* Initial value of GPIO_CFG1 register (configuration 1). */
#define BCC_CONF1_GPIO_CFG1_VALUE ( \
  BCC_GPIOX_AN_IN_RM_MEAS(6U) | \
  BCC_GPIOX_AN_IN_RM_MEAS(5U) | \
  BCC_GPIOX_AN_IN_RM_MEAS(4U) | \
  BCC_GPIOX_AN_IN_RM_MEAS(3U) | \
  BCC_GPIOX_AN_IN_RM_MEAS(2U) | \
  BCC_GPIOX_AN_IN_RM_MEAS(1U) | \
  BCC_GPIOX_DIG_OUT(0U) \
)

/* Initial value of GPIO_CFG2 register (configuration 1). */
#define BCC_CONF1_GPIO_CFG2_VALUE ( \
  BCC_GPIO2_ADC_TRG_DISABLED | \
  BCC_GPIO0_NO_WAKE_UP | \
  BCC_GPIO0_INP_HIGH_FP_NACT \
  /* Note: GPIOx_DR are initialized to zero (low output level). */ \
)

/* Initial value of GPIO_STS register (configuration 1). */
#define BCC_CONF1_GPIO_STS_VALUE 0x0000U

/* Initial value of AN_OT_UT_FLT register (configuration 1). */
#define BCC_CONF1_AN_OT_UT_FLT_VALUE 0x0000U

/* Initial value of GPIO_SHORT_ANx_OPEN_STS register (configuration 1). */
#define BCC_CONF1_GPIO_SHORT_VALUE 0x0000U

/* Initial value of FAULT3_STATUS register (configuration 1). */
#define BCC_CONF1_FAULT1_STATUS_VALUE 0x0000U

/* Initial value of FAULT3_STATUS register (configuration 1). */
#define BCC_CONF1_FAULT2_STATUS_VALUE 0x0000U

/* Initial value of FAULT3_STATUS register (configuration 1). */
#define BCC_CONF1_FAULT3_STATUS_VALUE 0x0000U

/* Initial value of FAULT_MASK1 register (configuration 1). */
#define BCC_CONF1_FAULT_MASK1_VALUE ( \
  BCC_VPWR_OV_FLT_DIS | \
  BCC_VPWR_LV_FLT_DIS | \
  BCC_COM_LOSS_FLT_DIS | \
  BCC_COM_ERR_FLT_DIS | \
  BCC_CSB_WUP_FLT_DIS | \
  BCC_GPIO0_WUP_FLT_DIS | \
  BCC_I2C_ERR_FLT_DIS | \
  BCC_IS_OL_FLT_DIS | \
  BCC_IS_OC_FLT_DIS | \
  BCC_AN_OT_FLT_EN | \
  BCC_AN_UT_FLT_DIS | \
  BCC_CT_OV_FLT_EN | \
  BCC_CT_UV_FLT_EN \
)

/* Initial value of FAULT_MASK2 register (configuration 1). */
#define BCC_CONF1_FAULT_MASK2_VALUE ( \
  BCC_VCOM_OV_FLT_DIS | \
  BCC_VCOM_UV_FLT_DIS | \
  BCC_VANA_OV_FLT_DIS | \
  BCC_VANA_UV_FLT_DIS | \
  BCC_ADC1_B_FLT_DIS | \
  BCC_ADC1_A_FLT_DIS | \
  BCC_GND_LOSS_FLT_DIS | \
  BCC_AN_OPEN_FLT_DIS | \
  BCC_GPIO_SHORT_FLT_DIS | \
  BCC_CB_SHORT_FLT_DIS | \
  BCC_CB_OPEN_FLT_DIS | \
  BCC_OSC_ERR_FLT_DIS | \
  BCC_DED_ERR_FLT_DIS | \
  BCC_FUSE_ERR_FLT_DIS \
)

/* Initial value of FAULT_MASK3 register (configuration 1). */
#define BCC_CONF1_FAULT_MASK3_VALUE ( \
  BCC_CC_OVR_FLT_DIS | \
  BCC_DIAG_TO_FLT_DIS | \
  /* CBx timeout detection (EOT_CBx bits). */ \
  BCC_EOT_CBX_FLT_DIS(1) |                  /* CB1. */  \
  BCC_EOT_CBX_FLT_DIS(2) |                  /* CB2. */  \
  BCC_EOT_CBX_FLT_DIS(3) |                  /* CB3. */  \
  BCC_EOT_CBX_FLT_DIS(4) |                  /* CB4. */  \
  BCC_EOT_CBX_FLT_DIS(5) |                  /* CB5. */  \
  BCC_EOT_CBX_FLT_DIS(6) |                  /* CB6. */  \
  BCC_EOT_CBX_FLT_DIS(7) |                  /* CB7. */  \
  BCC_EOT_CBX_FLT_DIS(8) |                  /* CB8. */  \
  BCC_EOT_CBX_FLT_DIS(9) |                  /* CB9. */  \
  BCC_EOT_CBX_FLT_DIS(10) |                  /* CB10. */  \
  BCC_EOT_CBX_FLT_DIS(11) |                  /* CB11. */  \
  BCC_EOT_CBX_FLT_DIS(12) |                  /* CB12. */  \
  BCC_EOT_CBX_FLT_DIS(13) |                  /* CB13. */  \
  BCC_EOT_CBX_FLT_DIS(14)                    /* CB14. */ \
)

/* Initial value of WAKEUP_MASK1 register (configuration 1). */
#define BCC_CONF1_WAKEUP_MASK1_VALUE ( \
  BCC_VPWR_OV_WAKEUP_DIS | \
  BCC_VPWR_LV_WAKEUP_DIS | \
  BCC_CSB_WUP_WAKEUP_DIS | \
  BCC_GPIO0_WUP_WAKEUP_DIS | \
  BCC_IS_OC_WAKEUP_DIS | \
  BCC_AN_OT_WAKEUP_DIS | \
  BCC_AN_UT_WAKEUP_DIS | \
  BCC_CT_OV_WAKEUP_DIS | \
  BCC_CT_UV_WAKEUP_DIS \
)

/* Initial value of WAKEUP_MASK2 register (configuration 1). */
#define BCC_CONF1_WAKEUP_MASK2_VALUE ( \
  BCC_VCOM_OV_WAKEUP_DIS | \
  BCC_VCOM_UV_WAKEUP_DIS | \
  BCC_VANA_OV_WAKEUP_DIS | \
  BCC_VANA_UV_WAKEUP_DIS | \
  BCC_ADC1_B_WAKEUP_DIS | \
  BCC_ADC1_A_WAKEUP_DIS | \
  BCC_GND_LOSS_WAKEUP_DIS | \
  BCC_IC_TSD_WAKEUP_DIS | \
  BCC_GPIO_SHORT_WAKEUP_DIS | \
  BCC_CB_SHORT_WAKEUP_DIS | \
  BCC_OSC_ERR_WAKEUP_DIS | \
  BCC_DED_ERR_WAKEUP_DIS \
)

/* Initial value of WAKEUP_MASK3 register (configuration 1). */
#define BCC_CONF1_WAKEUP_MASK3_VALUE ( \
  BCC_CC_OVR_WAKEUP_DIS | \
  /* CBx timeout detection (EOT_CBx bits). */ \
  BCC_EOT_CBX_WAKEUP_DIS(1) |                  /* CB1. */  \
  BCC_EOT_CBX_WAKEUP_DIS(2) |                  /* CB2. */  \
  BCC_EOT_CBX_WAKEUP_DIS(3) |                  /* CB3. */  \
  BCC_EOT_CBX_WAKEUP_DIS(4) |                  /* CB4. */  \
  BCC_EOT_CBX_WAKEUP_DIS(5) |                  /* CB5. */  \
  BCC_EOT_CBX_WAKEUP_DIS(6) |                  /* CB6. */  \
  BCC_EOT_CBX_WAKEUP_DIS(7) |                  /* CB7. */  \
  BCC_EOT_CBX_WAKEUP_DIS(8) |                  /* CB8. */  \
  BCC_EOT_CBX_WAKEUP_DIS(9) |                  /* CB9. */  \
  BCC_EOT_CBX_WAKEUP_DIS(10) |                  /* CB10. */  \
  BCC_EOT_CBX_WAKEUP_DIS(11) |                  /* CB11. */  \
  BCC_EOT_CBX_WAKEUP_DIS(12) |                  /* CB12. */  \
  BCC_EOT_CBX_WAKEUP_DIS(13) |                  /* CB13. */  \
  BCC_EOT_CBX_WAKEUP_DIS(14)                    /* CB14. */ \
)

/* Initial value of TH_ALL_CT register (configuration 1). */
#define BCC_CONF1_TH_ALL_CT_VALUE ( \
  BCC_ALL_CT_OV_TH_DEFAULT  /* CT OV threshold is 1510 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_ALL_CT_UV_TH_DEFAULT  /* CT UV threshold is 1000 mV. It is enabled/disabled through OV_UV_EN register. */ \
)

/* Initial value of TH_CT14 register (configuration 1). */

#define BCC_CONF1_TH_CT14_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)

  
/* Initial value of TH_CT13 register (configuration 1). */
#define BCC_CONF1_TH_CT13_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT12 register (configuration 1). */
#define BCC_CONF1_TH_CT12_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT11 register (configuration 1). */
#define BCC_CONF1_TH_CT11_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT10 register (configuration 1). */
#define BCC_CONF1_TH_CT10_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT9 register (configuration 1). */
#define BCC_CONF1_TH_CT9_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT8 register (configuration 1). */
#define BCC_CONF1_TH_CT8_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT7 register (configuration 1). */
#define BCC_CONF1_TH_CT7_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT6 register (configuration 1). */
#define BCC_CONF1_TH_CT6_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT5 register (configuration 1). */
#define BCC_CONF1_TH_CT5_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT4 register (configuration 1). */
#define BCC_CONF1_TH_CT4_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT3 register (configuration 1). */
#define BCC_CONF1_TH_CT3_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT2 register (configuration 1). */
#define BCC_CONF1_TH_CT2_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_CT1 register (configuration 1). */
#define BCC_CONF1_TH_CT1_VALUE ( \
  BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
  BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)
  
/* Initial value of TH_AN6_OT register (configuration 1). */
#define BCC_CONF1_TH_AN6_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN5_OT register (configuration 1). */
#define BCC_CONF1_TH_AN5_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN4_OT register (configuration 1). */
#define BCC_CONF1_TH_AN4_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN3_OT register (configuration 1). */
#define BCC_CONF1_TH_AN3_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN2_OT register (configuration 1). */
#define BCC_CONF1_TH_AN2_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN1_OT register (configuration 1). */
#define BCC_CONF1_TH_AN1_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(1162U)  /* AN OT threshold is 1162 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN0_OT register (configuration 1). */
#define BCC_CONF1_TH_AN0_OT_VALUE ( \
  BCC_SET_ANX_OT_TH(0U)  /* AN OT threshold is 0 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN6_UV register (configuration 1). */
#define BCC_CONF1_TH_AN6_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN5_UV register (configuration 1). */
#define BCC_CONF1_TH_AN5_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN4_UV register (configuration 1). */
#define BCC_CONF1_TH_AN4_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN3_UV register (configuration 1). */
#define BCC_CONF1_TH_AN3_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN2_UV register (configuration 1). */
#define BCC_CONF1_TH_AN2_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN1_UV register (configuration 1). */
#define BCC_CONF1_TH_AN1_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_AN0_UV register (configuration 1). */
#define BCC_CONF1_TH_AN0_UT_VALUE ( \
  BCC_SET_ANX_UT_TH(3822U)  /* AN UT threshold is 3822 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)
  
/* Initial value of TH_ISENSE_OC register (configuration 1). */
#define BCC_CONF1_TH_ISENSE_OC_VALUE ( \
  /* ISENSE OC threshold is 24576 mA (2458 uV using 100 uOhm resistor). It is enabled/disabled through FAULT_MASK1 and WAKEUP_MASK1 register. */ \
  BCC_SET_TH_ISENSE_OC(2458U) \
)

/* Initial value of TH_COULOMB_CNT_MSB register (configuration 1). */
#define BCC_CONF1_TH_COULOMB_CNT_MSB_VALUE ( \
  BCC_SET_TH_COULOMB_CNT_MSB(0U) /* Higher 16 bits of over Coulomb threshold (2's complement representation). */ \
)

/* Initial value of TH_COULOMB_CNT_LSB register (configuration 1). */
#define BCC_CONF1_TH_COULOMB_CNT_LSB_VALUE ( \
  BCC_SET_TH_COULOMB_CNT_LSB(0U) /* Lower 16 bits of over Coulomb threshold (2's complement representation). */ \
)

/* Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are 
 * not initialized. */

/* Addresses of configurable registers. */
static const uint8_t BCC1_CONF_REG_ADDR[BCC_INIT_REG_CNT] = {
  /* Note: INIT register is initialized automatically (no PEx properties 
   * dependency). */
  /* Note: SYS_CFG_GLOBAL reg. contains only command GO2SLEEP (no 
   * initialization needed). */
  BCC_REG_SYS_CFG1_ADDR,
  BCC_REG_SYS_CFG2_ADDR,
  BCC_REG_SYS_DIAG_ADDR,
  BCC_REG_ADC_CFG_ADDR,
  BCC_REG_ADC2_OFFSET_COMP_ADDR,
/*  BCC_REG_OV_UV_EN_ADDR,
  BCC_REG_CELL_OV_FLT_ADDR,
  BCC_REG_CELL_UV_FLT_ADDR,
  BCC_REG_CB1_CFG_ADDR,
  BCC_REG_CB2_CFG_ADDR,
  BCC_REG_CB3_CFG_ADDR,
  BCC_REG_CB4_CFG_ADDR,
  BCC_REG_CB5_CFG_ADDR,
  BCC_REG_CB6_CFG_ADDR,
  BCC_REG_CB7_CFG_ADDR,
  BCC_REG_CB8_CFG_ADDR,
  BCC_REG_CB9_CFG_ADDR,
  BCC_REG_CB10_CFG_ADDR,
  BCC_REG_CB11_CFG_ADDR,
  BCC_REG_CB12_CFG_ADDR,
  BCC_REG_CB13_CFG_ADDR,
  BCC_REG_CB14_CFG_ADDR,
//  BCC_REG_CB_OPEN_FLT_ADDR,
//  BCC_REG_CB_SHORT_FLT_ADDR, */
  BCC_REG_GPIO_CFG1_ADDR,
  BCC_REG_GPIO_CFG2_ADDR,
//  BCC_REG_GPIO_STS_ADDR,
//  BCC_REG_AN_OT_UT_FLT_ADDR,
//  BCC_REG_GPIO_SHORT_ADDR,
//  BCC_REG_FAULT1_STATUS_ADDR,
//  BCC_REG_FAULT2_STATUS_ADDR,
//  BCC_REG_FAULT3_STATUS_ADDR,
  BCC_REG_FAULT_MASK1_ADDR,
  BCC_REG_FAULT_MASK2_ADDR,
  BCC_REG_FAULT_MASK3_ADDR,
  BCC_REG_WAKEUP_MASK1_ADDR,
  BCC_REG_WAKEUP_MASK2_ADDR,
  BCC_REG_WAKEUP_MASK3_ADDR,
/*  BCC_REG_TH_ALL_CT_ADDR,
  BCC_REG_TH_CT14_ADDR,
  BCC_REG_TH_CT13_ADDR,
  BCC_REG_TH_CT12_ADDR,
  BCC_REG_TH_CT11_ADDR,
  BCC_REG_TH_CT10_ADDR,
  BCC_REG_TH_CT9_ADDR,
  BCC_REG_TH_CT8_ADDR,
  BCC_REG_TH_CT7_ADDR,
  BCC_REG_TH_CT6_ADDR,
  BCC_REG_TH_CT5_ADDR,
  BCC_REG_TH_CT4_ADDR,
  BCC_REG_TH_CT3_ADDR,
  BCC_REG_TH_CT2_ADDR,
  BCC_REG_TH_CT1_ADDR,
  BCC_REG_TH_AN6_OT_ADDR,
  BCC_REG_TH_AN5_OT_ADDR,
  BCC_REG_TH_AN4_OT_ADDR,
  BCC_REG_TH_AN3_OT_ADDR,
  BCC_REG_TH_AN2_OT_ADDR,
  BCC_REG_TH_AN1_OT_ADDR,
  BCC_REG_TH_AN0_OT_ADDR,
  BCC_REG_TH_AN6_UT_ADDR,
  BCC_REG_TH_AN5_UT_ADDR,
  BCC_REG_TH_AN4_UT_ADDR,
  BCC_REG_TH_AN3_UT_ADDR,
  BCC_REG_TH_AN2_UT_ADDR,
  BCC_REG_TH_AN1_UT_ADDR,
  BCC_REG_TH_AN0_UT_ADDR, 
  BCC_REG_TH_ISENSE_OC_ADDR,
  BCC_REG_TH_COULOMB_CNT_MSB_ADDR,
  BCC_REG_TH_COULOMB_CNT_LSB_ADDR,*/
};

static const uint16_t BCC_CONF_OT_ADDR[6] = {
  BCC_REG_TH_AN1_OT_ADDR,
  BCC_REG_TH_AN2_OT_ADDR,
  BCC_REG_TH_AN3_OT_ADDR,
  BCC_REG_TH_AN4_OT_ADDR,
  BCC_REG_TH_AN5_OT_ADDR,
  BCC_REG_TH_AN6_OT_ADDR,
};

static const uint16_t BCC_CONF_UT_ADDR[6] = {
	BCC_REG_TH_AN1_UT_ADDR,
	BCC_REG_TH_AN2_UT_ADDR,
	BCC_REG_TH_AN3_UT_ADDR,
	BCC_REG_TH_AN4_UT_ADDR,
	BCC_REG_TH_AN5_UT_ADDR,
	BCC_REG_TH_AN6_UT_ADDR,
};


/* Initial configuration of Battery Cell Controller devices. */
static const uint16_t BCC1_INIT_CONF[BCC_INIT_REG_CNT] = 
{
    
  BCC_CONF1_SYS_CFG1_VALUE,
  BCC_CONF1_SYS_CFG2_VALUE,
  BCC_CONF1_SYS_DIAG_VALUE,
  BCC_CONF1_ADC_CFG_VALUE,
  BCC_CONF1_ADC2_OFFSET_COMP_VALUE,
/*  BCC_CONF1_OV_UV_EN_VALUE,
  BCC_CONF1_CELL_OV_FLT_VALUE,
  BCC_CONF1_CELL_UV_FLT_VALUE,
  BCC_CONF1_CB1_CFG_VALUE,
  BCC_CONF1_CB2_CFG_VALUE,
  BCC_CONF1_CB3_CFG_VALUE,
  BCC_CONF1_CB4_CFG_VALUE,
  BCC_CONF1_CB5_CFG_VALUE,
  BCC_CONF1_CB6_CFG_VALUE,
  BCC_CONF1_CB7_CFG_VALUE,
  BCC_CONF1_CB8_CFG_VALUE,
  BCC_CONF1_CB9_CFG_VALUE,
  BCC_CONF1_CB10_CFG_VALUE,
  BCC_CONF1_CB11_CFG_VALUE,
  BCC_CONF1_CB12_CFG_VALUE,
  BCC_CONF1_CB13_CFG_VALUE,
  BCC_CONF1_CB14_CFG_VALUE,
  BCC_CONF1_CB_OPEN_FLT_VALUE,
  BCC_CONF1_CB_SHORT_FLT_VALUE,*/
  BCC_CONF1_GPIO_CFG1_VALUE,
  BCC_CONF1_GPIO_CFG2_VALUE,
/*  BCC_CONF1_GPIO_STS_VALUE,
  BCC_CONF1_AN_OT_UT_FLT_VALUE,
  BCC_CONF1_GPIO_SHORT_VALUE,
  BCC_CONF1_FAULT1_STATUS_VALUE,
  BCC_CONF1_FAULT2_STATUS_VALUE,
  BCC_CONF1_FAULT3_STATUS_VALUE,*/
  BCC_CONF1_FAULT_MASK1_VALUE,
  BCC_CONF1_FAULT_MASK2_VALUE,
  BCC_CONF1_FAULT_MASK3_VALUE,
  BCC_CONF1_WAKEUP_MASK1_VALUE,
  BCC_CONF1_WAKEUP_MASK2_VALUE,
  BCC_CONF1_WAKEUP_MASK3_VALUE,
/*  BCC_CONF1_TH_ALL_CT_VALUE,
  BCC_CONF1_TH_CT14_VALUE,
  BCC_CONF1_TH_CT13_VALUE,
  BCC_CONF1_TH_CT12_VALUE,
  BCC_CONF1_TH_CT11_VALUE,
  BCC_CONF1_TH_CT10_VALUE,
  BCC_CONF1_TH_CT9_VALUE,
  BCC_CONF1_TH_CT8_VALUE,
  BCC_CONF1_TH_CT7_VALUE,
  BCC_CONF1_TH_CT6_VALUE,
  BCC_CONF1_TH_CT5_VALUE,
  BCC_CONF1_TH_CT4_VALUE,
  BCC_CONF1_TH_CT3_VALUE,
  BCC_CONF1_TH_CT2_VALUE,
  BCC_CONF1_TH_CT1_VALUE,
  BCC_CONF1_TH_AN6_OT_VALUE,
  BCC_CONF1_TH_AN5_OT_VALUE,
  BCC_CONF1_TH_AN4_OT_VALUE,
  BCC_CONF1_TH_AN3_OT_VALUE,
  BCC_CONF1_TH_AN2_OT_VALUE,
  BCC_CONF1_TH_AN1_OT_VALUE,
  BCC_CONF1_TH_AN0_OT_VALUE,
  BCC_CONF1_TH_AN6_UT_VALUE,
  BCC_CONF1_TH_AN5_UT_VALUE,
  BCC_CONF1_TH_AN4_UT_VALUE,
  BCC_CONF1_TH_AN3_UT_VALUE,
  BCC_CONF1_TH_AN2_UT_VALUE,
  BCC_CONF1_TH_AN1_UT_VALUE,
  BCC_CONF1_TH_AN0_UT_VALUE,
  BCC_CONF1_TH_ISENSE_OC_VALUE,
  BCC_CONF1_TH_COULOMB_CNT_MSB_VALUE,
  BCC_CONF1_TH_COULOMB_CNT_LSB_VALUE*/
};


const float ResTemp[RESTEMP_SIZE] =
{
	377700,346000,325700,306800,289100,272600,257200,242700,229200,216600,																						//* -50~-41 *\/
	204700,193400,182800,172900,163600,154900,146700,139000,131700,124900,    				 //* -40~-31 *\/
	119260,113035.7,107177.5,101661.5,96465.6,91569.1,86953,82599.5,78492,74615.1,          //* -30~-21 *\/
	70954.5,67496.7,64229.3,61140.7,58220,55457,52842.4,50367.2,48023.2,45802.6,               //* -20~-11 *\/
	43698.4,41700.6,39806.5,38009.9,36305.4,34687.7,33151.8,31693.1,30307.4,28990.5,               //* -10~-1  *\/
	27738.6,26546.9,25413.4,24335.1,23308.9,22332.1,21402,20516,19671.9,18867.4,               //*   0~9   *\/
	18100.4,17368.5,16670.4,16004.5,15369,14762.4,14183.3,13630.3,13101.9,12597.1,               //*  10~19  *\/
	12114.5,11653.2,11212.1,10790.1,10386.4,10000,9629.4,9274.7,8935.1,8609.8,                   //*  20~29  *\/
	8298.1,7999.5,7713.3,7493,7175.9,6923.6,6681.5,6449.3,6226.3,6012.3,                         //*  30~39  *\/
	5806.8,5609.4,5419.8,5237.7,5062.6,4894.3,4732.5,4576.9,4427.3,4283.3,                         //*  40~49  *\/
	4144.7,4011.2,3882.7,3758.9,3639.7,3524.9,3414.2,3307.6,3204.9,3105.9,                         //*  50~59  *\/
	3010.4,2918.3,2829.5,2743.8,2661.2,2581.4,2504.4,2430.1,2358.3,2289,                         //*  60~69  *\/
	2222.1,2157.4,2095,2034.6,1976.3,1919.9,1865.4,1812.7,1761.7,1712.4,                         //*  70~79  *\/
	1664.7,1618.5,1573.8,1530.6,1488.8,1448.2,1408.9,1370.7,1333.8,1298,                         //*  80~89  *\/
	1263.3,1229.7,1197.1,1165.5,1135,1105.3,1076.5,1048.6,1021.6,995.3,                          //*  90~99  *\/
	969.9,945.2,921.2,897.9,875.4,853.4,832.2,811.5,791.4,771.9,                                   //* 100~109 *\/
	753,734.6,716.8,699.4,682.5,666.1,650.2,634.7,619.6,605,                                   //* 110~119 *\/
	590.7,576.9,563.4,550.3,537.5                                                        //* 120~125 *\/

};


/*
************************************************************************************************
*  Function:         ResTemp_LookUp()
*  Description:      阻值查表
*  Calls:            无
*  Input:            INT16U value, INT16U *p_tab,INT8U size
*  Output:           无
*  Return:           INT8U
*  Others:
************************************************************************************************
*/
INT8U ResTemp_LookUp(INT32U value, const float *p_tab, INT8U size)
{
	INT8U index = 0;
	INT8U n = 0;
	if (value >= *p_tab)
	{
		index = 0;
	}
	else
	{
		while ((value < *(p_tab + index)) && (n < size - 1))    /* 由阻值查阻值-温度表, 表内偏移-40℃ */
		{
			index++;
			n++;
		}
		if ((value - *(p_tab + index)) > (*(p_tab + index - 1) - value))  /* 四舍五入 */
		{
			index--;
		}
	}
	return index;
}





BCC1_TDeviceData    BCC1_DeviceData;       /* Battery Cell Controller device data structure. */
BCC1_TDeviceDataPtr BCC1_DeviceDataPtr = &BCC1_DeviceData;    /* Pointer to the Battery Cell Controller structure. */

/**
 * This macro returns number of bytes required for receiving RegCnt frames.
 *
 * @param RegCnt Number of register to be read.
 */
#define BCC1_GET_RX_SIZE(RegCnt) \
    (BCC_MSG_SIZE * ((uint16_t)(RegCnt) + 1U))

/**
 * This macro calculates final temperature value.
 *
 * @param TblIdx Index of value in NTC table which is close to the register value provided by user.
 * @param DegTenths Fractional part of temperature value.
 */
#define BCC1_COMP_TEMP(TblIdx, DegTenths) \
  ((((TblIdx) + BCC_NTC_TEMP_MIN) * 10) + (DegTenths))
  
/**
 * This macro returns Memory Data field from a frame.
 *
 * @param MsgPtr Pointer to memory that contains the frame.
 */
#define BCC1_GET_MSG_DATA(MsgPtr) \
  (((uint16_t)*((MsgPtr) + BCC_MSG_IDX_DATA_H) << 8U) | \
    (uint16_t)*((MsgPtr) + BCC_MSG_IDX_DATA_L))

/**
 * This macro converts binary value to Gray code used for Rolling Counter. 
 * It is a 2-bit Gray code. Final value is shifted right by 2 bits.
 *
 * @param BinVal Binary value to be converted.
 */
#define BCC1_GET_RC(BinVal) \
  ((((BinVal) >> 1) ^ (BinVal)) << 2)\                           //do not use

/**
 * This macro increments RCIdx value and executes modulo 4.
 *
 * @param RCIdx Index to be incremented.
 */
#define BCC1_INC_RC_IDX(RCIdx) \
  (((RCIdx) + 1U) & 0x03U)

/**
 * This macro returns a non-zero value when desired cell (CellNo) is enabled
 * in the BCC component. Otherwise returns zero.
 * 
 * @param CellMap Bit map that determines enabled cells 
 * (cell 0 enabled = zero bit is 1).
 * @param CellNo Number of a cell (range is {1,..., 14}).
 */
#define BCC1_IS_CELL_CONN(CellNo) \
  (BCC_CELL_MAP & (1U << ((CellNo) - 1U)))
  
  
  
  
  
 /*
** ===================================================================
**     Method      :  BCC1_WriteRegister (component BCC_MC3377x)
**     @brief
**         This method writes a value to addressed register of selected 
**         Battery Cell Controller device.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         uint8_t RegAddr - Register address. See BCC header file
**         with register map for possible values.
**         @param
**         uint16_t RegVal - New value of selected register.
**         @param
**         uint16_t *RetRegPtr - Automatic response of Battery Cell 
**         Controller, which contains updated register (TPL mode) or a 
**         register addressed in previous access (SPI mode). You can pass NULL 
**         when you do not care about the response.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/

ERR_TYPE BCC1_WriteRegister(uint8_t Cid, uint8_t RegAddr, uint16_t RegVal, uint16_t *RetRegPtr)
{
//  uint8_t          TxBufPtr[BCC_MSG_SIZE]; /* Transmission buffer. */
  uint8_t          *RxBufPtr;              /* Pointer to received data. */
  //  uint8_t          Rc;                     /* Rolling counter value. */
  ERR_TYPE         Error;

  if ((Cid > BCC_MSG_CID_MAX) || (RegAddr > BCC_MAX_REG_ADDR)) 
  {
    return ERR_PARAM_RANGE;
  }
  

  /* Calculate Rolling Counter (RC) and increment RC index. */
  //Rc = BCC1_GET_RC(BCC1_DeviceDataPtr->RcTbl[Cid - 1U]);
  Rc = (BCC1_DeviceDataPtr->RcTbl[Cid - 1U]) << 2;
  BCC1_DeviceDataPtr->RcTbl[Cid - 1U] = BCC1_INC_RC_IDX(BCC1_DeviceDataPtr->RcTbl[Cid - 1U]);

  /* Create frame for writing. */
  BCC1_pack_frame(RegVal, RegAddr, Cid, BCC_CMD_WRITE | Rc, TxBufPtr);
  Error = BCC1_tpl_comm(TxBufPtr, BCC1_DeviceDataPtr->RxBufPtr,
      BCC_MSG_SIZE, 2U * BCC_MSG_SIZE);
  if (Error != ERR_OK) 
  {
      
    return Error;
  }       

  /* Skip an echo frame. */
  RxBufPtr = (uint8_t *)(BCC1_DeviceDataPtr->RxBufPtr + BCC_MSG_SIZE);
  
  Error = BCC1_check_crc(RxBufPtr);

  if (Error != ERR_OK) 
  {
  
    return Error;
  }
  /* Check Rolling Counter value. */
  if ((*(RxBufPtr + BCC_MSG_IDX_CID_CMD) & BCC_MSG_RC_MASK) != Rc) 
  {
  
    return ERR_COM_RC;
  }

  /* Store content of received frame. */
  if (RetRegPtr != NULL) 
  {
    *RetRegPtr = BCC1_GET_MSG_DATA(RxBufPtr);
  }

  return ERR_OK;
}
  




/*
** ===================================================================
**     Method      :  BCC1_WriteRegisterGlobal (component BCC_MC3377x)
**     @brief
**         This method writes a value to addressed register of all configured 
**         Battery Cell Controller devices. This method is available when 
**         communication mode is TPL.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t RegAddr - Register address. See BCC header file
**         with register map for possible values.
**         @param
**         uint16_t RegVal - New value of selected register.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_WriteRegisterGlobal(uint8_t RegAddr, uint16_t RegVal)
{
  uint8_t          TxBufPtr[BCC_MSG_SIZE];     /* Buffer for sending data via TPL. */
  ERR_TYPE       Error;

  /* Check input parameters. */
  if (RegAddr > BCC_MAX_REG_ADDR) {
    return ERR_PARAM_RANGE;
  }
  

  /* Create frame for writing. */
  BCC1_pack_frame(RegVal, RegAddr, BCC1_CID_UNASSIG, BCC_CMD_GLOB_WRITE, 
      TxBufPtr);

  Error = BCC1_tpl_comm(TxBufPtr, BCC1_DeviceDataPtr->RxBufPtr, BCC_MSG_SIZE, 
      BCC_MSG_SIZE);

  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_ReadRegisters (component BCC_MC3377x)
**     @brief
**         This method reads a value from addressed register of selected 
**         Battery Cell Controller device.
**         In case of simultaneous read of more registers, address is 
**         incremented in ascending manner.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         uint8_t RegAddr - Register address. See BCC header file
**         with register map for possible values.
**         @param
**         uint8_t RegCnt  - Number of registers to read. For TPL mode max. 
**         value is defined by "Registers Read Limit" property.
**         @param
**         uint16_t *RegValPtr - Pointer to memory where content of 
**         selected 16 bit register is stored.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/


//uint8_t          *RxBufPtr = NULL; 
ERR_TYPE BCC1_ReadRegisters(uint8_t Cid, uint8_t RegAddr, uint8_t RegCnt, uint16_t* RegValPtr)
{
  //uint8_t          TxBufPtr[BCC_MSG_SIZE]; /* Transmission buffer. */
  uint8_t          *RxBufPtr = NULL;       /* Pointer to received data. */
  uint8_t          RegIdx;                 /* Index of a received register. */
 // uint8_t          Rc;                     /* Rolling counter value. */
  ERR_TYPE       Error;
 // int i;

  /* Check input parameters. */
  if ((Cid > BCC_MSG_CID_MAX) || (RegAddr > BCC_MAX_REG_ADDR) || (RegCnt == 0) ||
      (RegCnt > BCC_RX_LIMIT)) {
    return ERR_PARAM_RANGE;
  }
  
  
  /* Calculate Rolling Counter (RC) value and increment RC index. */
  Rc = (BCC1_DeviceDataPtr->RcTbl[Cid - 1U]) << 2;
  BCC1_DeviceDataPtr->RcTbl[Cid - 1U] = BCC1_INC_RC_IDX(BCC1_DeviceDataPtr->RcTbl[Cid - 1U]);

  /* Create frame for request. */
  BCC1_pack_frame((uint16_t)RegCnt, RegAddr, Cid, BCC_CMD_READ | Rc, TxBufPtr);
  Error = BCC1_tpl_comm(TxBufPtr, BCC1_DeviceDataPtr->RxBufPtr,
      BCC_MSG_SIZE, BCC1_GET_RX_SIZE(RegCnt));
  if (Error != ERR_OK) {

    return Error;
  }

  /* Check and store responses. */
  for (RegIdx = 0U; RegIdx < RegCnt; RegIdx++) {
    /* Pointer to beginning of a frame. */
    RxBufPtr = (uint8_t *)(BCC1_DeviceDataPtr->RxBufPtr + ((1U + RegIdx) * BCC_MSG_SIZE));
 /*   for (i = 0; i < 5; i++)
    {
    rxbuf_1[i] = *(RxBufPtr+i);
    rxbuf_2[i] = BCC1_DeviceDataPtr->RxBufPtr[(1 + RegIdx)*BCC_MSG_SIZE+i];
    }        */
    Error = BCC1_check_crc(RxBufPtr);
    if (Error != ERR_OK) {
      return Error;
    }
   tagidcal = (RegVal2 & 0xF000) >> 12;
    Error = BCC1_check_rc_tagid(RxBufPtr, Rc, (uint8_t)tagidcal);
    if (Error != ERR_OK) {
      return Error;
    }

    /* Store data. */
    *(RegValPtr + RegIdx) = BCC1_GET_MSG_DATA(RxBufPtr);
  }

  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  BCC1_UpdateRegister (component BCC_MC3377x)
**     @brief
**         This method updates content of a selected register. It affects bits 
**         specified by a bit mask.
**         This method is required (generated) when you enable a diagnostic 
**         method, PauseCBDrivers method or heartbeat function in daisy 
**         chain configuration.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         uint8_t RegAddr - Register address. See BCC header file
**         with register map for possible values.
**         @param
**         uint16_t RegMask - Bit mask. Bits set to 1 will be updated.
**         @param
**         uint16_t RegVal - New value of register bits defined by bit 
**         mask.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_UpdateRegister(uint8_t Cid, uint8_t RegAddr, uint16_t RegMask, uint16_t RegVal)
{
  uint16_t         RegValTemp;
  ERR_TYPE       Error;

  Error = BCC1_ReadRegisters(Cid, RegAddr, 1U, &RegValTemp);
  if (Error != ERR_OK) {
    return Error;
  }

  /* Update register value. */
  RegValTemp = BCC_REG_UNSET_BIT_VALUE(RegValTemp, RegMask);
  RegValTemp = BCC_REG_SET_BIT_VALUE(RegValTemp, (RegVal & RegMask));

  return BCC1_WriteRegister(Cid, RegAddr, RegValTemp, NULL);
}

/*
** ===================================================================
**     Method      :  BCC1_Sleep (component BCC_MC3377x)
**     @brief
**         Sets sleep mode of all Battery Cell Controller devices. In case of 
**         TPL mode MC33664TL goes to sleep mode automatically.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_Sleep(void)
{
  ERR_TYPE Error;

  Error = BCC1_WriteRegisterGlobal(BCC_REG_SYS_CFG_GLOBAL_ADDR, BCC_GO2SLEEP_ENABLED);
  /* Set MC33664 sleep mode. */
 // EN = LOW;
	
  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_WakeUp (component BCC_MC3377x)
**     @brief
**         Sets normal mode of all Battery Cell Controller devices. In case of 
**         TPL mode MC33664TL goes to normal mode automatically.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_WakeUp(void)
{
  ERR_TYPE Error;  
#if 0
  /* Enable TPL device. */
  Error = BCC1_enable_TPL();
  if (Error != ERR_OK) {
    return Error;
  }
#endif  
/* Two consecutive transitions of CSB_TX from low to high. */
  /* CSB_TX low. */
  SPI_3_PUSHR |= 0x00030000; 
  //SPIT_SS = LOW;
  /* Wait for t1; 20 us < t1. */
  delay_10us(2);

  /* CSB_TX high. */
  SPI_3_PUSHR &= 0xFFFCFFFF; 
  //SPIT_SS = HIGH;
  /* Wait for t2; 500 us < t2. */
  delay_10us(60);

 //SPIT_SS = LOW;
 SPI_3_PUSHR |= 0x00030000; 
  /* Wait for t1; 20 us < t1. */
  delay_10us(2);

  /* CSB_TX high. */
  //SPIT_SS = HIGH;
  SPI_3_PUSHR &= 0xFFFCFFFF; 
  /* Wait for t2; 500 us < t2. */
  delay_10us(5);

  return ERR_OK;
}

/*
** ===================================================================
**     Method      :  BCC1_StartConversion (component BCC_MC3377x)
**     @brief
**         Starts ADC conversion. It sets Start of Conversion bit and new 
**         value of TAG ID in ADC_CFG register.
**         TAG ID is incremented for each conversion. You can use method 
**         IsConverting to check conversion status.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/

ERR_TYPE BCC1_StartConversion(uint8_t Cid)
{
  //uint16_t         RegVal;     /* Value of ADC_CFG register. */
  ERR_TYPE       Error;

  if (Cid > BCC_MSG_CID_MAX) {
    return ERR_PARAM_RANGE;
  }

  

if (Cid == BCC1_CID_UNASSIG) {
    /* Global reset command. */
    RegVal2 = BCC_CONF1_ADC_CFG_VALUE|BCC_INIT_CONV_SEQ;
    Error = BCC1_WriteRegisterGlobal(BCC_REG_ADC_CFG_ADDR, RegVal2);
    //   Error = BCC1_WriteRegisterGlobal(BCC_REG_ADC_CFG_ADDR, 0x0C3F);
  } else
  {
    /* Increment TAG ID (4 bit value). */
  TagID = BCC_SET_TAG_ID(TagID,BCC1_DeviceDataPtr->TagId[Cid - 1]); 
 //TagID = 0;
  BCC1_DeviceDataPtr->TagId[Cid - 1] = (BCC1_DeviceDataPtr->TagId[Cid - 1] + 1U) & 0x0FU;

  /* Set new TAG ID and Start of Conversion bit. */
  RegVal2 = TagID|BCC_CONF1_ADC_CFG_VALUE|BCC_INIT_CONV_SEQ;
    Error = BCC1_WriteRegister(Cid, BCC_REG_ADC_CFG_ADDR, RegVal2, NULL);
   //  Error = BCC1_WriteRegister(Cid, BCC_REG_ADC_CFG_ADDR, 0x0C3F, NULL);
  }

  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_IsConverting (component BCC_MC3377x)
**     @brief
**         Checks status of conversion defined by End of Conversion bit in 
**         ADC_CFG register.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         bool *CompletedPtr - Pointer to check result. Set to TRUE when  
**         a conversion is complete.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_IsConverting(uint8_t Cid, BOOLEAN *CompletedPtr)
{
  uint16_t         RegVal;     /* Value of ADC_CFG register. */
  ERR_TYPE       Error;

  if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
    return ERR_PARAM_RANGE;
  }


  Error = BCC1_ReadRegisters(Cid, BCC_REG_ADC_CFG_ADDR, 1U, &RegVal);

  RegVal = RegVal & BCC_R_EOC_N_MASK;
  *(CompletedPtr) = (RegVal == 0x00U);

  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_GetRawMeasurements (component BCC_MC3377x)
**     @brief
**         Reads the measurement registers and returns raw values. You can use 
**         macros defined in BCC header file to perform correct unit conversion.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         uint16_t RawMeas[] - Array containing all values measured 
**         by Battery Cell Controller. Indexes into the array are defined in 
**         enumeration TMeasurement placed in BCC header file. Required size of 
**         the array depends on Battery Cell Controller model (see 
**         "component_name"_MEAS_CNT constant defined in BCC header file): 30 
**         items for MC33771.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_GetRawMeasurements(uint8_t Cid, uint16_t RawMeas[])
{
  ERR_TYPE       Error;
  uint8_t          i;

  if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
    return ERR_PARAM_RANGE;
  }


  /* Read all the measurement registers.
   * Note: the order and number of registers conforms to the order of measured
   * values in Measurements array, see enumeration TMeasurement.*/
  Error = BCC1_ReadRegisters(Cid, BCC_REG_CC_NB_SAMPLES_ADDR, BCC1_MEAS_CNT, RawMeas);
  /* Mask bits. */
  /* Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2 registers. */
  RawMeas[msrISENSE1] &= BCC_R_MEAS1_I_MASK;
  RawMeas[msrISENSE2] &= BCC_R_MEAS2_I_MASK;
  
  /* Mask the other registers (starting at 5th register). */
  for (i = 5U; i < BCC1_MEAS_CNT; i++) {
    RawMeas[i] &= BCC_R_MEAS_MASK;
  }

  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_GetFaultStatus (component BCC_MC3377x)
**     @brief
**         Reads the status registers and returns raw values. You can use
**         constants defined in "component_name"_MC33771.h file.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         uint16_t StatusPtr[] - Array containing all fault status 
**         information provided by Battery Cell Controller.
**         Indexes into the array are defined in TFaultStatus enumeration
**         placed in BCC header file. Required size of the array is 11. You 
**         can use macro BCCx_STAT_CNT defined in BCC header file, which
**         contains appropriate value.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_GetFaultStatus(uint8_t Cid, uint16_t StatusPtr[])
{
  ERR_TYPE Error = ERR_OK;

  if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
    return ERR_PARAM_RANGE;
  }

  /* Read CELL_OV_FLT and CELL_UV_FLT. */
  Error |= BCC1_ReadRegisters(Cid, BCC_REG_CELL_OV_FLT_ADDR, 2U, &StatusPtr[stCELLOV]);

  /* Read CB_OPEN_FLT, CB_SHORT_FLT. */
  Error |= BCC1_ReadRegisters(Cid, BCC_REG_CB_OPEN_FLT_ADDR, 2U, &StatusPtr[stCBOPEN]);

  /* Read GPIO_STS, AN_OT_UT_FLT, GPIO_SHORT_Anx_OPEN_STS. */
  Error |= BCC1_ReadRegisters(Cid, BCC_REG_GPIO_STS_ADDR, 3U, &StatusPtr[stGPIOSTATUS]);
  
  /* Read COM_STATUS, FAULT1_STATUS, FAULT2_STATUS and FAULT3_STATUS. */
  Error |= BCC1_ReadRegisters(Cid, BCC_REG_COM_STATUS_ADDR, 4U, &StatusPtr[stCOMM]);
  
  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_GetFaultPinVal (component BCC_MC3377x)
**     @brief
**         This method returns value of FAULT pin.
**         When the pin is at HIGH then one or more of used BCC devices 
**         is/are in fault state.
**         You can use methods GetFaultStatus and ClearFaultStatus for recovery.
**         Note that this method is available only when "Fault Pin Settings"
**         group is enabled in component properties.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @return
**         bool            - Level of FAULT pin (TRUE - high, FALSE - LOW).
** ===================================================================
*/
BOOLEAN BCC1_GetFaultPinVal(void)
{
	return 1;
	  //FAULT_BCC;
}

/*
** ===================================================================
**     Method      :  BCC1_ClearFaultStatus (component BCC_MC3377x)
**     @brief
**         This method clears selected fault status register.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         TFaultStatus FaultSel - Selection of a fault status register to 
**         be cleared. See definition of this enumeration in BCC header file.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_ClearFaultStatus(uint8_t Cid, BCC_TFaultStatus StatSel)
{

   ERR_TYPE  Error;
  /* This array is intended for conversion of BCC_TFaultStatus value to
   * a BCC register address. */
  const uint8_t REG_ADDR_MAP[BCC1_STAT_CNT] = {
      BCC_REG_CELL_OV_FLT_ADDR, BCC_REG_CELL_UV_FLT_ADDR,
      BCC_REG_CB_OPEN_FLT_ADDR, BCC_REG_CB_SHORT_FLT_ADDR,
      BCC_REG_GPIO_STS_ADDR, BCC_REG_AN_OT_UT_FLT_ADDR,
      BCC_REG_GPIO_SHORT_ADDR, BCC_REG_COM_STATUS_ADDR,
      BCC_REG_FAULT1_STATUS_ADDR, BCC_REG_FAULT2_STATUS_ADDR,
      BCC_REG_FAULT3_STATUS_ADDR
  };

  if (Cid > BCC_MSG_CID_MAX) {
    return ERR_PARAM_RANGE;
  }
  /* Note: COM_STATUS register is read only. */
  if ((StatSel < 0U) || (StatSel >= BCC1_STAT_CNT) || (StatSel == stCOMM)) {
    return ERR_PARAM_RANGE;
  }
  if (Cid == BCC1_CID_UNASSIG) {
	  /* Global reset command. */
	  Error = BCC1_WriteRegisterGlobal(REG_ADDR_MAP[StatSel], 0x00U);
	 
  } else
  {
	  Error = BCC1_WriteRegister(Cid, REG_ADDR_MAP[StatSel], 0x00U, NULL);
  } 
  return Error;
}

void BCC1_ClearAllFault(void)
{
	uint8_t i;
	/* Clear Registers Faults  */
	for(i = 0; i < BCC1_STAT_CNT; i++) 
	{
		(void)BCC1_ClearFaultStatus(0, i);
	} 
}

/*
** ===================================================================
**     Method      :  BCC1_SoftwareReset (component BCC_MC3377x)
**     @brief
**         Resets Battery Cell Controller device using software reset. It 
**         enters reset via SPI or TPL interface.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_SoftwareReset(uint8_t Cid)
{
  ERR_TYPE Error;

  /* Note: it is not necessary to read content of SYS_CFG1 register
   * to change only RST bit, because registers are set to default values. */
  if (Cid > BCC_MSG_CID_MAX) {
    return ERR_PARAM_RANGE;
  }
  else if (Cid == BCC1_CID_UNASSIG) {
    /* Global reset command. */
   // Error = BCC1_WriteRegisterGlobal(BCC_REG_SYS_CFG1_ADDR, BCC_W_SOFT_RST_MASK);
    Error = BCC1_WriteRegisterGlobal(BCC_REG_SYS_CFG1_ADDR, 0x9011);
  }
  else {
   // Error = BCC1_WriteRegister(Cid, BCC_REG_SYS_CFG1_ADDR,BCC_W_SOFT_RST_MASK, NULL);
      Error = BCC1_WriteRegister(Cid, BCC_REG_SYS_CFG1_ADDR,0x9011, NULL);
    if (ERR_COM_TIMEOUT == Error) { /* Device does not respond after reset - normal condition. */
      Error = ERR_OK;
    }
  }

  return Error;
}

/*
** ===================================================================
**     Method      :  BCC1_SetCBDrivers (component BCC_MC3377x)
**     @brief
**         Sets state of cell balancing drivers. It is designated to control 
**         all the drivers at once. It can be used to reset CB timer of all CB 
**         drivers.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         bool Enable     - Drivers state. Possible values are FALSE (all 
**         drivers are disabled) and TRUE (drivers are enabled).
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_SetCBChannal(uint8_t Cid, uint8_t Cbch[])
{
	uint16_t         RegVal;   /* Value of a register. */
	uint8_t          i;
	ERR_TYPE       Error = ERR_OK;

	if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
		return ERR_PARAM_RANGE;
	}
	
		for (i = 0U; i < BCC_MAX_CELLS; i++) {
		    RegVal = 0;
		//	if(Cbch[i]!= BalSet_Back[Cid-1][i])
			if(1)
			{
				if(Cbch[i] == 1)
				{
					RegVal = BCC_REG_SET_BIT_VALUE(RegVal, BCC_W_CB_EN_MASK);
					Error |= BCC1_WriteRegister(Cid, BCC_REG_CB1_CFG_ADDR + i, RegVal, NULL);
				}else
				{
					RegVal = BCC_REG_UNSET_BIT_VALUE(RegVal, BCC_W_CB_EN_MASK);
					Error  |= BCC1_WriteRegister(Cid, BCC_REG_CB1_CFG_ADDR + i, RegVal, NULL);
				}
			}
			BalSet_Back[Cid-1][i] = Cbch[i];	
		}
	return Error;
}

ERR_TYPE BCC1_SetCBDrivers(uint8_t Cid, BOOLEAN Enable, uint8_t Cbch[])
{
	uint16_t         RegVal;   /* Value of a register. */
	uint8_t          i;
	ERR_TYPE       Error = ERR_OK;

	if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
		return ERR_PARAM_RANGE;
	}



	if (Enable == TRUE) {
		/* Enable all CB driver with use of SYS_CFG1[CB_DRVEN]. It is necessary to
		 * restart each driver by CBx_CFG[CB_EN] bit.
		 * Note: CB timer (CBx_CFG[CB_TIMER]) restarts when a CB driver is enabled. */
			RegVal = BCC_CYCLIC_TIMER_DISABLED | 
			    BCC_DIAG_TIMEOUT_8S | 
			    BCC_I_MEAS_DISABLED | 
			    BCC_CB_AUTO_PAUSE_ENABLED | 
			    BCC_CB_DRV_ENABLED | 
			    BCC_DIAG_MODE_DISABLED | 
			    BCC_CB_MAN_PAUSE_DISABLED | 
			    BCC_SW_RESET_DISABLED | 
			    BCC_FAULT_WAVE_DISABLED | 
			    BCC_WAVE_DC_500US | 
			    BCC_OSC_MON_ENABLED;
		Error = BCC1_WriteRegister(Cid, BCC_REG_SYS_CFG1_ADDR, RegVal, NULL);

		for (i = 0U; i < BCC_MAX_CELLS; i++) {
		    RegVal = 0;
		    if(Cbch[i] == 1)
		    {
			RegVal = BCC_REG_SET_BIT_VALUE(RegVal, BCC_W_CB_EN_MASK);
			Error |= BCC1_WriteRegister(Cid, BCC_REG_CB1_CFG_ADDR + i, RegVal, NULL);
		    }else
		    {
			RegVal = BCC_REG_UNSET_BIT_VALUE(RegVal, BCC_W_CB_EN_MASK);
			Error  = BCC1_WriteRegister(Cid, BCC_REG_CB1_CFG_ADDR + i, RegVal, NULL);
		    }
				
		}
	}
	else {
		/* Disable all CB drivers with use of SYS_CFG1[CB_DRVEN]. */
		RegVal = BCC_REG_UNSET_BIT_VALUE(RegVal, BCC_RW_CB_DRVEN_MASK);
		Error = BCC1_WriteRegister(Cid, BCC_REG_SYS_CFG1_ADDR, RegVal, NULL);
	}

	return Error;
}

ERR_TYPE BCC1_EnabeCBDrivers(void)
{
	uint16_t RegVal;
	ERR_TYPE Error;
	
	RegVal = BCC_CYCLIC_TIMER_DISABLED | 
			 BCC_DIAG_TIMEOUT_8S | 
			 BCC_I_MEAS_DISABLED | 
			 BCC_CB_AUTO_PAUSE_ENABLED | 
			 BCC_CB_DRV_ENABLED | 
			 BCC_DIAG_MODE_DISABLED | 
			 BCC_CB_MAN_PAUSE_DISABLED | 
			 BCC_SW_RESET_DISABLED | 
			 BCC_FAULT_WAVE_DISABLED | 
			 BCC_WAVE_DC_500US | 
			 BCC_OSC_MON_ENABLED;;
	return BCC1_WriteRegisterGlobal(BCC_REG_SYS_CFG1_ADDR,RegVal);
}

ERR_TYPE BCC1_DisabeAllCBDrivers(void)
{
	uint16_t RegVal;
	ERR_TYPE Error;
	
	RegVal = BCC_CYCLIC_TIMER_DISABLED | 
			    BCC_DIAG_TIMEOUT_8S | 
			    BCC_I_MEAS_DISABLED | 
			    BCC_CB_AUTO_PAUSE_ENABLED | 
			    BCC_CB_DRV_DISABLED | 
			    BCC_DIAG_MODE_DISABLED | 
			    BCC_CB_MAN_PAUSE_DISABLED | 
			    BCC_SW_RESET_DISABLED | 
			    BCC_FAULT_WAVE_DISABLED | 
			    BCC_WAVE_DC_500US | 
			    BCC_OSC_MON_ENABLED;;
	return BCC1_WriteRegisterGlobal(BCC_REG_SYS_CFG1_ADDR,RegVal);
}

/*
** ===================================================================
**     Method      :  BCC1_PauseCBDrivers (component BCC_MC3377x)
**     @brief
**         This method can be used to manual pause cell balancing before on 
**         demand conversion.
**         As a result more precise measurement can be done. Note that it is 
**         user obligation to re-enable cell balancing after measurement ends.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @param
**         bool Pause      - TRUE (pause) / FALSE (unpause).
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_PauseCBDrivers(uint8_t Cid, BOOLEAN Pause)
{
  uint16_t RegVal = (Pause) ? BCC_CB_MAN_PAUSE_ENABLED : 
      BCC_CB_MAN_PAUSE_DISABLED;

  if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
    return ERR_PARAM_RANGE;
  }
      
  return BCC1_UpdateRegister(Cid, BCC_REG_SYS_CFG1_ADDR,
      BCC_RW_CB_MANUAL_PAUSE_MASK, RegVal);
}

/*
** ===================================================================
**     Method      :  BCC1_VerifyCom (component BCC_MC3377x)
**     @brief
**         This method uses No Operation command of BCC to verify 
**         communication without performing any operation.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster Identification Address. You can use 
**         constants defined in BCC header file.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/
ERR_TYPE BCC1_VerifyCom(uint8_t Cid)
{
  uint8_t          TxBufPtr[BCC_MSG_SIZE]; /* Transmission buffer. */
  uint8_t          *RxBufPtr;              /* Pointer to received data. */
 // uint8_t          Rc;                     /* Rolling counter value. */
  ERR_TYPE       Error;

  if ((Cid == BCC1_CID_UNASSIG) || (Cid > BCC_MSG_CID_MAX)) {
    return ERR_PARAM_RANGE;
  }

  /* Calculate Rolling Counter (RC) and increment RC index. */
  Rc = BCC1_DeviceDataPtr->RcTbl[Cid - 1U];
  BCC1_DeviceDataPtr->RcTbl[Cid - 1U] = BCC1_INC_RC_IDX(BCC1_DeviceDataPtr->RcTbl[Cid - 1U]);

  /* Create frame for writing.
   * Note: Memory Data and Memory Address fields can contain any value. */
  BCC1_pack_frame(0x00U, 0x00U, Cid, BCC_CMD_NOOP | Rc, TxBufPtr);
  Error = BCC1_tpl_comm(TxBufPtr, BCC1_DeviceDataPtr->RxBufPtr,
      BCC_MSG_SIZE, 2U * BCC_MSG_SIZE);
  if (Error != ERR_OK) {

    return Error;
  }

  /* Skip an echo frame. */
  RxBufPtr = (uint8_t *)(BCC1_DeviceDataPtr->RxBufPtr + BCC_MSG_SIZE);
  Error = BCC1_check_crc(RxBufPtr);
  if (Error != ERR_OK) {
    return Error;
  }
  /* Check Rolling Counter value. */
  if ((*(RxBufPtr + BCC_MSG_IDX_CID_CMD) & BCC_MSG_RC_MASK) != Rc) {

    return ERR_COM_RC;
  }


  return ERR_OK;
}  


/*
** ===================================================================
**     Method      :  BCC1_enable_TPL (component BCC_MC3377x)
**     @brief
**         This function enables MC33664 TPL device. It uses EN and
**         INTB pins.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @return
**         BCC_TError      - Error code, possible codes:
**                           ERR_OK - OK.
**                           ERR_COM_TIMEOUT - Timeout expired while waiting for
**                           a level of INTB pin.
** ===================================================================
*/
 ERR_TYPE BCC1_enable_TPL(void)
{
  uint16_t         Timeout = BCC_WAKEUP_TIMEOUT;    /* Wake-up timeout. */
#if 0 
  /* Set normal state (transition from low to high). */
  EN = LOW;
  /* Wait at least 100 us. */
  Delay10Us(15);
  EN = HIGH;

  /* Note: MC33664 has time tReady (max. 100 us, equal to the LOW level) to take effect.  */
  while ((INTB == HIGH) && 
      (Timeout > 0)) {
    /* Wait for INTB transition from high to low (max. 100 us). */
    /* Timeout. */
    Timeout--;
  }
  if(Timeout == 0)
  {
    return ERR_COM_TIMEOUT;
  }
  
  while ((INTB == LOW) &&
      (Timeout > 0)) {
    /* Wait for INTB transition from low to high (typ. 100 us). */
    /* Timeout. */
    Timeout--;
  }

  /* Now the device should be in normal mode (i.e. after INTB low to high
   * transition). For sure wait for 150 us. */
  Delay10Us(15);
#endif
 // return (Timeout == 0) ? ERR_COM_TIMEOUT : ERR_OK;
  return (Timeout == 0) ? ERR_COM_TIMEOUT : ERR_OK;
}

/*
** ===================================================================
**     Method      :  BCC1_init_regs (component BCC_MC3377x)
**     @brief
**         This function initializes a BCC device or all devices in
**         daisy chain.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster ID of BCC device.
**         @param
**         uint16_t *DevConf - Initialization values of BCC registers.
**         @return
**         BCC_TError      - Error code, possible codes:
**                           ERR_OK - OK.
**                           Other codes are defined by WriteRegister.
** ===================================================================
*/

 /* Initialize CFG registers  */
ERR_TYPE BCC1_init_cfgs()
{
	uint8_t          i;
	ERR_TYPE       Error = ERR_OK;    
	
	/* Initialize CFG registers  */
	for (i = 0; i < BCC_INIT_REG_CNT; i++) {
		Error |= BCC1_WriteRegisterGlobal(BCC1_CONF_REG_ADDR[i], BCC1_INIT_CONF[i]);
    }
	return Error;
}

 ERR_TYPE BCC1_init_regs(uint8_t Cid, const uint16_t DevConf[])
{
  uint8_t          i;
  ERR_TYPE       Error = ERR_OK;
 // uint16_t  TH_data[28];        

  (void)Cid;  /* Parameter is not used, suppress unused argument warning */
  
  /* Initialize CFG registers  */
  for (i = 0; i < BCC_INIT_REG_CNT; i++) {
    Error |= BCC1_WriteRegisterGlobal(BCC1_CONF_REG_ADDR[i], DevConf[i]);
    }
	
	/* Initialize OT UT OV UV registers  */
  for(i = 0; i < BCC1_DevCnt; i++) 
  {
	  Error |= BCC1_init_cellovuv(i+1, NVM_CellUv, NVM_CellOv, NVM_ChSelect[i]);
	  Error |= BCC1_init_cellotut(i+1, &NVM_CellUt[i][0], &NVM_CellOt[i][0]);
  }  

  
  /* Clear Registers Faults  */
  BCC1_ClearAllFault(); 
   return Error;  
}

 ERR_TYPE BCC1_init_cellovuv(uint8_t Cid, uint16_t CellUv, uint16_t CellOv, uint16_t ChSelect)
{
	uint16_t RegVal = 0U;
	uint16_t RegValBack = 0U;
	ERR_TYPE Error;
	
	RegVal = BCC_CTX_OV_TH_COMMON | BCC_CTX_UV_TH_COMMON | ChSelect; 			 
	Error = BCC1_WriteRegister(Cid, BCC_REG_OV_UV_EN_ADDR, RegVal, &RegValBack);
	if(RegVal != RegValBack)
	{
		Error |= ERR_WrongRes;
	}
	RegVal = BCC_SET_ALL_CT_UV_TH(CellUv) | BCC_SET_ALL_CT_OV_TH(CellOv);
	Error |= BCC1_WriteRegister(Cid, BCC_REG_TH_ALL_CT_ADDR, RegVal, &RegValBack);
	
	if(RegVal != RegValBack)
	{
		Error |= ERR_WrongRes;
	}	
	return Error;	
}

 ERR_TYPE BCC1_init_cellotut(uint8_t Cid, int16_t *CellUt, int16_t *CellOt)
{
	uint16_t RegVal = 0U;
	uint16_t RegValBack = 0U;
	ERR_TYPE Error;
	int i;
	for(i = 0; i < 6; i++)
	{
		RegVal = VCOM * (float)ResTemp[CellUt[i]- BCC_NTC_TEMP_MIN]/(ResTemp[CellUt[i] - BCC_NTC_TEMP_MIN] + RTEMP_PU);
		RegVal = BCC_SET_ANX_UT_TH(RegVal);
		Error |= BCC1_WriteRegister(Cid, BCC_CONF_UT_ADDR[i], RegVal, &RegValBack);
		if(RegVal != RegValBack)
		{
			Error |= ERR_WrongRes;
		}
		RegVal = VCOM * (float)ResTemp[CellOt[i]- BCC_NTC_TEMP_MIN]/(ResTemp[CellOt[i] - BCC_NTC_TEMP_MIN] + RTEMP_PU);
		RegVal = BCC_SET_ANX_OT_TH(RegVal);
		Error |= BCC1_WriteRegister(Cid, BCC_CONF_OT_ADDR[i], RegVal, &RegValBack);
		if(RegVal != RegValBack)
		{
			Error |= ERR_WrongRes;
		}	
	}

	return Error;	
}

/*
** ===================================================================
**     Method      :  BCC1_assign_cid (component BCC_MC3377x)
**     @brief
**         This function assigns CID to a BCC device that has CID equal
**         to zero. It closes bus switch to allow communication with
**         the next BCC.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         uint8_t Cid     - Cluster ID of BCC device.
**         @return
**         BCC_TError      - Error code, possible codes:
**                           ERR_OK - OK.
**                           Other codes are defined by ReadRegisters
**                           and WriteRegister.
** ===================================================================
*/
ERR_TYPE BCC1_assign_cid(uint8_t Cid)
{
  uint16_t         RegVal = 0U;   /* Value of a register. */
  ERR_TYPE       Error;        
  
  /* Check if unassigned node replies. This is the first reading after device 
   * reset. */
 // Error = BCC1_ReadRegisters(BCC1_CID_UNASSIG, BCC_REG_INIT_ADDR, 1U, &RegVal);
  /* Note: in SPI com. mode the device responds with all zero and the correct
   * CRC (null response) during the very first message. */
 // if ((ERR_OK != Error) && (ERR_NULL_RESP != Error)) {
 //   return Error;
//  }

  /* Assign CID and close the bus switch to be able to initialize next BCC device. */
  RegVal = BCC_SET_CID(RegVal, Cid) | BCC_BUS_SWITCH_ENABLED;
  Error = BCC1_WriteRegister(BCC1_CID_UNASSIG, BCC_REG_INIT_ADDR, RegVal, NULL);
  if (ERR_OK != Error) {
    return Error;
  }
  
  /* Check if assigned node replies. */
//  return BCC1_ReadRegisters(Cid, BCC_REG_INIT_ADDR, 1U, &RegVal);
return ERR_OK;
}

/*
** ===================================================================
**     Method      :  BCC1_init_devs (component BCC_MC3377x)
**     @brief
**         This function initializes all BCC devices enabled in component
**         properties.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @return
**         BCC_TError      - Error code, possible codes:
**                           ERR_OK - OK.
**                           Other codes are defined by assign_cid,
**                           init_regs and ReadRegisters.
** ===================================================================
*/
 ERR_TYPE BCC1_init_devs()
{
  uint8_t          i;
  uint16_t         RegVal; /* Value of a register. */
  ERR_TYPE       Error;
  
  /* Assign CID and close bus switch of all configured BCC devices. */
  for (i = 0U; i < BCC1_DevCnt; i++) {
  Error = BCC1_assign_cid(BCC1_CID_DEV1 + i);
    if (Error != ERR_OK) {
    //  return Error;    NEED TO CONFIRM
    }
  }
  
  /* Initialize registers of device(s). */
  Error = BCC1_init_regs(BCC1_CID_UNASSIG, BCC1_INIT_CONF);
  if (Error != ERR_OK) {
    return Error;
  }
  
  /* Check communication of all configured BCC devices. */
/*  for (i = 0; i < BCC1_DevCnt; i++) {
    Error = BCC1_ReadRegisters(BCC1_CID_DEV1 + i, BCC_REG_INIT_ADDR, 1U, &RegVal);
    if (Error != ERR_OK) {
      return Error;
    }
  }
*/
  return ERR_OK;
}


/*
** ===================================================================
**     Method      :  BCC1_Init (component BCC_MC3377x)
**     @brief
**         Initializes the Battery Cell Controller device or devices (depends 
**         on configuration). 
**         It assigns CID, initializes communication interface according to 
**         selected mode (classic SPI or TPL) and configures the device with 
**         predefined values from Processor Expert properties.
**     Parameters  :
**         NAME            - DESCRIPTION
**         @param
**         LDD_TUserData *UserDataPtr - Pointer to the user data. This 
**         pointer will be passed as an event or callback parameter.
**         @return
**         BCC_TError      - If success returns ERR_OK else returns common 
**         errors from PE_Error.h or specific errors from BCC header files.
** ===================================================================
*/

void BCC1_DataInit()
{
	uint8_t BCC1_CID_DEV; 
	
	BCC1_DeviceDataPtr = &BCC1_DeviceData;
	/* Initialize TAG ID. */
	for(BCC1_CID_DEV = 0; BCC1_CID_DEV < BCC1_DevCnt; BCC1_CID_DEV++) 
	{     
		BCC1_DeviceDataPtr->RcTbl[BCC1_CID_DEV] = 0U;
		BCC1_DeviceDataPtr->TagId[BCC1_CID_DEV] = 0U;
	}
	//(void)BCC1_SoftwareReset(BCC1_CID_UNASSIG);
}

ERR_TYPE BCC1_Init()
{
  ERR_TYPE Error;
  uint8_t BCC1_CID_DEV; 
  
  BCC1_DeviceDataPtr = &BCC1_DeviceData;
  /* Initialize TAG ID. */
  for(BCC1_CID_DEV = 0; BCC1_CID_DEV < BCC1_DevCnt; BCC1_CID_DEV++) 
  {     
      BCC1_DeviceDataPtr->RcTbl[BCC1_CID_DEV] = 0U;
      BCC1_DeviceDataPtr->TagId[BCC1_CID_DEV] = 0U;
  }
  
 
  
  /* Wait for 5 ms (tvpwr(ready)) - for the IC to be ready for initialization. */
  // delay_ms(5);
  
  return BCC1_init_devs();
}

int a;
void GetVoltage(float *CellVol, float *StackVol, float *VADC,uint16_t RawMeas1[])
{
    (*StackVol) = BCC1_GET_STACK_VOLT(RawMeas1[msrSTACKVOLT]) / 1000U;
    CellVol[0] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT1]) / 1000U;
    CellVol[1] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT2]) / 1000U;
    CellVol[2] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT3]) / 1000U;
    CellVol[3] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT4]) / 1000U;
    CellVol[4] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT5]) / 1000U;
    CellVol[5] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT6]) / 1000U;
    CellVol[6] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT7]) / 1000U;
    CellVol[7] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT8]) / 1000U;
    CellVol[8] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT9]) / 1000U;
    CellVol[9] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT10]) / 1000U;
    CellVol[10] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT11]) / 1000U;
    CellVol[11] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT12]) / 1000U;
    CellVol[12] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT13]) / 1000U;
    CellVol[13] = BCC1_GET_VOLT(RawMeas1[msrCELLVOLT14]) / 1000U;
    
    VADC[0] = BCC1_GET_VOLT(RawMeas1[msrVBGADC1A]) / 1000U;
  VADC[1] = BCC1_GET_VOLT(RawMeas1[msrVBGADC1B]) / 1000U;
  
    

}

//extern uint32_t GPIOVol[];
uint32_t RSample[6];


void GetTemp(float *CellTemp, INT8U *HW_TempOpenFaultVld, INT8U *HW_TempShortFaultVld, float *ICTemp,float GPIOVol[],uint16_t RawMeas1[]) 
{    
    	INT8U i, j;       //variable value define

	for (j = 0; j < 6; j++)
	{
		HW_TempOpenFaultVld[j] = 1;
		HW_TempShortFaultVld[j] = 1;
	}

		(*ICTemp) = BCC1_GET_IC_TEMP(RawMeas1[msrICTEMP]);
		GPIOVol[0] = BCC1_GET_VOLT(RawMeas1[msrAN0]) / 1000U;
   	   GPIOVol[1] = BCC1_GET_VOLT(RawMeas1[msrAN1]) / 1000U;
		GPIOVol[2] = BCC1_GET_VOLT(RawMeas1[msrAN2]) / 1000U;
		 GPIOVol[3] = BCC1_GET_VOLT(RawMeas1[msrAN3]) / 1000U;
		  GPIOVol[4] = BCC1_GET_VOLT(RawMeas1[msrAN4]) / 1000U;
		   GPIOVol[5] = BCC1_GET_VOLT(RawMeas1[msrAN5]) / 1000U;
			GPIOVol[6] = BCC1_GET_VOLT(RawMeas1[msrAN6]) / 1000U;
        for(i = 0; i < 6; i++)
        {
				if (GPIOVol[i+1] > TempShot && GPIOVol[i+1] < TempOW)        //对是否断线或短路进行判断
			{
				RSample[i] = ((FP32)GPIOVol[i+1] * RTEMP_PU)/(VCOM - GPIOVol[i+1]);
				CellTemp[i] =  ResTemp_LookUp(RSample[i], ResTemp, RESTEMP_SIZE) + BCC_NTC_TEMP_MIN ;
				
			}
				else if (GPIOVol[i+1] <= TempShot)
			{
				HW_TempShortFaultVld[i] = 0;

				CellTemp[i] = TempInvalid;
			}
		else
		{
			HW_TempOpenFaultVld[i] = 0;

			CellTemp[i] = TempInvalid;
		}
        
        
}
}

#ifdef __cplusplus
}  /* extern "C" */
#endif 
/* END BCC1. */

