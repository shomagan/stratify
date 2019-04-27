#ifndef __FT6X06_H
#define __FT6X06_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <stdint.h>

/* Set Multi-touch as non supported */
#ifndef TS_MULTI_TOUCH_SUPPORTED
  #define TS_MULTI_TOUCH_SUPPORTED 0
#endif

/* Set Auto-calibration as non supported */  
#ifndef TS_AUTO_CALIBRATION_SUPPORTED
  #define TS_AUTO_CALIBRATION_SUPPORTED 0
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "ts.h"
#include "mcu/types.h"
/* Macros --------------------------------------------------------------------*/
#define TS_I2C_ADDRESS                   ((uint16_t)0x70)
/** @typedef ft6x06_handle_TypeDef
 *  ft6x06 Handle definition.
 */
typedef struct
{
  u8 i2cInitialized;

  /* field holding the current number of simultaneous active touches */
  u8 currActiveTouchNb;

  /* field holding the touch index currently managed */
  u8 currActiveTouchIdx;

} ft6x06_handle_TypeDef;

  /* Maximum border values of the touchscreen pad */
#define  FT_6206_MAX_WIDTH              ((u16)800)     /* Touchscreen pad max width   */
#define  FT_6206_MAX_HEIGHT             ((u16)480)     /* Touchscreen pad max height  */

  /* Touchscreen pad max width and height values for FT6x36 Touch*/
#define  FT_6206_MAX_WIDTH_HEIGHT       ((u16)240)     

  /* Possible values of driver functions return status */
#define FT6206_STATUS_OK                0
#define FT6206_STATUS_NOT_OK            1

  /* Possible values of global variable 'TS_I2C_Initialized' */
#define FT6206_I2C_NOT_INITIALIZED      0
#define FT6206_I2C_INITIALIZED          1

  /* Max detectable simultaneous touches */
#define FT6206_MAX_DETECTABLE_TOUCH     2

  /**
   * @brief : Definitions for FT6206 I2C register addresses on 8 bit
   **/

  /* Current mode register of the FT6206 (R/W) */
#define FT6206_DEV_MODE_REG             0x00

  /* Possible values of FT6206_DEV_MODE_REG */
#define FT6206_DEV_MODE_WORKING         0x00
#define FT6206_DEV_MODE_FACTORY         0x04

#define FT6206_DEV_MODE_MASK            0x7
#define FT6206_DEV_MODE_SHIFT           4

  /* Gesture ID register */
#define FT6206_GEST_ID_REG              0x01

  /* Possible values of FT6206_GEST_ID_REG */
#define FT6206_GEST_ID_NO_GESTURE       0x00
#define FT6206_GEST_ID_MOVE_UP          0x10
#define FT6206_GEST_ID_MOVE_RIGHT       0x14
#define FT6206_GEST_ID_MOVE_DOWN        0x18
#define FT6206_GEST_ID_MOVE_LEFT        0x1C
#define FT6206_GEST_ID_ZOOM_IN          0x48
#define FT6206_GEST_ID_ZOOM_OUT         0x49

  /* Touch Data Status register : gives number of active touch points (0..2) */
#define FT6206_TD_STAT_REG              0x02

  /* Values related to FT6206_TD_STAT_REG */
#define FT6206_TD_STAT_MASK             0x0F
#define FT6206_TD_STAT_SHIFT            0x00

  /* Values Pn_XH and Pn_YH related */
#define FT6206_TOUCH_EVT_FLAG_PRESS_DOWN 0x00
#define FT6206_TOUCH_EVT_FLAG_LIFT_UP    0x01
#define FT6206_TOUCH_EVT_FLAG_CONTACT    0x02
#define FT6206_TOUCH_EVT_FLAG_NO_EVENT   0x03

#define FT6206_TOUCH_EVT_FLAG_SHIFT     6
#define FT6206_TOUCH_EVT_FLAG_MASK      (3 << FT6206_TOUCH_EVT_FLAG_SHIFT)

#define FT6206_MSB_MASK                 0x0F
#define FT6206_MSB_SHIFT                0

  /* Values Pn_XL and Pn_YL related */
#define FT6206_LSB_MASK                 0xFF
#define FT6206_LSB_SHIFT                0

#define FT6206_P1_XH_REG                0x03
#define FT6206_P1_XL_REG                0x04
#define FT6206_P1_YH_REG                0x05
#define FT6206_P1_YL_REG                0x06

  /* Touch Pressure register value (R) */
#define FT6206_P1_WEIGHT_REG            0x07

  /* Values Pn_WEIGHT related  */
#define FT6206_TOUCH_WEIGHT_MASK        0xFF
#define FT6206_TOUCH_WEIGHT_SHIFT       0

  /* Touch area register */
#define FT6206_P1_MISC_REG              0x08

  /* Values related to FT6206_Pn_MISC_REG */
#define FT6206_TOUCH_AREA_MASK         (0x04 << 4)
#define FT6206_TOUCH_AREA_SHIFT        0x04

#define FT6206_P2_XH_REG               0x09
#define FT6206_P2_XL_REG               0x0A
#define FT6206_P2_YH_REG               0x0B
#define FT6206_P2_YL_REG               0x0C
#define FT6206_P2_WEIGHT_REG           0x0D
#define FT6206_P2_MISC_REG             0x0E

  /* Threshold for touch detection */
#define FT6206_TH_GROUP_REG            0x80

  /* Values FT6206_TH_GROUP_REG : threshold related  */
#define FT6206_THRESHOLD_MASK          0xFF
#define FT6206_THRESHOLD_SHIFT         0

  /* Filter function coefficients */
#define FT6206_TH_DIFF_REG             0x85

  /* Control register */
#define FT6206_CTRL_REG                0x86

  /* Values related to FT6206_CTRL_REG */

  /* Will keep the Active mode when there is no touching */
#define FT6206_CTRL_KEEP_ACTIVE_MODE    0x00

  /* Switching from Active mode to Monitor mode automatically when there is no touching */
#define FT6206_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE  0x01

  /* The time period of switching from Active mode to Monitor mode when there is no touching */
#define FT6206_TIMEENTERMONITOR_REG     0x87

  /* Report rate in Active mode */
#define FT6206_PERIODACTIVE_REG         0x88

  /* Report rate in Monitor mode */
#define FT6206_PERIODMONITOR_REG        0x89

  /* The value of the minimum allowed angle while Rotating gesture mode */
#define FT6206_RADIAN_VALUE_REG         0x91

  /* Maximum offset while Moving Left and Moving Right gesture */
#define FT6206_OFFSET_LEFT_RIGHT_REG    0x92

  /* Maximum offset while Moving Up and Moving Down gesture */
#define FT6206_OFFSET_UP_DOWN_REG       0x93

  /* Minimum distance while Moving Left and Moving Right gesture */
#define FT6206_DISTANCE_LEFT_RIGHT_REG  0x94

  /* Minimum distance while Moving Up and Moving Down gesture */
#define FT6206_DISTANCE_UP_DOWN_REG     0x95

  /* Maximum distance while Zoom In and Zoom Out gesture */
#define FT6206_DISTANCE_ZOOM_REG        0x96

  /* High 8-bit of LIB Version info */
#define FT6206_LIB_VER_H_REG            0xA1

  /* Low 8-bit of LIB Version info */
#define FT6206_LIB_VER_L_REG            0xA2

  /* Chip Selecting */
#define FT6206_CIPHER_REG               0xA3

  /* Interrupt mode register (used when in interrupt mode) */
#define FT6206_GMODE_REG                0xA4

#define FT6206_G_MODE_INTERRUPT_MASK    0x03
#define FT6206_G_MODE_INTERRUPT_SHIFT   0x00

  /* Possible values of FT6206_GMODE_REG */
#define FT6206_G_MODE_INTERRUPT_POLLING 0x00
#define FT6206_G_MODE_INTERRUPT_TRIGGER 0x01

  /* Current power mode the FT6206 system is in (R) */
#define FT6206_PWR_MODE_REG             0xA5

  /* FT6206 firmware version */
#define FT6206_FIRMID_REG               0xA6

  /* FT6206 Chip identification register */
#define FT6206_CHIP_ID_REG              0xA8

  /*  Possible values of FT6206_CHIP_ID_REG */
#define FT6206_ID_VALUE                 0x11
#define FT6x36_ID_VALUE                 0xCD

  /* Release code version */
#define FT6206_RELEASE_CODE_ID_REG      0xAF

  /* Current operating mode the FT6206 system is in (R) */
#define FT6206_STATE_REG                0xBC

/**
 * @brief  Initialize the ft6x06 communication bus
 *         from MCU to FT6206 : ie I2C channel initialization (if required).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval None
 */
void ft6x06_Init(u16 DeviceAddr);

/**
 * @brief  Software Reset the ft6x06.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval None
 */
void ft6x06_Reset(u16 DeviceAddr);

/**
 * @brief  Read the ft6x06 device ID, pre intitalize I2C in case of need to be
 *         able to read the FT6206 device ID, and verify this is a FT6206.
 * @param  DeviceAddr: I2C FT6x06 Slave address.
 * @retval The Device ID (two bytes).
 */
u16 ft6x06_ReadID(u16 DeviceAddr);

/**
 * @brief  Configures the touch Screen IC device to start detecting touches
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
 * @retval None.
 */
void ft6x06_TS_Start(u16 DeviceAddr);

/**
 * @brief  Return if there is touches detected or not.
 *         Try to detect new touches and forget the old ones (reset internal global
 *         variables).
 * @param  DeviceAddr: Device address on communication Bus.
 * @retval : Number of active touches detected (can be 0, 1 or 2).
 */
u8 ft6x06_TS_DetectTouch(u16 DeviceAddr);

/**
 * @brief  Get the touch screen X and Y positions values
 *         Manage multi touch thanks to touch Index global
 *         variable 'ft6x06_handle.currActiveTouchIdx'.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  X: Pointer to X position value
 * @param  Y: Pointer to Y position value
 * @retval None.
 */
void ft6x06_TS_GetXY(u16 DeviceAddr, u16 *X, u16 *Y);

/**
 * @brief  Configure the FT6206 device to generate IT on given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
 * @retval None
 */
void ft6x06_TS_EnableIT(u16 DeviceAddr);

/**
 * @brief  Configure the FT6206 device to stop generating IT on the given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
 * @retval None
 */
void ft6x06_TS_DisableIT(u16 DeviceAddr);

/**
 * @brief  Get IT status from FT6206 interrupt status registers
 *         Should be called Following an EXTI coming to the MCU to know the detailed
 *         reason of the interrupt.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval TS interrupts status
 */
u8 ft6x06_TS_ITStatus (u16 DeviceAddr);

/**
 * @brief  Clear IT status in FT6206 interrupt status clear registers
 *         Should be called Following an EXTI coming to the MCU.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
 * @retval TS interrupts status
 */
void ft6x06_TS_ClearIT (u16 DeviceAddr);

/**** NEW FEATURES enabled when Multi-touch support is enabled ****/

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

/**
 * @brief  Get the last touch gesture identification (zoom, move up/down...).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6x06).
 * @param  pGestureId : Pointer to get last touch gesture Identification.
 * @retval None.
 */
void ft6x06_TS_GetGestureID(u16 DeviceAddr, u32 * pGestureId);

/**
 * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
 *         This touch detailed information contains :
 *         - weight that was applied to this touch
 *         - sub-area of the touch in the touch panel
 *         - event of linked to the touch (press down, lift up, ...)
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6x06).
 * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
 *                    detailed information.
 * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
 * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
 * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

 * @retval None.
 */
void ft6x06_TS_GetTouchInfo(u16   DeviceAddr,
                            u32   touchIdx,
                            u32 * pWeight,
                            u32 * pArea,
                            u32 * pEvent);

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/* Imported TS IO functions --------------------------------------------------------*/


/* TouchScreen (TS) external IO functions */
extern int touch_fd;
extern int touch_io_init(int fd);
extern void     touch_io_write_reg(int fd,uint8_t addr, uint8_t reg, uint8_t value);
extern uint8_t touch_io_read_data(int fd,uint8_t addr,uint8_t reg);
extern uint16_t touch_io_read_multiple(int fd,uint8_t addr,uint8_t reg,uint8_t *buffer,uint16_t len);

#define TS_IO_Init() touch_io_init(touch_fd)
#define TS_IO_Write(a,x,y) touch_io_write_reg(touch_fd,a,x,y)
#define TS_IO_Read(a,r) touch_io_read_data(touch_fd,a,r)
#define TS_IO_ReadMultiple(a,r,d,l) touch_io_read_multiple(touch_fd,a,r,d,l)
#define TS_IO_Delay(x) usleep(x*1000)




#ifdef __cplusplus
}
#endif
#endif /* __FT6X06_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
