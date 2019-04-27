#include "ts.h"
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"

#define FT6x06_MAX_INSTANCE  2
int touch_fd =0;
int touch_io_init(int fd);
void     touch_io_write_reg(int fd,uint8_t addr, uint8_t reg, uint8_t value);
uint8_t touch_io_read_data(int fd,uint8_t addr,uint8_t reg);
uint16_t touch_io_read_multiple(int fd,uint8_t addr,uint8_t reg,uint8_t *buffer,uint16_t len);

int touch_io_init(int fd){
    i2c_attr_t i2c_attr;
    printf("ts i2c file id %d\n",fd);
    if(fd<=0){
        fd = open("/dev/i2c2", O_RDWR);
        printf("ts i2c have opened %d\n",fd);
    }
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    memset(&i2c_attr.pin_assignment, 0xff, sizeof(i2c_pin_assignment_t));
    i2c_attr.freq =10000;
    i2c_attr.slave_addr[0].addr16 = TS_I2C_ADDRESS>>1;
    i2c_attr.o_flags = I2C_FLAG_SET_MASTER|I2C_FLAG_STRETCH_CLOCK;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    return fd;
}
void touch_io_write_reg(int fd,uint8_t addr, uint8_t reg, uint8_t value){
    i2c_attr_t i2c_attr;
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    i2c_attr.o_flags = I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_IS_PTR_16;
    i2c_attr.slave_addr[0].addr16 = addr>>1;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    lseek(fd,(int)reg,SEEK_SET);
    write(fd,&value,1);
}
uint8_t touch_io_read_data(int fd,uint8_t addr,uint8_t reg){
    i2c_attr_t i2c_attr;
    u8 data;
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    i2c_attr.o_flags = I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_IS_PTR_16;
    i2c_attr.slave_addr[0].addr16 = addr>>1;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    lseek(fd,(int)reg,SEEK_SET);
    read(fd,&data,1);
    return data;
}
uint16_t touch_io_read_multiple(int fd,uint8_t addr,uint8_t reg,uint8_t *buffer,uint16_t len){
    i2c_attr_t i2c_attr;
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    i2c_attr.o_flags = I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_IS_PTR_16;
    i2c_attr.slave_addr[0].addr16 = addr>>1;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    lseek(fd,(int)reg,SEEK_SET);
    read(fd,buffer,len);
    return len;
}
void touch_delay(u32 delay){
    usleep(delay*1000);
}
/* Touch screen driver structure initialization */
TS_DrvTypeDef ft6x06_ts_drv ={
  ft6x06_Init,
  ft6x06_ReadID,
  ft6x06_Reset,

  ft6x06_TS_Start,
  ft6x06_TS_DetectTouch,
  ft6x06_TS_GetXY,

  ft6x06_TS_EnableIT,
  ft6x06_TS_ClearIT,
  ft6x06_TS_ITStatus,
  ft6x06_TS_DisableIT
};

/* ft6x06 instances by address */
u8 ft6x06[FT6x06_MAX_INSTANCE] = {0};

/* Global ft6x06 handle */
static ft6x06_handle_TypeDef ft6x06_handle = { FT6206_I2C_NOT_INITIALIZED, 0, 0};

static u8 ft6x06_GetInstance(u16 DeviceAddr);
/* Private functions prototypes-----------------------------------------------*/
#if (TS_AUTO_CALIBRATION_SUPPORTED == 1)
/**
  * @brief  Start TouchScreen calibration phase
  * @param  DeviceAddr: FT6206 Device address for communication on I2C Bus.
  * @retval Status FT6206_STATUS_OK or FT6206_STATUS_NOT_OK.
  */
static u32 ft6x06_TS_Calibration(u16 DeviceAddr);
#endif /* TS_AUTO_CALIBRATION_SUPPORTED == 1 */

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT6206 Device address for communication on I2C Bus.
  * @retval Status FT6206_STATUS_OK or FT6206_STATUS_NOT_OK.
  */
static u32 ft6x06_TS_Configure(u16 DeviceAddr);
/**
  * @brief  Initialize the ft6x06 communication bus
  *         from MCU to FT6206 : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
  * @retval None
  */
void ft6x06_Init(u16 DeviceAddr){
  u8 instance;
  u8 empty;
  /* Check if device instance already exists */
  instance = ft6x06_GetInstance(DeviceAddr);
  /* To prevent double initialization */
  if(instance == 0xFF)  {
    /* Look for empty instance */
    empty = ft6x06_GetInstance(0);
    if(empty < FT6x06_MAX_INSTANCE)    {
      /* Register the current device instance */
      ft6x06[empty] = (u8)DeviceAddr;
      /* Initialize IO BUS layer */
      printf("ts before an init \n");
      touch_fd = TS_IO_Init();
      printf("ts file id %d\n", touch_fd);
    }
  }
}

/**
  * @brief  Software Reset the ft6x06.
  *         @note : Not applicable to FT6206.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
  * @retval None
  */
void ft6x06_Reset(u16 DeviceAddr){
  /* Do nothing */
  /* No software reset sequence available in FT6206 IC */
}

/**
  * @brief  Read the ft6x06 device ID, pre initialize I2C in case of need to be
  *         able to read the FT6206 device ID, and verify this is a FT6206.
  * @param  DeviceAddr: I2C FT6x06 Slave address.
  * @retval The Device ID (two bytes).
  */
u16 ft6x06_ReadID(u16 DeviceAddr){
  /* Initialize I2C link if needed */
  //TS_IO_Init();
  /* Return the device ID value */
  return (TS_IO_Read(DeviceAddr, FT6206_CHIP_ID_REG));
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  *         It goes through an internal calibration process (Hw calibration sequence of
  *         the touch screen).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void ft6x06_TS_Start(u16 DeviceAddr){
#if (TS_AUTO_CALIBRATION_SUPPORTED == 1)
  /* Hw Calibration sequence start : should be done once after each power up */
  /* This is called internal calibration of the touch screen                 */
  ft6x06_TS_Calibration(DeviceAddr);
#endif
  /* Minimum static configuration of FT6206 */
  ft6x06_TS_Configure(DeviceAddr);

  /* By default set FT6206 IC in Polling mode : no INT generation on FT6206 for new touch available */
  /* Note TS_INT is active low                                                                      */
  ft6x06_TS_DisableIT(DeviceAddr);
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
u8 ft6x06_TS_DetectTouch(u16 DeviceAddr){
  volatile u8 nbTouch = 0;

  /* Read register FT6206_TD_STAT_REG to check number of touches detection */
  nbTouch = TS_IO_Read((u8)DeviceAddr, FT6206_TD_STAT_REG);
  nbTouch &= FT6206_TD_STAT_MASK;

  if(nbTouch > FT6206_MAX_DETECTABLE_TOUCH)
  {
    /* If invalid number of touch detected, set it to zero */
    nbTouch = 0;
  }

  /* Update ft6x06 driver internal global : current number of active touches */
  ft6x06_handle.currActiveTouchNb = nbTouch;

  /* Reset current active touch index on which to work on */
  ft6x06_handle.currActiveTouchIdx = 0;

  return(nbTouch);
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'ft6x06_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void ft6x06_TS_GetXY(u16 DeviceAddr, u16 *X, u16 *Y){
  u8 regAddress = 0;
  u8  dataxy[4];
  
  if(ft6x06_handle.currActiveTouchIdx < ft6x06_handle.currActiveTouchNb)  {
    switch(ft6x06_handle.currActiveTouchIdx){
    case 0 :    
      regAddress = FT6206_P1_XH_REG; 
      break;
    case 1 :
      regAddress = FT6206_P2_XH_REG; 
      break;

    default :
      break;
    }
    /* Read X and Y positions */
    for (int i=0;i<4;i++){
        //dataxy[i] = TS_IO_Read((u8)DeviceAddr, (u8)(regAddress+i));
        TS_IO_ReadMultiple((u8)DeviceAddr, (u8)(regAddress+i), &dataxy[i], 1);
    }
    //TS_IO_ReadMultiple((u8)DeviceAddr, regAddress, dataxy, 4);
    /* Send back ready X position to caller */
    *X = (u16)((u16)(dataxy[0] & FT6206_MSB_MASK) << 8) | (u16)(dataxy[1] & FT6206_LSB_MASK);
    /* Send back ready Y position to caller */
    *Y = (u16)((u16)(dataxy[2] & FT6206_MSB_MASK) << 8) | (u16)(dataxy[3] & FT6206_LSB_MASK);
    ft6x06_handle.currActiveTouchIdx++;
  }
}

/**
  * @brief  Configure the FT6206 device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
  * @retval None
  */
void ft6x06_TS_EnableIT(u16 DeviceAddr){
  u8 regValue = 0;
  regValue = (FT6206_G_MODE_INTERRUPT_TRIGGER & (FT6206_G_MODE_INTERRUPT_MASK >> FT6206_G_MODE_INTERRUPT_SHIFT)) << FT6206_G_MODE_INTERRUPT_SHIFT;
  /* Set interrupt trigger mode in FT6206_GMODE_REG */
  TS_IO_Write(DeviceAddr, FT6206_GMODE_REG, regValue);
}

/**
  * @brief  Configure the FT6206 device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT6206).
  * @retval None
  */
void ft6x06_TS_DisableIT(u16 DeviceAddr){
  u8 regValue = 0;
  regValue = (FT6206_G_MODE_INTERRUPT_POLLING & (FT6206_G_MODE_INTERRUPT_MASK >> FT6206_G_MODE_INTERRUPT_SHIFT)) << FT6206_G_MODE_INTERRUPT_SHIFT;

  /* Set interrupt polling mode in FT6206_GMODE_REG */
  TS_IO_Write(DeviceAddr, FT6206_GMODE_REG, regValue);
}

/**
  * @brief  Get IT status from FT6206 interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to FT6206.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
  * @retval TS interrupts status : always return 0 here
  */
u8 ft6x06_TS_ITStatus(u16 DeviceAddr){
  /* Always return 0 as feature not applicable to FT6206 */
  return 0;
}

/**
  * @brief  Clear IT status in FT6206 interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to FT6206.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6206).
  * @retval None
  */
void ft6x06_TS_ClearIT(u16 DeviceAddr)
{
  /* Nothing to be done here for FT6206 */
}

/**** NEW FEATURES enabled when Multi-touch support is enabled ****/

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT6x06).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void ft6x06_TS_GetGestureID(u16 DeviceAddr, u32 * pGestureId)
{
  volatile u8 ucReadData = 0;

  ucReadData = TS_IO_Read(DeviceAddr, FT6206_GEST_ID_REG);

  * pGestureId = ucReadData;
}

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
                            u32 * pEvent)
{
  u8 regAddress = 0;
  u8 dataxy[3];
  
  if(touchIdx < ft6x06_handle.currActiveTouchNb)
  {
    switch(touchIdx)
    {
    case 0 : 
      regAddress = FT6206_P1_WEIGHT_REG;
      break;
      
    case 1 :
      regAddress = FT6206_P2_WEIGHT_REG;
      break;
      
    default :
      break;
      
    } /* end switch(touchIdx) */
    
    /* Read weight, area and Event Id of touch index */
    TS_IO_ReadMultiple(DeviceAddr, regAddress, dataxy, sizeof(dataxy)); 
    
    /* Return weight of touch index */
    * pWeight = (dataxy[0] & FT6206_TOUCH_WEIGHT_MASK) >> FT6206_TOUCH_WEIGHT_SHIFT;
    /* Return area of touch index */
    * pArea = (dataxy[1] & FT6206_TOUCH_AREA_MASK) >> FT6206_TOUCH_AREA_SHIFT;
    /* Return Event Id  of touch index */
    * pEvent = (dataxy[2] & FT6206_TOUCH_EVT_FLAG_MASK) >> FT6206_TOUCH_EVT_FLAG_SHIFT;
    
  } /* of if(touchIdx < ft6x06_handle.currActiveTouchNb) */
}

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

#if (TS_AUTO_CALIBRATION_SUPPORTED == 1)
/**
  * @brief  Start TouchScreen calibration phase
  * @param  DeviceAddr: FT6206 Device address for communication on I2C Bus.
  * @retval Status FT6206_STATUS_OK or FT6206_STATUS_NOT_OK.
  */
static u32 ft6x06_TS_Calibration(u16 DeviceAddr)
{
  u32 nbAttempt = 0;
  volatile u8 ucReadData;
  volatile u8 regValue;
  u32 status = FT6206_STATUS_OK;
  u8 bEndCalibration = 0;

  /* >> Calibration sequence start */

  /* Switch FT6206 back to factory mode to calibrate */
  regValue = (FT6206_DEV_MODE_FACTORY & FT6206_DEV_MODE_MASK) << FT6206_DEV_MODE_SHIFT;
  TS_IO_Write(DeviceAddr, FT6206_DEV_MODE_REG, regValue); /* 0x40 */

  /* Read back the same register FT6206_DEV_MODE_REG */
  ucReadData = TS_IO_Read(DeviceAddr, FT6206_DEV_MODE_REG);
  TS_IO_Delay(300); /* Wait 300 ms */

  if(((ucReadData & (FT6206_DEV_MODE_MASK << FT6206_DEV_MODE_SHIFT)) >> FT6206_DEV_MODE_SHIFT) != FT6206_DEV_MODE_FACTORY )
  {
    /* Return error to caller */
    return(FT6206_STATUS_NOT_OK);
  }

  /* Start calibration command */
  TS_IO_Write(DeviceAddr, FT6206_TD_STAT_REG, 0x04);
  TS_IO_Delay(300); /* Wait 300 ms */

  /* 100 attempts to wait switch from factory mode (calibration) to working mode */
  for (nbAttempt=0; ((nbAttempt < 100) && (!bEndCalibration)) ; nbAttempt++)
  {
    ucReadData = TS_IO_Read(DeviceAddr, FT6206_DEV_MODE_REG);
    ucReadData = (ucReadData & (FT6206_DEV_MODE_MASK << FT6206_DEV_MODE_SHIFT)) >> FT6206_DEV_MODE_SHIFT;
    if(ucReadData == FT6206_DEV_MODE_WORKING)
    {
      /* Auto Switch to FT6206_DEV_MODE_WORKING : means calibration have ended */
      bEndCalibration = 1; /* exit for loop */
    }
    
    TS_IO_Delay(200); /* Wait 200 ms */
  }

  /* Calibration sequence end << */

  return(status);
}
#endif /* TS_AUTO_CALIBRATION_SUPPORTED == 1 */

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT6206 Device address for communication on I2C Bus.
  * @retval Status FT6206_STATUS_OK or FT6206_STATUS_NOT_OK.
  */
static u32 ft6x06_TS_Configure(u16 DeviceAddr){
  u32 status = FT6206_STATUS_OK;

  /* Nothing special to be done for FT6206 */

  return(status);
}

/**
  * @brief  Check if the device instance of the selected address is already registered
  *         and return its index  
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Index of the device instance if registered, 0xFF if not.
  */
static u8 ft6x06_GetInstance(u16 DeviceAddr){
  u8 idx = 0;
  /* Check all the registered instances */
  for(idx = 0; idx < FT6x06_MAX_INSTANCE ; idx ++) {
    if(ft6x06[idx] == DeviceAddr){
      return idx; 
    }
  }
  return 0xFF;
}
