#ifndef __TS_C
#define __TS_C

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h> 
#include "ts.h"

uint8_t  tsOrientation = TS_SWAP_NONE;
/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t ts_get_state(TS_StateTypeDef *TS_State){
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  uint8_t ts_status = TS_OK;
  uint16_t tmp;
  uint16_t Raw_x[TS_MAX_NB_TOUCH];
  uint16_t Raw_y[TS_MAX_NB_TOUCH];
  uint16_t xDiff;
  uint16_t yDiff;
  uint32_t index;
  /* Check and update the number of touches active detected */
  TS_State->touchDetected = ft6x06_TS_DetectTouch(TS_I2C_ADDRESS);
  if(TS_State->touchDetected){
    for(index=0; index < TS_State->touchDetected; index++){
      /* Get each touch coordinates */
      ft6x06_TS_GetXY(TS_I2C_ADDRESS, &(Raw_x[index]), &(Raw_y[index]));
      if(tsOrientation & TS_SWAP_XY){
        tmp = Raw_x[index];
        Raw_x[index] = Raw_y[index];
        Raw_y[index] = tmp;
      }
      if(tsOrientation & TS_SWAP_X){
        Raw_x[index] = FT_6206_MAX_WIDTH_HEIGHT - 1 - Raw_x[index];
      }
      if(tsOrientation & TS_SWAP_Y){
        Raw_y[index] = FT_6206_MAX_WIDTH_HEIGHT - 1 - Raw_y[index];
      }
      xDiff = Raw_x[index] > _x[index]? (u16)(Raw_x[index] - _x[index]): (u16)(_x[index] - Raw_x[index]);
      yDiff = Raw_y[index] > _y[index]? (u16)(Raw_y[index] - _y[index]): (u16)(_y[index] - Raw_y[index]);
      if ((xDiff + yDiff) > 5){
        _x[index] = Raw_x[index];
        _y[index] = Raw_y[index];
      }
      TS_State->touchX[index] = (u16)_x[index];
      TS_State->touchY[index] = (u16)_y[index];
    } /* of for(index=0; index < TS_State->touchDetected; index++) */
  } /* end of if(TS_State->touchDetected != 0) */
  return (ts_status);
}

#ifdef __cplusplus
}
#endif

#endif /* __TS_C */
