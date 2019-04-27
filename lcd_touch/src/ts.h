/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TS_H
#define __TS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> 
#include  "ft6x06.h"
#define TS_SWAP_NONE                    ((uint8_t) 0x01)
#define TS_SWAP_X                       ((uint8_t) 0x02)
#define TS_SWAP_Y                       ((uint8_t) 0x04)
#define TS_SWAP_XY                      ((uint8_t) 0x08)

#define TS_ORIENTATION_PORTRAIT         ((uint8_t) 0x00)
#define TS_ORIENTATION_LANDSCAPE        ((uint8_t) 0x01)
#define TS_ORIENTATION_LANDSCAPE_ROT180 ((uint8_t) 0x02)
#define TS_MAX_NB_TOUCH                 ((uint32_t)FT6206_MAX_DETECTABLE_TOUCH)
extern uint8_t  tsOrientation;

typedef struct
{  
  void       (*Init)(uint16_t);
  uint16_t   (*ReadID)(uint16_t);
  void       (*Reset)(uint16_t);
  void       (*Start)(uint16_t);
  uint8_t    (*DetectTouch)(uint16_t);
  void       (*GetXY)(uint16_t, uint16_t*, uint16_t*);
  void       (*EnableIT)(uint16_t);
  void       (*ClearIT)(uint16_t);
  uint8_t    (*GetITStatus)(uint16_t);
  void       (*DisableIT)(uint16_t);
}TS_DrvTypeDef;
/* Touch screen driver structure */
extern TS_DrvTypeDef ft6x06_ts_drv;

/**
*  @brief TS_StateTypeDef
*  Define TS State structure
*/
typedef struct{
  uint8_t  touchDetected;                /*!< Total number of active touches detected at last scan */
  uint16_t touchX[TS_MAX_NB_TOUCH];      /*!< Touch X[0], X[1] coordinates on 12 bits */
  uint16_t touchY[TS_MAX_NB_TOUCH];      /*!< Touch Y[0], Y[1] coordinates on 12 bits */
} TS_StateTypeDef;

typedef enum
{
  TS_OK                = 0x00, /*!< Touch Ok */
  TS_ERROR             = 0x01, /*!< Touch Error */
  TS_TIMEOUT           = 0x02, /*!< Touch Timeout */
  TS_DEVICE_NOT_FOUND  = 0x03  /*!< Touchscreen device not found */
} TS_StatusTypeDef;
uint8_t ts_get_state(TS_StateTypeDef *TS_State);
#ifdef __cplusplus
}
#endif

#endif /* __TS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
