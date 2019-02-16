/**
  ******************************************************************************
  * @file    lcd.h
  * @author  MCD Application Team
  * @version V4.0.1
  * @date    21-July-2015
  * @brief   This file contains all the functions prototypes for the LCD driver.   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "fonts.h"
/** @addtogroup BSP
  * @{
  */
#define  LCD_ORIENTATION_PORTRAIT         ((uint8_t)0x00)  /*!< Portrait orientation choice of LCD screen  */
#define  LCD_ORIENTATION_LANDSCAPE        ((uint8_t)0x01)  /*!< Landscape orientation choice of LCD screen */
#define  LCD_ORIENTATION_LANDSCAPE_ROT180 ((uint32_t)0x02) /*!< Landscape rotated 180° orientation choice of LCD screen */

/**
* @brief  LCD color
*/
#define LCD_COLOR_BLUE          ((uint16_t)0x001F)
#define LCD_COLOR_GREEN         ((uint16_t)0x07E0)
#define LCD_COLOR_RED           ((uint16_t)0xF800)
#define LCD_COLOR_CYAN          ((uint16_t)0x07FF)
#define LCD_COLOR_MAGENTA       ((uint16_t)0xF81F)
#define LCD_COLOR_YELLOW        ((uint16_t)0xFFE0)
#define LCD_COLOR_LIGHTBLUE     ((uint16_t)0x841F)
#define LCD_COLOR_LIGHTGREEN    ((uint16_t)0x87F0)
#define LCD_COLOR_LIGHTRED      ((uint16_t)0xFC10)
#define LCD_COLOR_LIGHTMAGENTA  ((uint16_t)0xFC1F)
#define LCD_COLOR_LIGHTYELLOW   ((uint16_t)0xFFF0)
#define LCD_COLOR_DARKBLUE      ((uint16_t)0x0010)
#define LCD_COLOR_DARKGREEN     ((uint16_t)0x0400)
#define LCD_COLOR_DARKRED       ((uint16_t)0x8000)
#define LCD_COLOR_DARKCYAN      ((uint16_t)0x0410)
#define LCD_COLOR_DARKMAGENTA   ((uint16_t)0x8010)
#define LCD_COLOR_DARKYELLOW    ((uint16_t)0x8400)
#define LCD_COLOR_WHITE         ((uint16_t)0xFFFF)
#define LCD_COLOR_LIGHTGRAY     ((uint16_t)0xD69A)
#define LCD_COLOR_GRAY          ((uint16_t)0x8410)
#define LCD_COLOR_DARKGRAY      ((uint16_t)0x4208)
#define LCD_COLOR_BLACK         ((uint16_t)0x0000)
#define LCD_COLOR_BROWN         ((uint16_t)0xA145)
#define LCD_COLOR_ORANGE        ((uint16_t)0xFD20)

typedef struct  {
   uint32_t TextColor;
   uint32_t BackColor;
   sFONT    *pFont;
}LCD_DrawPropTypeDef;

/** @addtogroup Components
  * @{
  */

/** @addtogroup LCD
  * @{
  */
 
/** @defgroup LCD_Exported_Types
  * @{
  */

/** @defgroup LCD_Driver_structure  LCD Driver structure
  * @{
  */
typedef struct
{
  void     (*Init)(void);
  uint16_t (*ReadID)(void);
  void     (*DisplayOn)(void);
  void     (*DisplayOff)(void);
  void     (*SetCursor)(uint16_t, uint16_t);
  void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
  uint16_t (*ReadPixel)(uint16_t, uint16_t);
  
   /* Optimized operation */
  void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  
  uint16_t (*GetLcdPixelWidth)(void);
  uint16_t (*GetLcdPixelHeight)(void);
  void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
  void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*);
}LCD_DrvTypeDef;    
 void lcd_draw_char(uint16_t x, uint16_t y, uint8_t val);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
