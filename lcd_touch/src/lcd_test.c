#ifndef LCD_TEST_C_
#define LCD_TEST_C_
#include <stdio.h>
#include <string.h>
#include "lcd_test.h"
#include "lcd.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"
#include "st7789h2.h"
#include "ts.h"

#define FMC_BANK2_BASE  ((uint32_t)(0x60000000 | 0x04000000))  
#define FMC_BANK2       ((LCD_CONTROLLER_TypeDef *) FMC_BANK2_BASE)
#define SOS_BOARD_LCD_BL_PORT 7 //PH11
#define SOS_BOARD_LCD_BL_PIN 11
#define SOS_BOARD_LCD_TE_INT_PORT 2 //PC8
#define SOS_BOARD_LCD_TE_INT_PIN 8
#define SOS_BOARD_LCD_RST_PORT 7 //PH7
#define SOS_BOARD_LCD_RST_PIN 7
#define SOS_BOARD_LCD_RST_TOUCH_PORT 7 //PH9
#define SOS_BOARD_LCD_RST_TOUCH_PIN 9

#define SOS_BOARD_LED6_PIN 1
#define SOS_BOARD_LED5_PIN 7
#define ST7789H2_LCD_ID             0x04

#define TS_I2C_ADDRESS                   ((uint16_t)0x70)
#define DEBUG 0

static void ts_wait_for_press_state(uint8_t press_state,TS_StateTypeDef * TS_State);

int lcd_fd =-1;
int lcd_test(void){
    u16 size_x, size_y;
    size_x=0; size_y=0;
    printf("lcd example c\n");
    int reset_bl_pio_fd = open("/dev/pio7",O_RDWR); //port_H
    pio_attr_t pio_attr;
    u16 id;
    pio_attr.o_pinmask = (1<<SOS_BOARD_LCD_BL_PIN)|(1<<SOS_BOARD_LCD_RST_PIN)|(1<<SOS_BOARD_LCD_RST_TOUCH_PIN);
    pio_attr.o_flags = PIO_FLAG_IS_SPEED_LOW | PIO_FLAG_SET_OUTPUT;
    ioctl(reset_bl_pio_fd, I_PIO_SETATTR, &pio_attr);
    ioctl(reset_bl_pio_fd, I_PIO_SETMASK, (1<<SOS_BOARD_LCD_BL_PIN));
    ioctl(reset_bl_pio_fd, I_PIO_SETMASK, (1<<SOS_BOARD_LCD_RST_TOUCH_PIN));
    printf("bl set \n");
    printf("lcd reset \n");

    lcd_fd = open("/dev/lcd0", O_RDWR);
    /*with setting configuration*/
    printf("lcd fd %d \n",lcd_fd);
    if(lcd_fd<0){
        printf("lcd dev have't opened c \n");
    }else{
        printf("lcd dev have opened c \n");
        reset_lcd(reset_bl_pio_fd,(SOS_BOARD_LCD_RST_PIN));
        ST7789H2_Init();
        printf("display init \n");
        id = (u8)ST7789H2_ReadID();
        printf("id %u \n",id);
        u8 y = 0;
        if(id ==ST7789H2_ID){
            ST7789H2_SetOrientation(LCD_ORIENTATION_PORTRAIT);
            size_y = ST7789H2_GetLcdPixelHeight();
            size_x = ST7789H2_GetLcdPixelWidth();
            printf("x - %u, y - %u \n",size_y,size_x);
            for (u16 i=0;i<size_y;i++){
                ST7789H2_DrawHLine(LCD_COLOR_BLACK,0,i,size_x);
            }

            lcd_draw_char(size_x/2, y, 't');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'o');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'u');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'c');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'h');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, ' ');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'm');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'e');
            y += Font24.Height;
        }
        ft6x06_Init(TS_I2C_ADDRESS);
        printf("ts have inited\n");
        usleep(1000);
        id=ft6x06_ReadID(TS_I2C_ADDRESS);
        printf("ts have read\n");
        u8 touch_number = 6;
        u8 touch_number_char  = 0x30;
        if(id == FT6x36_ID_VALUE){
            printf("ft6x06 id equals %u \n",id);
            tsOrientation = TS_SWAP_X | TS_SWAP_Y;
            ft6x06_TS_Start(TS_I2C_ADDRESS);
            /* Wait until pressed state on the touch panel */
            TS_StateTypeDef TS_State;
            TS_State.touchX[0]=0;
            TS_State.touchX[1]=0;
            TS_State.touchY[0]=0;
            TS_State.touchY[1]=0;
            touch_number_char = 0x30 + touch_number;
            lcd_draw_char(size_x/2, y, touch_number_char);
            ts_wait_for_press_state(1,&TS_State);
            printf("ts0 x - %u , y - %u \n",TS_State.touchX[0],TS_State.touchY[0]);
            lcd_draw_char(TS_State.touchX[0],TS_State.touchY[0], 'x');
            touch_number--;
            touch_number_char = 0x30 + touch_number;
            lcd_draw_char(size_x/2, y, touch_number_char);
            /* Wait until touch is released on touch panel */
            ts_wait_for_press_state(0,&TS_State);
            printf("ts0 x - %u , y - %u \n",TS_State.touchX[0],TS_State.touchY[0]);
            for(int i=0;i<5;){
                ts_wait_for_press_state(1,&TS_State);
                touch_number--;
                touch_number_char = 0x30 + touch_number;
                lcd_draw_char(size_x/2, y, touch_number_char);
                lcd_draw_char(TS_State.touchX[0],TS_State.touchY[0], 'x');
                printf("ts0 x - %u , y - %u \n",TS_State.touchX[0],TS_State.touchY[0]);
                i++;
            }
        }else{
            printf("ft6x06 id dont equal\n");
        }
        close(touch_fd);
        close(lcd_fd);
    }
    /*use default board configuration*/
    lcd_fd= open("/dev/lcd0", O_RDWR);
    if(lcd_fd<0){
        printf("lcd dev have't opened c\n");
    }else{
        printf("lcd dev have opened c\n");
        close(lcd_fd);
    }
    sleep(1);
    close(reset_bl_pio_fd);
    return lcd_fd;
}
/**
 * @brief reset_lcd
 * @param reset_fd opened pio for reset pin
 * @param pin should init before
 * @return
 */
int reset_lcd(int reset_fd,u8 pin){
    /* Apply hardware reset according to procedure indicated in FRD154BP2901 documentation */
    ioctl(reset_fd, I_PIO_CLRMASK, 1<<pin);
    usleep(5000);   /* Reset signal asserted during 5ms  */
    ioctl(reset_fd, I_PIO_SETMASK, 1<<pin);
    usleep(10000);  /* Reset signal released during 10ms */
    ioctl(reset_fd, I_PIO_CLRMASK, 1<<pin);
    usleep(20000);  /* Reset signal asserted during 20ms */
    ioctl(reset_fd, I_PIO_SETMASK, 1<<pin);
    usleep(10000);  /* Reset signal released during 10ms */
    return 0;
}
/**
  * @brief  Reads the selected LCD Register.
  * @param  Command: command value (or register address as named in st7789h2 doc).
  * @retval Register Value.
  */
u8 lcd_read_reg(int lcd_fd,u8 command){
    emc_attr_t emc_attr;
  /* Send command */
    emc_attr.o_flags =EMC_FLAG_IS_AHB|EMC_FLAG_AHB_WRITE_REG;
    emc_attr.data_or_reg = command;
    ioctl(lcd_fd, I_EMC_SETATTR, &emc_attr);
  /* Read dummy data */
    u8 data;
    read(lcd_fd,&data,1);
  /* Read register value */
    read(lcd_fd,&data,1);
    return data;
}
/**
  * @brief  TouchScreen_Calibration_WaitForPressedState : wait until a particular press/depress action
  *         The function is managing anti-rebound : that is the awaited state when detected
  *         needs to be stable for a sufficient time (timeout time), otherwise a new sense to search
  *         for awaited state is performed. When awaited state is found and state is stable for timeout
  *         duration, the function is exited.
  * @param  uint8_t Pressed :  Awaited pressed state
  *         - Await touch (single/multiple) detection if Pressed == 1
  *         - Await no touch detection if Pressed == 0
  * @retval None
  */
static void ts_wait_for_press_state(uint8_t press_state,TS_StateTypeDef * TS_State){
    uint8_t  status = TS_OK;
    uint32_t exit_first_level = 0;  /* By default no exit request from first level while loop  */
    uint32_t exit_second_level = 0; /* By default no exit request from second level while loop */
    struct timespec now;
    u64 m_sec,m_sec_current;
    /* First level while loop entry */
    do  {
        /* reset exit second level while loop in case it was set */
        exit_second_level = 0;
        /* Sense of touch state from touch IC until get the awaited state in parameter 'Pressed' */
        status = ts_get_state(TS_State);
        if(status == TS_OK){
            if (((press_state == 0) && (TS_State->touchDetected == 0)) ||
                    ((press_state == 1) && ((TS_State->touchDetected == 1) || (TS_State->touchDetected == 2)))){
                /* Got awaited press state */
                /* Record in 'TimeStart' the time of awaited touch event for anti-rebound calculation */
                /* The state should persist for a minimum sufficient time */
                clock_gettime(CLOCK_REALTIME, &now);
                m_sec = (u64)now.tv_sec*1000 + (u64)now.tv_nsec/1000000;
#if DEBUG
                printf("start time %lu %lu %lu %lu \n",(u32)(m_sec>>32),(u32)(m_sec),now.tv_sec,now.tv_nsec);
#endif
                /* Is state of the touch changing ? */
                /* Second level while loop entry */
                do{
                    /* New sense of touch state from touch IC : to evaluate if state was stable */
                    status = ts_get_state(TS_State);
                    if(status == TS_OK){
                        /* Is there a state change compared since having found the awaited state ? */
                        clock_gettime(CLOCK_REALTIME, &now);
                        m_sec_current = (u64)now.tv_sec*1000 + (u64)now.tv_nsec/1000000;
                        if (((press_state == 0) && ((TS_State->touchDetected == 1) || (TS_State->touchDetected == 2))) ||
                                ((press_state == 1) && ((TS_State->touchDetected == 0)))){
                            /* Too rapid state change => anti-rebound management : restart first touch search */
#if DEBUG
                            printf("state change %lu %lu %lu %lu \n",(u32)(m_sec_current>>32),(u32)m_sec_current,now.tv_sec,now.tv_nsec);
#endif
                            exit_second_level = 1; /* exit request from second level while loop */
                        }else if (m_sec_current > (m_sec+100)){
                            /* State have not changed for the timeout duration (stable touch for 100 ms) */
                            /* This means the touch state is stable : can exit function */
                            /* found valid touch, exit both while levels */
#if DEBUG
                            printf("time expired %lu %lu %lu %lu \n",(u32)(m_sec_current>>32),(u32)m_sec_current,(u32)(m_sec>>32),(u32)(m_sec));
#endif
                            exit_second_level = 1;
                            exit_first_level  = 1;
                        }
                        /* Wait 10 ms before next sense of touch at next loop iteration */
                        LCD_IO_Delay(10);
                    } /* of if(status == TS_OK) */
                }
                while (!exit_second_level);
            } /* of if (((Pressed == 0) && .... */
        } /* of if(status == TS_OK) */
        if(!exit_first_level){
            /* Wait some time before next sense of touch at next loop iteration */
            LCD_IO_Delay(10);
        }
    } while (!exit_second_level);
}

#endif
