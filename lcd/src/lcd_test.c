#ifndef LCD_TEST_C_
#define LCD_TEST_C_
#include <stdio.h>
#include <string.h>
#include "lcd_test.h"
#include "lcd.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"
#include "st7789h2.h"
#define FMC_BANK2_BASE  ((uint32_t)(0x60000000 | 0x04000000))  
#define FMC_BANK2       ((LCD_CONTROLLER_TypeDef *) FMC_BANK2_BASE)
#define SOS_BOARD_LCD_BL_PORT 7 //PH11
#define SOS_BOARD_LCD_BL_PIN 11
#define SOS_BOARD_LCD_TE_INT_PORT 2 //PC8
#define SOS_BOARD_LCD_TE_INT_PIN 8
#define SOS_BOARD_LCD_RST_PORT 7 //PH7
#define SOS_BOARD_LCD_RST_PIN 7
#define SOS_BOARD_LED6_PIN 1
#define SOS_BOARD_LED5_PIN 7
#define ST7789H2_LCD_ID             0x04
int lcd_fd =-1;
int lcd_test(void){
    u16 size_x, size_y;
    size_x=0; size_y=0;
    printf("lcd example c\n");
    int reset_bl_pio_fd = open("/dev/pio7",O_RDWR);
    /*int te_pio_fd = open("/dev/pio2",O_RDWR);
    int led6_pio_fd = open("/dev/pio1",O_RDWR);
    int led5_pio_fd = open("/dev/pio0",O_RDWR);*/
    pio_attr_t pio_attr;
    u8 id;
    pio_attr.o_pinmask = (1<<SOS_BOARD_LCD_BL_PIN)|(1<<SOS_BOARD_LCD_RST_PIN);
    pio_attr.o_flags = PIO_FLAG_IS_SPEED_LOW | PIO_FLAG_SET_OUTPUT;
    ioctl(reset_bl_pio_fd, I_PIO_SETATTR, &pio_attr);
    ioctl(reset_bl_pio_fd, I_PIO_SETMASK, (1<<SOS_BOARD_LCD_BL_PIN));
    printf("bl set\n");
    printf("lcd reset\n");

    lcd_fd = open("/dev/lcd0", O_RDWR);
    /*with setting configuration*/
    if(lcd_fd<0){
        printf("lcd dev have't opened c\n");
    }else{
        printf("lcd dev have opened c\n");
        reset_lcd(reset_bl_pio_fd,SOS_BOARD_LCD_RST_PIN);
        ST7789H2_Init();
        printf("display init\n");
        id = (u8)ST7789H2_ReadID();
        printf("id %u \n",id);
        if(id ==ST7789H2_ID){
            ST7789H2_SetOrientation(LCD_ORIENTATION_PORTRAIT);
            size_y = ST7789H2_GetLcdPixelHeight();
            size_x = ST7789H2_GetLcdPixelWidth();
            printf("x - %u, y - %u \n",size_y,size_x);
            for (u16 i=0;i<size_y;i++){
                ST7789H2_DrawHLine(LCD_COLOR_BLACK,0,i,size_x);
            }
            u8 y = 0;
            lcd_draw_char(size_x/2, y, 's');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 't');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'r');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'a');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 't');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'i');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'f');
            y += Font24.Height;
            lcd_draw_char(size_x/2, y, 'y');
            y += Font24.Height;
            sleep(3);
        }
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
    sleep(4);
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
    ioctl(reset_fd, I_PIO_CLRMASK, (1<<pin));
    usleep(5000);   /* Reset signal asserted during 5ms  */
    ioctl(reset_fd, I_PIO_SETMASK, (1<<pin));
    usleep(10000);  /* Reset signal released during 10ms */
    ioctl(reset_fd, I_PIO_CLRMASK, (1<<pin));
    usleep(20000);  /* Reset signal asserted during 20ms */
    ioctl(reset_fd, I_PIO_SETMASK, (1<<pin));
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

#endif
