#ifndef LCD_C_
#define LCD_C_

#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"
#include "st7789h2.h"

/* LCD IO functions */
int lcd_io_init(int fd);
void     lcd_io_write_multiple_data(int fd,u16 *data, u32 size);
void     lcd_io_write_reg(int fd,u8 reg);
void     lcd_io_write_data(int fd,u16 data);
uint16_t lcd_io_read_data(int fd);
void     lcd_delay(u32 delay);
static void lcd_char(uint16_t x, uint16_t y, const uint8_t *c);

int  lcd_io_init(int fd){
    emc_attr_t emc_attr;
  /* Send command */
    memset(&emc_attr,0,sizeof(emc_attr_t));
    memset(&emc_attr.pin_assignment, 0xff, sizeof(emc_pin_assignment_t));
    emc_attr.o_flags = EMC_FLAG_ENABLE|EMC_FLAG_IS_AHB|EMC_FLAG_IS_PSRAM_BANK2;
    emc_attr.data_bus_width = 16;
    return ioctl(fd, I_EMC_SETATTR, &emc_attr);
}
void lcd_io_write_multiple_data(int fd,u16 *data, u32 size){
    for (u32 i = 0; i < size; i++){
      /* Send command */
        write(fd,&data[i],2);
    }
}
void lcd_io_write_reg(int fd,u8 reg){
    emc_attr_t emc_attr;
  /* Send command */
    emc_attr.o_flags =EMC_FLAG_IS_AHB|EMC_FLAG_AHB_WRITE_REG;
    emc_attr.data_or_reg = reg;
    ioctl(fd, I_EMC_SETATTR, &emc_attr);
}
void  lcd_io_write_data(int fd,u16 data){
    write(fd,&data,2);
}
uint16_t lcd_io_read_data(int fd){
    u16 data;
    read(fd,&data,2);
    return data;
}
void     lcd_delay(u32 delay){
    usleep(delay*1000);
}
void lcd_draw_char(uint16_t x, uint16_t y, uint8_t val){
    lcd_char(x, y, &Font24.table[(val - ' ') *\
      Font24.Height * ((Font24.Width + 7) / 8)]);
}
static void lcd_char(uint16_t x, uint16_t y, const uint8_t *c){
    uint32_t i = 0, j = 0;
    uint16_t height, width;
    uint8_t offset;
    uint8_t *pchar;
    uint32_t line;
    height = Font24.Height;
    width  = Font24.Width;
    offset =  8 *((width + 7)/8) -  width ;
    for(i = 0; i < height; i++)  {
        pchar = ((uint8_t *)c + (width + 7)/8 * i);
        switch(((width + 7)/8))    {
        case 1:
            line =  pchar[0];
            break;
        case 2:
            line =  (pchar[0]<< 8) | pchar[1];
            break;
        case 3:
        default:
            line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
            break;
        }
        for (j = 0; j < width; j++){
            if(line & (1 << (width- j + offset- 1))) {
                ST7789H2_WritePixel((x + j), y, LCD_COLOR_BLUE);
            }else{
                ST7789H2_WritePixel((x + j), y, LCD_COLOR_BLACK);
            }
        }
        y++;
    }
}

#endif
