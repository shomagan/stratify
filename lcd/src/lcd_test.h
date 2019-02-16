#ifndef LCD_TEST_H_
#define LCD_TEST_H_

#if defined __cplusplus
extern "C" {
#endif
#include <trace.h>
int lcd_test(void);
int reset_lcd(int reset_fd,u8 pin);
u8 lcd_read_reg(int lcd_fd,u8 command);
#if defined __cplusplus
}
#endif

#endif
