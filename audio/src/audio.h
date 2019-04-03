#ifndef _AUDIO__H_
#define _AUDIO__H_

#if defined __cplusplus
extern "C" {
#endif
#include <trace.h>
#define WM8994_I2C_ADDRESS                  ((u16)0x34)
#define WM8994_CHIPID_ADDR                  0x00
#define WM8994_ID                           0x8994
#define DISCOVERY_I2Cx_TIMING               ((uint32_t)0x40912732)
int audio_init(int fd);
u16 audio_read_u16(int fd,u8 dev_addr, u16 reg_addr);
int audio_write_u16(int fd,u8 dev_addr, u16 reg_addr,u16 value);
#if defined __cplusplus
}
#endif
#endif
