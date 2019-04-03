#ifndef _AUDIO__C_
#define _AUDIO__C_

#include "audio.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"


u16 audio_read_u16(int fd,u8 dev_addr, u16 reg_addr){
    i2c_attr_t i2c_attr;
    u16 data;
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    i2c_attr.o_flags = I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_IS_PTR_16;
    i2c_attr.slave_addr[0].addr16 = dev_addr>>1;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    reg_addr = ((reg_addr>>8)&0x00ff)|((reg_addr<<8)&0xff00);
    lseek(fd,(int)reg_addr,SEEK_SET);
    read(fd,&data,2);
    data = ((data>>8)&0x00ff)|((data<<8)&0xff00);
    return data;
}
int audio_write_u16(int fd,u8 dev_addr, u16 reg_addr,u16 value){
    i2c_attr_t i2c_attr;
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    i2c_attr.o_flags = I2C_FLAG_PREPARE_PTR_DATA|I2C_FLAG_IS_PTR_16;
    i2c_attr.slave_addr[0].addr16 = dev_addr>>1;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);
    lseek(fd,(int)reg_addr,SEEK_SET);
    value = ((value>>8)&0x00ff)|((value<<8)&0xff00);
    return write(fd,&value,2);
}

int audio_init(int fd){
    i2c_attr_t i2c_attr;
    if(fd<0){
        fd = open("/dev/i2c0", O_RDWR);
    }
    memset(&i2c_attr,0,sizeof(i2c_attr_t));
    memset(&i2c_attr.pin_assignment, 0xff, sizeof(i2c_pin_assignment_t));
    i2c_attr.freq =DISCOVERY_I2Cx_TIMING;
    i2c_attr.slave_addr[0].addr16 = WM8994_I2C_ADDRESS>>1;
    i2c_attr.o_flags = I2C_FLAG_SET_MASTER|I2C_FLAG_STRETCH_CLOCK;
    ioctl(fd, I_I2C_SETATTR, &i2c_attr);

    return fd;
}

#endif
