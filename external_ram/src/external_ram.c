#ifndef EXTERNAL_RAM_C_
#define EXTERNAL_RAM_C_
#include <stdio.h>
#include <string.h>
#include "external_ram.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"
#define SRAM_BANK_ADDR                 ((uint32_t)0x60000000)
#define SRAM_SIZE                      ((uint32_t)0x80000)
#define TEST_BUFFER_SIZE         ((uint32_t)250)


int external_ram(void){
    int fd;
    u8 test_buffer_read[TEST_BUFFER_SIZE];
    u8 test_buffer_write[TEST_BUFFER_SIZE];
    for (u16 i = 0;i<TEST_BUFFER_SIZE;i++){
        test_buffer_write[i] = (u8)i;
    }
    printf("external ram example c\n");
    fd = open("/dev/fmc_psram0", O_RDWR);
    /*with setting configuration*/
    if(fd<0){
        printf("failed fmc_psram0 open \n");
    }else{
        emc_attr_t emc_attr;
        printf("success fmc_psram0 open \n");
        memset(&emc_attr,0,sizeof(emc_attr_t));
        memset(&emc_attr.pin_assignment, 0xff, sizeof(emc_pin_assignment_t));
        emc_attr.pin_assignment.bl[0] = mcu_pin(4,0);
        emc_attr.pin_assignment.bl[1] = mcu_pin(4,1);
        emc_attr.pin_assignment.oe = mcu_pin(3,4);
        emc_attr.pin_assignment.we = mcu_pin(3,5);
        emc_attr.pin_assignment.ncs[0] = mcu_pin(3,7);
        emc_attr.pin_assignment.data[0] = mcu_pin(3,14);
        emc_attr.pin_assignment.data[1] = mcu_pin(3,15);
        emc_attr.pin_assignment.data[2] = mcu_pin(3,0);
        emc_attr.pin_assignment.data[3] = mcu_pin(3,1);
        emc_attr.pin_assignment.data[4] = mcu_pin(4,7);
        emc_attr.pin_assignment.data[5] = mcu_pin(4,8);
        emc_attr.pin_assignment.data[6] = mcu_pin(4,9);
        emc_attr.pin_assignment.data[7] = mcu_pin(4,10);
        emc_attr.pin_assignment.data[8] = mcu_pin(4,11);
        emc_attr.pin_assignment.data[9] = mcu_pin(4,12);
        emc_attr.pin_assignment.data[10] = mcu_pin(4,13);
        emc_attr.pin_assignment.data[11] = mcu_pin(4,14);
        emc_attr.pin_assignment.data[12] = mcu_pin(4,15);
        emc_attr.pin_assignment.data[13] = mcu_pin(3,8);
        emc_attr.pin_assignment.data[14] = mcu_pin(3,9);
        emc_attr.pin_assignment.data[15] = mcu_pin(3,10);
        emc_attr.pin_assignment.address[0] = mcu_pin(5,0);
        emc_attr.pin_assignment.address[1] = mcu_pin(5,1);
        emc_attr.pin_assignment.address[2] = mcu_pin(5,2);
        emc_attr.pin_assignment.address[3] = mcu_pin(5,3);
        emc_attr.pin_assignment.address[4] = mcu_pin(5,4);
        emc_attr.pin_assignment.address[5] = mcu_pin(5,5);
        emc_attr.pin_assignment.address[6] = mcu_pin(5,12);
        emc_attr.pin_assignment.address[7] = mcu_pin(5,13);
        emc_attr.pin_assignment.address[8] = mcu_pin(5,14);
        emc_attr.pin_assignment.address[9] = mcu_pin(5,15);
        emc_attr.pin_assignment.address[10] = mcu_pin(6,0);
        emc_attr.pin_assignment.address[11] = mcu_pin(6,1);
        emc_attr.pin_assignment.address[12] = mcu_pin(6,2);
        emc_attr.pin_assignment.address[13] = mcu_pin(6,3);
        emc_attr.pin_assignment.address[14] = mcu_pin(6,4);
        emc_attr.pin_assignment.address[15] = mcu_pin(6,5);
        emc_attr.pin_assignment.address[16] = mcu_pin(3,11);
        emc_attr.pin_assignment.address[17] = mcu_pin(3,12);
        /*first set att master*/
        emc_attr.size = SRAM_SIZE;
        emc_attr.base_address = SRAM_BANK_ADDR;
        emc_attr.o_flags = EMC_FLAG_IS_PSRAM|EMC_FLAG_ENABLE|EMC_FLAG_IS_PSRAM_BANK1;
        emc_attr.data_bus_width = 16;

        ioctl(fd, I_EMC_SETATTR, &emc_attr);
        printf("fmc enabled\n");
        lseek(fd,(int)0,SEEK_SET);
        write(fd,test_buffer_write,TEST_BUFFER_SIZE);
        read(fd,test_buffer_read,TEST_BUFFER_SIZE);
        u32 i=0;
        for (i=0;i<TEST_BUFFER_SIZE;i++){
            if (test_buffer_read[i]!=test_buffer_write[i]){
                break;
            }
        }
        if(i>=TEST_BUFFER_SIZE){
            printf ("passed equal %u %u\n",test_buffer_read[12],test_buffer_read[249]);
        }else{
            printf ("not equal %u %u \n",test_buffer_read[18],test_buffer_read[249]);
        }
        for (i=0;i<(SRAM_SIZE-TEST_BUFFER_SIZE);i+=TEST_BUFFER_SIZE){
            memset(&test_buffer_write,(u8)i,TEST_BUFFER_SIZE);
            memset(&test_buffer_read,(u8)0,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            write(fd,test_buffer_write,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            read(fd,test_buffer_read,TEST_BUFFER_SIZE);
            u32 j=0;
            for (j=0;j<TEST_BUFFER_SIZE;j++){
                if (test_buffer_read[j]!=test_buffer_write[j]){
                    printf("error verification %lu",j);
                    break;
                }
            }
            if(j<TEST_BUFFER_SIZE){
                printf("error verification %lu",j);
                break;
            }
        }
        u32 j=0;
        for (i=0;i<(SRAM_SIZE-TEST_BUFFER_SIZE);i+=TEST_BUFFER_SIZE){
            j=0;
            memset(&test_buffer_write,(u8)i,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            write(fd,test_buffer_write,TEST_BUFFER_SIZE);
            if(i==0){
                printf("value %u %u\n",(u8)i,test_buffer_write[0]);
            }
            if(i==TEST_BUFFER_SIZE){
                printf("value %u %u\n",(u8)i,test_buffer_write[0]);
            }

        }
        printf("value %lu \n",i);
        i-=TEST_BUFFER_SIZE;
        for (;i>0;i-=TEST_BUFFER_SIZE){
            lseek(fd,(int)i,SEEK_SET);
            read(fd,test_buffer_read,TEST_BUFFER_SIZE);
            j=0;
            for (j=0;j<TEST_BUFFER_SIZE;j++){
                if (test_buffer_read[j]!=(u8)i){
                    printf("error verification additional %lu %u %lu \n",j,test_buffer_read[j],i);
                    printf("error verification additional %u %u %u \n",test_buffer_read[j+2],test_buffer_read[j+1],test_buffer_read[j+3]);
                    break;
                }
            }
            if(j<TEST_BUFFER_SIZE){
                printf("error verification additional %lu \n",j);
                break;
            }
        }

        printf("fmc_psram0 tested - %lu ram space from - %lu \n", i,(SRAM_SIZE-TEST_BUFFER_SIZE));
        if(close(fd)<0){
            printf("failed fmc_psram0 closing\n");
        }else{
            printf("fmc_psram0 closed\n");
        }
    }
    /*use default board configuration*/
    fd = open("/dev/fmc_psram0", O_RDWR);
    if(fd<0){
        printf("failed fmc_psram0 open \n");
    }else{
        printf("success fmc_psram0 open \n");
        /*first set att master*/
        ioctl(fd, I_EMC_SETATTR, NULL);
        printf("fmc enabled\n");
        lseek(fd,(int)0,SEEK_SET);
        memset(&test_buffer_write,(u8)33,TEST_BUFFER_SIZE);
        write(fd,test_buffer_write,TEST_BUFFER_SIZE);
        read(fd,test_buffer_read,TEST_BUFFER_SIZE);
        u32 i=0;
        for (i=0;i<TEST_BUFFER_SIZE;i++){
            if (test_buffer_read[i]!=test_buffer_write[i]){
                break;
            }
        }
        if(i>=TEST_BUFFER_SIZE){
            printf ("passed equal %u %u\n",test_buffer_read[12],test_buffer_read[249]);
        }else{
            printf ("not equal %u %u \n",test_buffer_read[18],test_buffer_read[249]);
        }
        for (i=0;i<(SRAM_SIZE-TEST_BUFFER_SIZE);i+=TEST_BUFFER_SIZE){
            memset(&test_buffer_write,(u8)i,TEST_BUFFER_SIZE);
            memset(&test_buffer_read,(u8)0,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            write(fd,test_buffer_write,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            read(fd,test_buffer_read,TEST_BUFFER_SIZE);
            u32 j=0;
            for (j=0;j<TEST_BUFFER_SIZE;j++){
                if (test_buffer_read[j]!=test_buffer_write[j]){
                    printf("error verification %lu",j);
                    break;
                }
            }
            if(j<TEST_BUFFER_SIZE){
                printf("error verification %lu\n",j);
                break;
            }
        }
        u32 j=0;
        for (i=0;i<(SRAM_SIZE-TEST_BUFFER_SIZE);i+=TEST_BUFFER_SIZE){
            j=0;
            memset(&test_buffer_write,(u8)i,TEST_BUFFER_SIZE);
            lseek(fd,(int)i,SEEK_SET);
            write(fd,test_buffer_write,TEST_BUFFER_SIZE);
            if(i==0){
                printf("vaule %u %u\n",(u8)i,test_buffer_write[0]);
            }
        }
        printf("vaule %u \n",(u8)i);
        for (i=0;i<(SRAM_SIZE-TEST_BUFFER_SIZE);i+=TEST_BUFFER_SIZE){
            lseek(fd,(int)i,SEEK_SET);
            read(fd,test_buffer_read,TEST_BUFFER_SIZE);
            j=0;
            for (j=0;j<TEST_BUFFER_SIZE;j++){
                if (test_buffer_read[j]!=(u8)i){
                    printf("error verification additional %lu %u %u \n",j,test_buffer_read[j],(u8)i);
                    printf("error verification additional %u %u %u \n",test_buffer_read[j+2],test_buffer_read[j+1],test_buffer_read[j+3]);
                    break;
                }
            }
            if(j<TEST_BUFFER_SIZE){
                printf("error verification additional %lu \n",j);
                break;
            }
        }

        printf("fmc_psram0 tested - %lu ram space from - %lu \n", i,(SRAM_SIZE-TEST_BUFFER_SIZE));
        if(close(fd)<0){
            printf("failed fmc_psram0 closing\n");
        }else{
            printf("fmc_psram0 closed\n");
        }
    }

    return fd;
}
#endif
