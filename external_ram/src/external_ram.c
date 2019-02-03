#ifndef QSPI_TEST_C_
#define QSPI_TEST_C_
#include <stdio.h>
#include <string.h>
#include "external_ram.h"
#include "sos/dev/emc.h"
#include "sos/sos.h"
#define SRAM_BANK_ADDR                 ((uint32_t)0x60000000)
#define SRAM_SIZE                      ((uint32_t)0x100000)
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
        /*first set att master*/
        emc_attr.size = SRAM_SIZE;
        emc_attr.base_address = SRAM_BANK_ADDR;
        emc_attr.o_flags = EMC_FLAG_ENABLE;
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
