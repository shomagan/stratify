#ifndef LCD_TEST_C_
#define LCD_TEST_C_
#include <stdio.h>
#include <string.h>
#include "audio.h"
#include "audio_test.h"
#include "wm8994.h"
#include "sos/dev/i2c.h"
#include "sos/dev/i2s.h"
#include "sos/sos.h"
#include "sos/dev/emc.h"
static int sai_in_cb(void * data, const mcu_event_t * event);
static int sai_out_cb(void * data, const mcu_event_t * event);
static void * out_buffer_thread_control(void * args);
static u32 volume = 15;
static u8 thread_is_execute = 1;
static u32 event_out=0;
#define SRAM_SIZE                      (0x80000)
#define AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD                  8192  /* buffer size in half-word */
uint16_t buff_in[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD];
uint16_t buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD];
u16 test_position = 0;
#define I2S_AUDIOFREQ_48K                ((uint32_t)48000U)
#define I2S_AUDIOFREQ_16K                ((uint32_t)16000U)
#define DEFAULT_AUDIO_IN_CHANNEL_NBR 2
#define BIT(x) (1<<x)
#define DEFAULT_AUDIO_IN_FREQ I2S_AUDIOFREQ_16K
#define DEFAULT_TIME_REC                      30  /* Recording time in second (default: 30s) */
#define REC_SAMPLE_LENGTH   (DEFAULT_TIME_REC * DEFAULT_AUDIO_IN_FREQ * DEFAULT_AUDIO_IN_CHANNEL_NBR * 2)

static int fd_psram,position = 0;
int audio_test(void){
    int res = 0;
    int fd,fd_sai_out,fd_sai_in;
    u16 data;
    sai_attr_t sai_out,sai_in;
    mcu_action_t action ;
    u32 write_reg ;
    u8 read_enable = 1;
    fd_sai_out = open("/dev/sai2", O_RDWR);
    fd_sai_in = open("/dev/sai3", O_RDWR);
    fd = open("/dev/i2c0", O_RDWR);
    fd_psram = open("/dev/fmc_psram0", O_RDWR);
    ioctl(fd_psram, I_EMC_SETATTR, NULL);
    if(fd<0 || fd_sai_out<0 || fd_sai_in<0 || fd_psram<0){
        res =-1;
        printf("open have failed \n");
    }else if(read_enable){
        action.channel = 3;
        action.handler.callback = sai_in_cb;
        action.handler.context = NULL;
        memset(&sai_out,0,sizeof(sai_attr_t));
        memset(&sai_in,0,sizeof(sai_attr_t));
        memset(&sai_out.pin_assignment,0xff,sizeof(sai_pin_assignment_t));
        memset(&sai_in.pin_assignment,0xff,sizeof(sai_pin_assignment_t));
        sai_out.freq = 16000;
        sai_out.mck_mult = 1;
        sai_out.o_flags = I2S_FLAG_SET_MASTER|I2S_FLAG_IS_RECEIVER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE;//dma disabled
        sai_out.slot = BIT(0) |BIT(1) |BIT(2) | BIT(3);
        sai_in.o_flags = I2S_FLAG_SET_SLAVE|I2S_FLAG_IS_RECEIVER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE|SAI_FLAG_IS_SYNCHRONOUS;
        sai_in.freq = 16000;
        sai_in.mck_mult = 1;
        sai_in.slot =  BIT(0) |BIT(1) |BIT(2) | BIT(3);
        ioctl(fd_sai_out, I_I2S_SETATTR, &sai_out); /*init current config*/
        ioctl(fd_sai_in, I_I2S_SETATTR, &sai_in); /*init current config*/
        data = wm8994_ReadID(fd,WM8994_I2C_ADDRESS);
        printf("id read 0x%02x \n",data);
        if(data == WM8994_ID){
            wm8994_Reset(WM8994_I2C_ADDRESS);
            volume = 50;
            write_reg = wm8994_Init(WM8994_I2C_ADDRESS, INPUT_DEVICE_DIGITAL_MIC1_MIC2, volume, I2S_AUDIOFREQ_16K);
            printf("write %lu", write_reg);
            ioctl(fd_sai_in,I_MCU_SETACTION, &action);
            printf("fmc enabled\n");

            lseek(fd_psram,(int)0,SEEK_SET);
            memset(buff_in,0,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
            printf("clear buff\n");
            for (int i =0;i<10;i++){
                lseek(fd_psram,(int)i*AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2,SEEK_SET);
                write(fd_psram,buff_in,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
            }
            printf("start recording\n");
            position = 0;
            read(fd_sai_in,buff_in,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD * 2);
           /* for (i =0; i<SRAM_SIZE ; i+=(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2)){
                lseek(fd_psram,(int)i,SEEK_SET);
                read(fd_psram,buff_in,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD * 2);
                for (int j=0;j<10;j++){
                    printf("%u - %u \n",j+i,buff_in[j]);
                }
            }*/
            wm8994_Stop(WM8994_I2C_ADDRESS,CODEC_PDWN_SW);
        }
    }
    close(fd_sai_in);
    close(fd_sai_out);
    fd_sai_out = open("/dev/sai2", O_RDWR);
    if(fd<0 || fd_sai_out<0 || fd_sai_in<0 || fd_psram<0){
        res =-1;
        printf("open have failed \n");
    }else{
        pthread_mutexattr_t attr;
        pthread_t t;
        pthread_attr_t pattr;
        if ( pthread_attr_init(&pattr) < 0 ){
            fflush(stdout);
            printf("attr_init failed");
        }
        if ( pthread_attr_setstacksize(&pattr, 2048) < 0 ){
            fflush(stdout);
            printf("setstacksize failed");
        }
        if ( pthread_create(&t, &pattr, out_buffer_thread_control, NULL) < 0 ){
            printf("create failed");
        }

        action.channel = 2;
        action.handler.callback = sai_out_cb;
        action.handler.context = NULL;
        printf("callback %p",action.handler.callback);
        memset(&sai_out,0,sizeof(sai_attr_t));
        memset(&sai_out.pin_assignment,0xff,sizeof(sai_pin_assignment_t));
        sai_out.freq = 16000;
        sai_out.mck_mult = 1;
        sai_out.o_flags = I2S_FLAG_SET_MASTER|I2S_FLAG_IS_TRANSMITTER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE;//dma disabled
        sai_out.slot = BIT(0)|BIT(1)|BIT(2)|BIT(3);
        ioctl(fd_sai_out, I_I2S_SETATTR, &sai_out); /*init current config*/
        sai_out.o_flags = SAI_FLAG_SET_SLOT;//dma disabled
        sai_out.slot = BIT(0)|BIT(2);
        ioctl(fd_sai_out, I_I2S_SETATTR, &sai_out); /*init current config*/
        wm8994_Reset(WM8994_I2C_ADDRESS);
        data = wm8994_ReadID(fd,WM8994_I2C_ADDRESS);
        printf("id read 0x%02x \n",data);
        volume = 15;
        write_reg = wm8994_Init(WM8994_I2C_ADDRESS, OUTPUT_DEVICE_BOTH, volume, I2S_AUDIOFREQ_16K);
        wm8994_Play(WM8994_I2C_ADDRESS,buff_out,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        /*start playing*/
        ioctl(fd_sai_out,I_MCU_SETACTION, &action);
        printf("start playing\n");
        position = 0;
        lseek(fd_psram,(int)position,SEEK_SET);
        read(fd_psram,&buff_out[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
        memset(buff_in,0,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        write(fd_sai_out,buff_out,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        printf("test position %u %u\n",test_position,position);
        /*for (int i=0;i<AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2;i++){
            printf("%u - %u\n",buff_in[i],buff_in[i+AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2]);
        }*/
        printf("WRITE passed\n");
        /*int i;
        for (i =0; i<SRAM_SIZE ; i+=(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2)){
            lseek(fd_psram,(int)i,SEEK_SET);
            read(fd_psram,buff_in,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD * 2);
            for (int j=0;j<100;j++){
                printf("%u - %u \n",j+i,buff_in[j]);
            }
        }*/

        //sleep(1);
    }
    close(fd_sai_out);
    close(fd);
    printf("fd have closed");
    return res;
}
/**
 * @brief sai_in_cb tho step handing ,
 *          1 - half receive
 *          2 - full receive data
 *return > 0 process will be continue
 */
int sai_in_cb(void * data, const mcu_event_t * event){
    int res = 1;
    if (event->o_events & MCU_EVENT_FLAG_HALF_TRANSFER){
        if(position < (int)SRAM_SIZE){
            lseek(fd_psram,(int)position,SEEK_SET);
            write(fd_psram,&buff_in[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
            position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
            res = 1;
        }else{
            res=0;
        }
    }else if(event->o_events & MCU_EVENT_FLAG_DATA_READY){
        if(position<(int)SRAM_SIZE){
            lseek(fd_psram,(int)position,SEEK_SET);
            write(fd_psram,&buff_in[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
            position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
            res = 1;
        }else{
            res = 0;
        }
    }
    return res;
}
/**
 * @brief sai_in_cb tho step handing ,
 *          1 - half receive
 *          2 - full receive data
 *return > 0 process will be continue
 */
int sai_out_cb(void * data, const mcu_event_t * event){
    int res = 1;
    if (event->o_events & MCU_EVENT_FLAG_HALF_TRANSFER){
        if(position < (int)SRAM_SIZE){
            event_out |= MCU_EVENT_FLAG_HALF_TRANSFER;
            position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
            res = 1;
        }else{
            res=0;
        }
    }else if(event->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
        if(position < (int)SRAM_SIZE){
            event_out |= MCU_EVENT_FLAG_WRITE_COMPLETE;
            position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
            res = 1;
        }else{
            res=0;
        }
    }
    return res;
}

void * out_buffer_thread_control(void * args){
    int * t = (int*)args;

    printf("start out_buffer_thread_control %u...",thread_is_execute);
    while(thread_is_execute){
        if (event_out & MCU_EVENT_FLAG_HALF_TRANSFER){
            event_out = 0;
            if(position < (int)SRAM_SIZE){
                lseek(fd_psram,(int)position,SEEK_SET);
                read(fd_psram,&buff_out[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                for(u16 i =0;i<(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2);i++){
                    if((buff_out[i]>0) && (test_position<(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2))){
                        buff_in[test_position] = position + i;
                        buff_in[test_position+AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2] = buff_out[+i];
                        test_position++;
                    }
                }
            }else{
                break;
            }
        }else if(event_out & MCU_EVENT_FLAG_WRITE_COMPLETE){
            event_out = 0;
            if(position < (int)SRAM_SIZE){
                lseek(fd_psram,(int)position,SEEK_SET);
                read(fd_psram,&buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                for(u16 i =0;i<(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2);i++){
                    if((buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2+i]>0) && (test_position<(AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2))){
                        buff_in[test_position] = position + i;
                        buff_in[test_position+AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2] = buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2+i];
                        test_position++;
                    }
                }
            }else{
                break;
            }
        }
        sched_yield();
    }
    return NULL;
}


#endif
