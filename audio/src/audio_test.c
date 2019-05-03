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
static u8 volume = 15;
static u8 thread_is_execute = 1;
static u32 event_out=0;
#define SRAM_SIZE                      (0x80000)
#define AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD                  8000  /* buffer size in half-word */
static uint16_t buff_in[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD];
static uint16_t buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD];
static u16 test_position = 0;
#define I2S_AUDIOFREQ_48K                ((uint32_t)48000U)
#define I2S_AUDIOFREQ_16K                ((uint32_t)16000U)
#define DEFAULT_AUDIO_IN_CHANNEL_NBR 2
#define BIT(x) (1<<x)
#define DEFAULT_TIME_REC                      30  /* Recording time in second (default: 30s) */
#define TEST_OUTPUT 0
static int fd_psram,position = 0;
static uint32_t freq = I2S_AUDIOFREQ_16K;
int audio_test(void){
    int res = 0;
    int fd,fd_sai_out,fd_sai_in;
    u16 data;
    sai_attr_t sai_out,sai_in;
    mcu_action_t action ;
    u32 write_reg ;
    u8 read_enable = 1;
    pthread_t t;
    pthread_attr_t pattr;
    if ( pthread_attr_init(&pattr) < 0 ){
        fflush(stdout);
        printf("attr_init failed\n");
    }
    if ( pthread_attr_setstacksize(&pattr, 2048) < 0 ){
        fflush(stdout);
        printf("setstacksize failed\n");
    }
    if ( pthread_create(&t, &pattr, out_buffer_thread_control, NULL) < 0 ){
        printf("create failed\n");
    }
    struct timespec now;
    u32 m_sec,m_sec_current;
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
        sai_out.freq = freq;
        sai_out.mck_mult = 1;
        sai_out.o_flags = I2S_FLAG_SET_MASTER|I2S_FLAG_IS_RECEIVER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE;//dma disabled
        sai_out.slot = BIT(0) |BIT(1) |BIT(2) | BIT(3);
        sai_in.o_flags = I2S_FLAG_SET_SLAVE|I2S_FLAG_IS_RECEIVER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE|SAI_FLAG_IS_SYNCHRONOUS;
        sai_in.freq = freq;
        sai_in.mck_mult = 1;
        sai_in.slot =  BIT(0) |BIT(1) |BIT(2) | BIT(3);
        ioctl(fd_sai_out, I_I2S_SETATTR, &sai_out); /*init current config*/
        ioctl(fd_sai_in, I_I2S_SETATTR, &sai_in); /*init current config*/
        data = wm8994_ReadID(fd,WM8994_I2C_ADDRESS);
        printf("id read 0x%02x \n",data);
        if(data == WM8994_ID){
            wm8994_Reset(WM8994_I2C_ADDRESS);
            volume = 125;
            write_reg = wm8994_Init(WM8994_I2C_ADDRESS, INPUT_DEVICE_DIGITAL_MIC1_MIC2, volume, freq);
            ioctl(fd_sai_in,I_MCU_SETACTION, &action);
            printf("fmc enabled\n");
            lseek(fd_psram,(int)0,SEEK_SET);
            memset(buff_in,0,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
            printf("clear buff\n");
            position = 0;
            clock_gettime(CLOCK_REALTIME, &now);
            m_sec = (u32)now.tv_sec*1000 + (u32)now.tv_nsec/1000000;
            read(fd_sai_in,buff_in,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD * 2);
            clock_gettime(CLOCK_REALTIME, &now);
            m_sec_current = (u32)now.tv_sec*1000 + (u32)now.tv_nsec/1000000;
            printf("cancel recording %u ,time  ms %u\n",position , (m_sec_current - m_sec) );
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
        action.channel = 2;
        action.handler.callback = sai_out_cb;
        action.handler.context = NULL;
        memset(&sai_out,0,sizeof(sai_attr_t));
        memset(&sai_out.pin_assignment,0xff,sizeof(sai_pin_assignment_t));
        sai_out.freq = freq;
        sai_out.mck_mult = 1;
        sai_out.o_flags = I2S_FLAG_SET_MASTER|I2S_FLAG_IS_TRANSMITTER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
            SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE;//dma disabled
        sai_out.slot = BIT(0)|BIT(1)|BIT(2)|BIT(3);
        ioctl(fd_sai_out, I_I2S_SETATTR, &sai_out); /*init current config*/
        wm8994_Reset(WM8994_I2C_ADDRESS);
        data = wm8994_ReadID(fd,WM8994_I2C_ADDRESS);
        printf("id read 0x%02x \n",data);
        volume = 20;
        write_reg = wm8994_Init(WM8994_I2C_ADDRESS, OUTPUT_DEVICE_BOTH, volume, freq);
        wm8994_Play(WM8994_I2C_ADDRESS,buff_out,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        /*start playing*/
        ioctl(fd_sai_out,I_MCU_SETACTION, &action);
#if TEST_OUTPUT
#define TEST_PERIOD        8    // 2000Hz
#define TEST_PERIOD_2        16    // 1000Hz
#define TEST_PERIOD_4        32    // 500Hz
#define TEST_PERIOD_8        64    // 250Hz
#define TEST_PERIOD_16        128    // 125Hz
#define MAX_VALUE          10000
        int period = TEST_PERIOD;

        for (int j=0; j<SRAM_SIZE;j+=AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD){
            for(int i=0;i<AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;i++){
                if(((i % period) == 0) || ((i % period) == period/2)){
                    buff_out[i] = MAX_VALUE/2;
                }else if(((i % period) == period/4)){
                    buff_out[i] = MAX_VALUE;
                }else if(((i % period) == (period/4)*3)){
                    buff_out[i] = 0;
                }else{
                    buff_out[i] = buff_out[i-1];
                }
            }
            lseek(fd_psram,(int)position,SEEK_SET);
            write(fd_psram,&buff_out[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
            position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
            switch(period){
            case(TEST_PERIOD):
                period = TEST_PERIOD_2;
                break;
            case(TEST_PERIOD_2):
                period = TEST_PERIOD_4;
                break;
            case(TEST_PERIOD_4):
                period = TEST_PERIOD_8;
                break;
            case(TEST_PERIOD_8):
                period = TEST_PERIOD_16;
                break;
            case(TEST_PERIOD_16):
                period = TEST_PERIOD;
                break;
            default:
                period = TEST_PERIOD;
                break;
            }
        }
        position = 0;
#endif
        position = 0;
        lseek(fd_psram,(int)position,SEEK_SET);
        read(fd_psram,&buff_out[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
        thread_is_execute = 2;
        clock_gettime(CLOCK_REALTIME, &now);
        m_sec = (u32)now.tv_sec*1000 + (u32)now.tv_nsec/1000000;
        write(fd_sai_out,buff_out,AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD*2);
        clock_gettime(CLOCK_REALTIME, &now);
        m_sec_current = (u32)now.tv_sec*1000 + (u32)now.tv_nsec/1000000;
        printf("cancel playing %u %u, time ms %u \n",test_position,position,(m_sec_current - m_sec));
    }
    close(fd_sai_out);
    close(fd);
    printf("fd have closed\n");
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
            event_out |= MCU_EVENT_FLAG_HALF_TRANSFER;
            res = 1;
        }else{
            res = 0;
        }
    }else if(event->o_events & MCU_EVENT_FLAG_DATA_READY){
        if(position<(int)SRAM_SIZE){
            event_out |= MCU_EVENT_FLAG_WRITE_COMPLETE;
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
            res = 1;
        }else{
            res = 0;
        }
    }else if(event->o_events & MCU_EVENT_FLAG_WRITE_COMPLETE){
        if(position < (int)SRAM_SIZE){
            event_out |= MCU_EVENT_FLAG_WRITE_COMPLETE;
            res = 1;
        }else{
            res = 0;
        }
    }
    return res;
}

void * out_buffer_thread_control(void * args){
    printf("start out_buffer_thread_control %u...\n",thread_is_execute);
    while(thread_is_execute){
        if (thread_is_execute == 1){//read
            if (event_out & MCU_EVENT_FLAG_HALF_TRANSFER){
                event_out = 0;
                if(position < (int)SRAM_SIZE){
                    lseek(fd_psram,(int)position,SEEK_SET);
                    write(fd_psram,&buff_in[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                    position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
                }
            }else if(event_out & MCU_EVENT_FLAG_WRITE_COMPLETE){
                event_out = 0;
                if(position < (int)SRAM_SIZE){
                    lseek(fd_psram,(int)position,SEEK_SET);
                    write(fd_psram,&buff_in[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                    position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
                }
            }
        }else{
            if (event_out & MCU_EVENT_FLAG_HALF_TRANSFER){
                event_out = 0;
                if(position < (int)SRAM_SIZE){
                    lseek(fd_psram,(int)position,SEEK_SET);
                    read(fd_psram,&buff_out[0],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                    position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
                }else{
                    break;
                }
            }else if(event_out & MCU_EVENT_FLAG_WRITE_COMPLETE){
                event_out = 0;
                if(position < (int)SRAM_SIZE){
                    lseek(fd_psram,(int)position,SEEK_SET);
                    read(fd_psram,&buff_out[AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD/2],AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD);
                    position += AUDIO_IN_PCM_BUFFER_SIZE_IN_HALF_WORD;//AUDIO_IN_PCM_BUFFER_SIZE is meuserment in half word (u16 = byte/2)
                }else{
                    break;
                }
            }
        }
        sched_yield();
    }
    return NULL;
}


#endif
