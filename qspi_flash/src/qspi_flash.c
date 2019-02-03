#ifndef QSPI_TEST_C_
#define QSPI_TEST_C_
#include <stdio.h>
#include <string.h>
#include "qspi_flash.h"
#include "sos/dev/qspi.h"
#include "sos/sos.h"

#define FLASH_TEST_ADDRESS  ((uint32_t)0x0000)
#define TEST_BUFFER_SIZE         ((uint32_t)258)

int qspi_flash(void){
    printf("qspi example c\n");
    int fd;
    u8 test_buffer_read[TEST_BUFFER_SIZE];
    u8 test_buffer_write[TEST_BUFFER_SIZE];
    for (u16 i = 0;i<TEST_BUFFER_SIZE;i++){
        test_buffer_write[i] = (u8)i;
    }
    /*use wit default value*/
    fd = open("/dev/qspi0", O_RDWR);
    if(fd<0){
        printf("Failed qspi open \n");
    }else{
        printf("qspi opened \n");
        /*preinit settings attribute for qspi*/
        ioctl(fd, I_QSPI_SETATTR, NULL);
        printf("qspi set attr with def value\n");
        external_flash_set_qspi_mode(fd);
        qspi_enter_four_bytes_address(fd);
        qspi_set_dummy_cycles_and_strenght(fd,QSPI_DUMMY_CYCLES_READ_QUAD,QSPI_CR_ODS_15);
        qspi_erase_block(fd,0);
        qspi_erase_block(fd,256);
        external_flash_read(fd,0,test_buffer_read,TEST_BUFFER_SIZE);
        external_flash_write(fd,0,test_buffer_write,TEST_BUFFER_SIZE);
        external_flash_read(fd,0,test_buffer_read,TEST_BUFFER_SIZE);
        u16 i=0;
        for (i=0;i<TEST_BUFFER_SIZE;i++){
            if (test_buffer_read[i]!=test_buffer_write[i]){
                break;
            }
        }
        if(i>=TEST_BUFFER_SIZE){
            printf ("passed equal \n");
        }else{
            printf ("not equal \n");
        }
        if(close(fd)<0){
            printf ("invalid closeing file\n");
        }
    }
    /*use wit setting params*/
    fd = open("/dev/qspi0", O_RDWR);
    if(fd<0){
        printf("Failed qspi open \n");
    }else{
        printf("qspi opened \n");
        qspi_attr_t qspi_attr;
        /*preinit settings attribute for qspi*/
        memset(&qspi_attr,0,sizeof(qspi_attr_t));
        memset(&qspi_attr.pin_assignment, 0xff, sizeof(qspi_pin_assignment_t));
        /*first set att master*/
        qspi_attr.o_flags = QSPI_FLAG_SET_MASTER;
        /*set size flash */
        qspi_attr.width = 26;    //2^26 - 64 mbytes
        /*clock prescaller 1 -> (mcu clock / 2)*/
        qspi_attr.freq = 1;
        /*pin assignment*/
        qspi_attr.pin_assignment.cs = mcu_pin(1,6);/*PB2*/
        qspi_attr.pin_assignment.sck = mcu_pin(1,2);/*PB6*/
        qspi_attr.pin_assignment.data[0] = mcu_pin(2,9);/*PC9*/
        qspi_attr.pin_assignment.data[1] = mcu_pin(2,10);/*PC10*/
        qspi_attr.pin_assignment.data[2] = mcu_pin(3,13);/*PD13*/
        qspi_attr.pin_assignment.data[3] = mcu_pin(4,2);/*PE2*/
        qspi_attr.read_instruction = QPI_READ_4_BYTE_ADDR_CMD;
        qspi_attr.mem_mapped_read_instruction = QUAD_OUT_FAST_READ_CMD;
        qspi_attr.write_instruction = QPI_PAGE_PROG_4_BYTE_ADDR_CMD;
        qspi_attr.dummy_cycle = QSPI_DUMMY_CYCLES_READ_QUAD_IO;
        ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
        printf("qspi sets with pin assignment \n");
        external_flash_set_qspi_mode(fd);
        qspi_enter_four_bytes_address(fd);
        qspi_set_dummy_cycles_and_strenght(fd,QSPI_DUMMY_CYCLES_READ_QUAD,QSPI_CR_ODS_15);
        qspi_erase_block(fd,0);
        qspi_erase_block(fd,256);
        external_flash_read(fd,0,test_buffer_read,TEST_BUFFER_SIZE);
        external_flash_write(fd,0,test_buffer_write,TEST_BUFFER_SIZE);
        external_flash_read(fd,0,test_buffer_read,TEST_BUFFER_SIZE);
        u16 i=0;
        for (i=0;i<TEST_BUFFER_SIZE;i++){
            if (test_buffer_read[i]!=test_buffer_write[i]){
                break;
            }
        }
        if(i>=TEST_BUFFER_SIZE){
            printf ("passed equal \n");
        }else{
            printf ("not equal \n");
        }
        if(close(fd)<0){
            printf ("invalid closeing file\n");
        }
    }

    return fd;
}
/**
  * @brief  This function read the SR
  * @param  fd qspi file
  * @retval status reg
  */
u8 qspi_read_status_reg(int fd){
    u8 data = 0;
    qspi_attr_t qspi_attr;
    memset(&qspi_attr,0,sizeof(qspi_attr_t));
    qspi_attr.o_flags = QSPI_FLAG_READ_REGISTER|QSPI_FLAG_IS_REGISTER_WIDTH_8;
    qspi_attr.command = READ_STATUS_REG_CMD;
    qspi_attr.data = &data;
    ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
    return data;
}
/*@brief write data
 * @param address - address start
 * @param buff pointer data to
 * @param length - data length
 * return value writed data
 * */
u16 external_flash_read(int fd, u32 address,u8* buff,u16 length){
    u16 result = 0;
    u16 page_count,remain;
    u16 mergin;
    //disable address mode if enabled
    mergin = (u8)(~address) + 1;
    if(length > mergin){
        lseek(fd,(int)address,SEEK_SET);
        read(fd,buff,mergin);
        result = mergin;
        length -= mergin;                              // re-calculate the number of elements
        buff += mergin;                                             // modify the pointer to the buffer
        address += mergin;                                             // modify the start address in the memory
        page_count = length / QSPI_PAGE_SIZE;  // calculate number of pages to be written
        remain = length % QSPI_PAGE_SIZE;   // calculate the remainder after filling up one or more entire pages
        while(page_count--){
            lseek(fd,(int)address,SEEK_SET);
            read(fd,buff,QSPI_PAGE_SIZE);
            result += QSPI_PAGE_SIZE;
            buff += QSPI_PAGE_SIZE;
            address += QSPI_PAGE_SIZE;
        }
        lseek(fd,(int)address,SEEK_SET);
        read(fd,buff,remain);
        result += remain;
    }else{
        lseek(fd,(int)address,SEEK_SET);
        read(fd,buff,length);
        result = length;
    }
    return result;
}
/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
int qspi_erase_block(int fd,u32 block_address){
    int result=0;
    /* Enable write operations */
    if (qspi_write_enable(fd) ==0){
        qspi_attr_t qspi_attr;
        u16 i=10000;
        u8 status_reg;
        /*change mode for read operations*/
        memset(&qspi_attr,0,sizeof(qspi_attr_t));
        qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER|QSPI_FLAG_IS_REGISTER_WIDTH_32;
        qspi_attr.command = SUBSECTOR_ERASE_4_BYTE_ADDR_CMD;
        qspi_attr.data = (u8*)&block_address;
        ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
        status_reg = qspi_read_status_reg(fd);
        while(status_reg & QSPI_SR_WIP){
            status_reg = qspi_read_status_reg(fd);
            if (i-- == 0) {
                result = -1;
                break;
            }
        }
    }else{
        result = -1;
    }
    return result;
}
/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  int fd - file id
  * @retval None
  */
int qspi_write_enable(int fd){
    int result = 0;
    u8 status;
    u16 i;
    const u16 MAX_ITERATES = 10;
    qspi_attr_t qspi_attr;
    i=0;
    status = qspi_read_status_reg(fd);
    for (i =0 ;i<MAX_ITERATES ;i++){
        /*change mode for read operations*/
        memset(&qspi_attr,0,sizeof(qspi_attr_t));
        qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER;
        qspi_attr.command = WRITE_ENABLE_CMD;
        ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
        status = qspi_read_status_reg(fd);
        if(status & QSPI_SR_WREN){
            break;
        }
    }
    if(i >=MAX_ITERATES ){
        printf("write enable error");
        result =-1;
    }
    return result;
}
/*@brief write data
 * @param fd - file id
 * @param address - address start
 * @param buff pointer data from
 * @param length - data length
 * return value writed data
 * */
u16 external_flash_write(int fd,u32 address,u8* buff,u16 length){
    u16 result = 0;
    u16 page_count,remain;
    u16 mergin;
    u8 status_reg;
    //writing
    status_reg = qspi_read_status_reg(fd);
    mergin = (u8)(~address) + 1;
    if(length > mergin){
        while(status_reg & QSPI_SR_WIP){
            status_reg = qspi_read_status_reg(fd);
        }
        qspi_write_enable(fd);
        lseek(fd,(int)address,SEEK_SET);
        write(fd,buff,mergin);
        result = mergin;
        length -= mergin;                              // re-calculate the number of elements
        buff += mergin;                                             // modify the pointer to the buffer
        address += mergin;                                             // modify the start address in the memory
        page_count = length / QSPI_PAGE_SIZE;  // calculate number of pages to be written
        remain = length % QSPI_PAGE_SIZE;   // calculate the remainder after filling up one or more entire pages
        while(page_count--){
            status_reg = qspi_read_status_reg(fd);
            while(status_reg & QSPI_SR_WIP){
                status_reg = qspi_read_status_reg(fd);
            }
            qspi_write_enable(fd);
            lseek(fd,(int)address,SEEK_SET);
            write(fd,buff,QSPI_PAGE_SIZE);
            buff += QSPI_PAGE_SIZE;
            address += QSPI_PAGE_SIZE;
            result+=QSPI_PAGE_SIZE;
        }
        status_reg = qspi_read_status_reg(fd);
        while(status_reg & QSPI_SR_WIP){
            status_reg = qspi_read_status_reg(fd);
        }
        qspi_write_enable(fd);
        lseek(fd,(int)address,SEEK_SET);
        write(fd,buff,remain);
        result+=remain;
    }else{
        while(status_reg & QSPI_SR_WIP){
            status_reg = qspi_read_status_reg(fd);
        }
        qspi_write_enable(fd);
        lseek(fd,(int)address,SEEK_SET);
        write(fd,buff,length);
        result = length;
    }
    return result;
}
/**
  * @brief  This function for enter in qspi mode.
  * @param  fs qspi file id
  * @retval None
  */
int external_flash_set_qspi_mode(int fd){
    int result,i;
    result = 0;
    u8 data;
    const u16 MAX_ITERATES = 10;
    qspi_attr_t qspi_attr;
    i=0;
    for (i =0 ;i<MAX_ITERATES ;i++){
        /*change mode for read operations*/
        memset(&qspi_attr,0,sizeof(qspi_attr_t));
        qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER | QSPI_FLAG_IS_INSTRUCTION_1_LINE;
        qspi_attr.command = ENTER_QUAD_CMD;
        ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
        data = qspi_read_status_reg(fd);
        if(data & QSPI_SR_QUADEN){
            qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER |QSPI_FLAG_IS_INSTRUCTION_4_LINE;
            qspi_attr.command = ENTER_QUAD_CMD;
            ioctl(fd, I_QSPI_SETATTR, &qspi_attr);

            break;
        }
    }
    if(i >=MAX_ITERATES ){
        result =-1;
    }
    return result;
}
/**
  * @brief  This function set the QSPI memory in 4-byte address mode
  * @param  fs qspi file id
  * @retval None
  */
int qspi_enter_four_bytes_address(int fd){
    int result;
    result = 0;
    qspi_attr_t qspi_attr;
    memset(&qspi_attr,0,sizeof(qspi_attr_t));
    qspi_write_enable(fd);
    /*change mode for read operations*/
    memset(&qspi_attr,0,sizeof(qspi_attr_t));
    qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER;
    qspi_attr.command = ENTER_4_BYTE_ADDR_MODE_CMD;
    ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
    return result;
}
/**
  * @brief  This function configure the dummy cycles on memory side.
  * @param  fs qspi file id
  * @retval None
  */
int qspi_set_dummy_cycles_and_strenght(int fd, u8 dummy_cycles,u8 strenght){
  u8 status[2];
  qspi_attr_t qspi_attr;
  memset(&qspi_attr,0,sizeof(qspi_attr_t));
  /*read one*/
  status[0] = qspi_read_status_reg(fd);
  /*read two*/
  qspi_attr.o_flags = QSPI_FLAG_READ_REGISTER|QSPI_FLAG_IS_REGISTER_WIDTH_8;
  qspi_attr.command = READ_CFG_REG_CMD;
  qspi_attr.data = &status[1];
  ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
  qspi_write_enable(fd);
  /*modify*/
  dummy_cycles = (u8)(dummy_cycles << 6) & QSPI_CR_NB_DUMMY;
  status[1] &= ~QSPI_CR_NB_DUMMY;
  status[1] |=dummy_cycles;
  strenght &= QSPI_CR_ODS;
  status[1] &= ~QSPI_CR_ODS;
  status[1] |=strenght;
  /*modify*/
  qspi_attr.o_flags = QSPI_FLAG_WRITE_REGISTER|QSPI_FLAG_IS_REGISTER_WIDTH_16;
  qspi_attr.command = WRITE_STATUS_CFG_REG_CMD;
  qspi_attr.data = &status[0];
  ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
  u16 i =10000;
  while(i--){};
  /*read one*/
  status[0] = qspi_read_status_reg(fd);
  /*read two*/
  qspi_attr.o_flags = QSPI_FLAG_READ_REGISTER|QSPI_FLAG_IS_REGISTER_WIDTH_8;
  qspi_attr.command = READ_CFG_REG_CMD;
  qspi_attr.data = &status[1];
  ioctl(fd, I_QSPI_SETATTR, &qspi_attr);
  return 0;
}
#endif
