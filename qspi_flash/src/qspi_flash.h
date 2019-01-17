#ifndef QSPI_TEST_H_
#define QSPI_TEST_H_

#if defined __cplusplus
extern "C" {
#endif
#include <trace.h>
int qspi_flash(void);



/* Configuration Register for mx25l512 start */
#define QSPI_DUMMY_CYCLES_READ_QUAD      3
#define QSPI_DUMMY_CYCLES_READ           8
#define QSPI_DUMMY_CYCLES_READ_QUAD_IO   10
#define QSPI_DUMMY_CYCLES_READ_DTR       6
#define QSPI_DUMMY_CYCLES_READ_QUAD_DTR  8

#define QSPI_CR_ODS                      ((uint8_t)0x07)    /*!< Output driver strength */
#define QSPI_CR_ODS_30                   ((uint8_t)0x07)    /*!< Output driver strength 30 ohms (default)*/
#define QSPI_CR_ODS_15                   ((uint8_t)0x06)    /*!< Output driver strength 15 ohms */
#define QSPI_CR_ODS_20                   ((uint8_t)0x05)    /*!< Output driver strength 20 ohms */
#define QSPI_CR_ODS_45                   ((uint8_t)0x03)    /*!< Output driver strength 45 ohms */
#define QSPI_CR_ODS_60                   ((uint8_t)0x02)    /*!< Output driver strength 60 ohms */
#define QSPI_CR_ODS_90                   ((uint8_t)0x01)    /*!< Output driver strength 90 ohms */
#define QSPI_CR_TB                       ((uint8_t)0x08)    /*!< Top/Bottom bit used to configure the block protect area */
#define QSPI_CR_PBE                      ((uint8_t)0x10)    /*!< Preamble Bit Enable */
#define QSPI_CR_4BYTE                    ((uint8_t)0x20)    /*!< 3-bytes or 4-bytes addressing */
#define QSPI_CR_NB_DUMMY                 ((uint8_t)0xC0)    /*!< Number of dummy clock cycles */

#define QUAD_OUT_FAST_READ_CMD               0x6B
/* Quad Operations */
#define ENTER_QUAD_CMD                       0x35
#define EXIT_QUAD_CMD                        0xF5
/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04
#define QUAD_IN_FAST_PROG_CMD                0x38
#define EXT_QUAD_IN_FAST_PROG_CMD            0x38
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x3E

/* Status Register */
#define QSPI_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define QSPI_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */
#define QSPI_SR_BLOCKPR                  ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
#define QSPI_SR_PRBOTTOM                 ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
#define QSPI_SR_QUADEN                   ((uint8_t)0x40)    /*!< Quad IO mode enabled if =1 */
#define QSPI_SR_SRWREN                   ((uint8_t)0x80)    /*!< Status register write enable/disable */
/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define READ_CFG_REG_CMD                     0x15
#define WRITE_STATUS_CFG_REG_CMD             0x01

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

#define QSPI_PAGE_SIZE                       256
#define TIMER_SAFE_VALUE                     10000

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QPI_READ_4_BYTE_ADDR_CMD             0xEC
/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21
/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define QPI_PAGE_PROG_4_BYTE_ADDR_CMD        0x12
/**
  * @brief  This function read the SR
  * @param  fd - file id
  * @retval None
  */
u8 qspi_read_status_reg(int fd);
/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  int fd - file id
  * @retval None
  */
int qspi_write_enable(int fd);
/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
int qspi_erase_block(int fd,u32 block_address);

/*@brief write data
 * @param address - address start
 * @param buff pointer data to
 * @param length - data length
 * return value writed data
 * */
u16 external_flash_read(int fd, u32 address,u8* buff,u16 length);
/*@brief write data
 * @param fd - file id
 * @param address - address start
 * @param buff pointer data from
 * @param length - data length
 * return value writed data
 * */
u16 external_flash_write(int fd,u32 address,u8* buff,u16 length);
/**
  * @brief  This function for enter in qspi mode.
  * @param  fs qspi file id
  * @retval None
  */
int external_flash_set_qspi_mode(int fd);
/**
  * @brief  This function set the QSPI memory in 4-byte address mode
  * @param  fs qspi file id
  * @retval None
  */
int qspi_enter_four_bytes_address(int fd);
/**
  * @brief  This function configure the dummy cycles on memory side.
  * @param  fs qspi file id
  * @retval None
  */
int qspi_set_dummy_cycles_and_strenght(int fd, u8 dummy_cycles,u8 strenght);


/* Configuration Register for mx25l512 end */
#if defined __cplusplus
}
#endif

#endif
