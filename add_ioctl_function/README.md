add ioctl functions steps
==============================
for example on qspi additinal functions 

1.add body function in file StratifyOS-mcu-stm32\src\qspi_dev.c with prefix **mcu_qspi_**
```cpp
...

int mcu_qspi_command(const devfs_handle_t * handle, void * ctl){
    qspi_local_t * qspi = qspi_local + handle->port * sizeof(qspi_local_t);
    u32 data_command;
    QSPI_CommandTypeDef command;
    data_command = (u32)ctl;
    mcu_debug_printf("mcu_qspi_command \n");
    command = get_command_config(qspi->state);
    command.Instruction       = data_command;
    command.AddressMode       = QSPI_ADDRESS_NONE;
    command.DataMode          = QSPI_DATA_NONE;
    command.NbData = 0;
    if( HAL_QSPI_Command(&qspi->hal_handle, &command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ){
        qspi->transfer_handler.write = 0;
        return SYSFS_SET_RETURN(EIO);
    }
    return 0;
}
...
```

and add function **mcu_qspi_command** to macros   "DEVFS_MCU_DRIVER_IOCTL_FUNCTION" with prefix mcu_qspi_

```cpp
...

DEVFS_MCU_DRIVER_IOCTL_FUNCTION(qspi, QSPI_VERSION, QSPI_IOC_IDENT_CHAR, I_MCU_TOTAL + I_QSPI_TOTAL, \
                                **mcu_qspi_command**,mcu_qspi_addr_command,\
                                mcu_qspi_read_regs,mcu_qspi_write_regs)
...
```

2.Correct StratifyOS\include\mcu\qspi.h
add function **command** to macros   MCU_QSPI_IOCTL_REQUEST_DECLARATION
```cpp
...

#define MCU_QSPI_IOCTL_REQUEST_DECLARATION(driver_name) \
	DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, getinfo); \
	DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, setattr); \
    DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, setaction);\
    DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, **command**);\
    DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, addr_command);\
    DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, read_regs);\
    DEVFS_DRIVER_DECLARTION_IOCTL_REQUEST(driver_name, write_regs)
...
```

 

3.Correct file StratifyOS\include\sos\dev\qspi.h
    add macros

```cpp
...
    #define I_QSPI_COMMAND _IOCTL(QSPI_IOC_IDENT_CHAR, I_MCU_TOTAL + 0)
...
```


    change I_QSPI_TOTAL in 

```cpp
...
    #define I_QSPI_TOTAL 4
...
```
