[StratifyOs](https://github.com/StratifyLabs/StratifyOS) examples
=====================

Board description stm32f723iec
==============================
    1. external flash use - MX25L51245G from macronix(512 mbit - 64 mbyte) is connected
    to Quad-SPI interface of STM32F723IEK6.
    2. external ram use - IS66WV51216EBLL-55BLI from Integrated Silicon Solution Inc
    ( 8 mbit - 1 mbyte) is connected to the FMC interface of the STM32F723IEK6 with 16
    bits of data and 18 bits of addresses (4-Mbit memory accessible).
    3. TFT LCD 240x240 pixels FRD154BP2902 from Frida is connected to FMC data interface
    of STM32F723IEK6.
    4. Capacitive Control Touch Panel (Frida LS015GF614A) is controlled by STM32F723IEK6
    through I2C.
    5.An audio codec WM8994ECS/R from CIRRUS with 4 DACs and 2ADCs is connected to SAI
    interface of STM32F723IEK6. It communicates with STM32F723IEK6 via I2C bus:
        The analog line input is connected to ADC of WM8994ECS/R through blue audio 
        jack CN4
        The analog line output is connected to DAC of WM8994ECS/R via green audio 
        jack CN5
        Two external speakers can be connected to WM8994ECS/R via CN10 for left 
        speaker and CN7 for right speaker
        Four digital microphones (ST-MEMS microphone) MP34DT01TR are on 32F723EDISCOVERY 
        board. They are connected to input digital microphones of WM8994ECS/R