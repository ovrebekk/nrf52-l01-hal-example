# nRF24L01 test application 
## Overview
A quick and dirty port of the nRF24 HAL to allow nRF24L01+ based modules to be connected to an nRF52832/nRF52840 DK. 

The nRF HAL interface is more or less unchanged, the main difference is the interface to the nRF5 SPI drivers and GPIO. 