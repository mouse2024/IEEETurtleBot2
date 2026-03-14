#Master Code 
#Orkun KESKİN Jan 2022

import spidev
import time
#SPI Bus and CE pin selection 
spi_bus=0
spi_device=0

#SPI activating
spi = spidev.SpiDev()
spi.open(spi_bus,spi_device)
#Transmitting speed is 1Mbps
spi.max_speed_hz=1000000

#Sending Sequence
while True:
    send_byte = 0x05  #Sent byte to the Receiver from Transmitter
    spi.xfer2([send_byte])
    time.sleep(0.5)  #Delay for Arduino's register, it may be 0.25
