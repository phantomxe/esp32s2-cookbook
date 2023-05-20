# ESP32-S2 CH32V003 Programmer

Connect SWIO (Pin D4) on CH32V003 to GPIO6 on the ESP32S2.
 * Optionally use 10k pull-up on SWIO.
 * Optional Clock Synthesizer is on GPIO2 "Multi2" / "M0"
 * VCC Output on GPIO11/12 on ESP32S2.

Use a DevkitC. Once programmed, see ch32v003fun for more detailed instructions.

You can either use a ESP32-S2 [DevkitC](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S2-DEVKITC-1-N8R2/16688755) or [this](https://github.com/cnlohr/cnhardware/tree/master/esp32s2-funprog)

This is the ESP32-S2 programmer for [ch32v003fun](https://github.com/cnlohr/ch32v003fun)

Also Lolin S2-Mini works with some additional circuit needed for unbrick function of minichlink. Soic-8 version of ch32v003 j4m6 can unbrick sometimes when using the PD4 as GPIO.

* SWIO on GPIO6 
* Power control line GPIO11 at 3.3V
* Power control line GPIO12 at 5V
* UART TX on GPIO17
* UART RX on GPIO18
 
<img src="https://github.com/phantomxe/esp32s2-cookbook/assets/22988043/d00f9143-06b9-4eea-bc95-37cbcfd4f618" width="400">


## To just flash it without building

```
esptool.py -p /dev/ttyACM1 -b 460800 --before=no_reset --after=no_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 build/bootloader/bootloader.bin 0x10000 build/usb_sandbox.bin 0x8000 build/partition_table/partition-table.bin
```
Or if you, like about half the programmers on this planet are struggling getting python to do the right thing, just use ESPUtil https://github.com/cpq/esputil


