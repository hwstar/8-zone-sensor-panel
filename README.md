![alt text](https://github.com/hwstar/8-zone-sensor-panel/blob/main/assets/logo.png)

![alt text](https://github.com/hwstar/8-zone-sensor-panel/blob/main/assets/proto-pic.jpg)

![alt text](https://github.com/hwstar/8-zone-sensor-panel/blob/main/assets/made-for-esphome-black-on-white.png)

This is the code and documentation repository for an 8 zone sensor panel
controller kit. It has the following features:

* ESP32 Microcontroller utulizing the WT32-ETH01 Ethernet Module.
* Uses a 40VA 16.5VAC wall transformer with an internal fuse.
* Charges a 12V 7A lead acid battery 
* AC power failure detection.
* 8 supervised zones (Expandable on-board to 16). Using 2K ohm end of line resistors.
* 1 Output for a 12V siren.
* 4 pin JST connector with I2C bus signals and 3.3V
* 8 pin JST connector for future zone expanders.
* 5 pin connector for future keypad terminals.
* 2 12 pin pluggable terminal blocks for zone sensor inputs
* 1 12 pin connector for AC power, AUX power, Siren and Battery connections.
* 3 status LED's show AC power state, siren state, and ESP32 heartbeat
* TVS diodes on all sensor inputs, power, and other external connectors.
* On-board TMP102 Temperature sensor.
* Monitoring of battery, auxilliary, and siren voltages using the ESP32 ADC.
* PTC fuse protection on the Battery, Auxillary Power and Siren outputs.

This hardware will be for sale on Tindie when it has been fully tested.
It is primarily designed to work with [ESPHome](https://github.com/esphome/esphome) firmware version 2023.6.3 or later.

## Disclaimer

By purchasing and/or using this sensor panel hardware, you indicate you agree that are using it at your own risk.  You agree that I will not be held liable for hardware failure, fire, injury, death, theft, property damage or any other loss. Professional monitoring is not available for this product from me, and this is something you will need to consider when using this hardware. Since the hardware is shipped in kit form and you will be modifying the software, you will be solely responsible for determining if this hardware and software is fit for purpose. For non-commercial,  residential use only.

## Audience

The controller is shipped without firmware which can connect to your WIFI network. It will require some firmware programming and electrical knowledge to successfully implement. 
One should have some experience modifying ESPHome yaml files, reading wiring diagrams, using a Windows or Linux command line shell, using serial devices such as the FTDI serial cable,
and using the ESPHome tool to program ESPHome devices. If you are not sure how to program firmware into an ESPHome device, please study the documentation on esphome.com and become
familiar with it before purchasing a board. A link to the the programming documentation can be found  !!TODO Update link!!

## WIKI

Besides this README.md, there is more documentation in the WIKI !!TODO Update Link!!
