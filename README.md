STC-1000p on steroids
==========

This is a version of the STC-1000p (STC-1000p-STM8) with new hardware and a new microcontroller, the **STM8S105C6T6**. It is a major upgrade to the stock **STM8S003F3** µC. In the version, you get all the exciting features from
the stc-1000p-stm8 version (including the PID-control with SSR output), PLUS:
- Three additional LEDs placed directly above the 7-segment display. These can be used for all kinds of indications.
- Interface for an **nRF24L01+** 2.4 GHz transceiver. Now you can control and communicate wireless with the STC1000p. The nRF24L01+ is known to have a large range (up to 100 metres is reported).
- One-wire interface, so you can hook up a **DS18B20** One-Wire temperature-sensor for reliable temperature measurements.
- **I<sup>2</sup>C**-interface for additional connection to other hardware (e.g. a MCP23017 16-bit IO expander to control solenoid ball valves).
- An UART interface so that commands can be sent using serial communication.
- A 433 MHz transmitter interface is added so that temperatures and relay on/off are sent (using the Fine Offset protocol from Matts original fimware).

The **STM8S105C6T6** µC has 32 Kbytes of Flash memory, 1024 bytes of EEPROM and 2 Kbytes of RAM. So there's plenty of room for all new functionality. It also has more than sufficient IO pins, since it is a LQFP48 package.

I designed a new frontpanel PCB and backplane PCB that are the exact same sizes as the current PCBs. So it fits the existing housing.

Software Development Environment
-----------
- Cosmic STM8 compiler. A license can be obtained free of charge from Cosmic-software.com. Install this first.
- ST Visual Develop (STVD). This can be obtained free of charge from st.com
- A coloured ST-Link V2 USB adapter (which are very cheap to find on ebay)

In order to program this thing, you can't have a code-size limit of 8 Kbytes, so IAR will not do the job. Fortunately, the Cosmic STM8 development environment (IdeaSTM8) has been made available free-of-charge. Just register and download and you have 
a development environment that nicely supports this device.

Quick start
-----------
To be filled in...

Introduction
--------------
To be filled in...

Frontpanel PCB
----------
Here's the second version of the new frontpanel PCB.

![frontpanel](img/stc1000p_steroids_schematics.png)<br>
*Eagle schematics v0.21 of the Frontpanel with an STM8S105C6 µC*

![frontpanel](img/stc1000p_steroids_pcb.png)<br>
*Eagle PCB of the Frontpanel, v0.21*

![frontpanel](img/Frontpanel_pcb.jpg)<br>
*PCB as received from the PCB manufacturer (allpcb.com).*

Backplane PCB
----------
Although it is possible to connect the frontpanel PCB directly to an existing backplane, it is much useful to replace this with a new backplane. Main advantage is that you will have decent access to all new features via connectors at the back.

![backplane](img/stc1000p_backplane_schematics.png)<br>
*Eagle schematics of the new backplane*

![backplane](img/stc1000p_backplane_pcb.png)<br>
*Eagle PCB of the new backplane*

![backplane](img/Backplane_pcb.jpg)<br>
*PCB as received from the PCB manufacturer (allpcb.com).*

There's now a 10 pole terminal-block with the following connections:
**T2**: connect a temperature sensor to this pin and the +5V pin (same as in original STC1000)
**+5V**: +5V power-supply voltage (same as in original STC1000)
**T1**: connect a second temperature sensor to this pin and the +5V pin (same as for Matts firmware)
**GND**: ground (0 V) connection. 
**SSR**: SSR output. Connect this to the input line of a solid-state relay. This is a PID controlled output.
**SCL**: Serial clock-line of the I<sup>2</sup>C interface.
**SDA**: Serial data-line of the I<sup>2</sup>C interface.
**OW**: One-wire output. Support a DS18B20 temperature sensor for more accurate readings.
**RX**: Receive-data pin for the serial communication (UART). This is a 3.3 Volt pin, so do not connect higher voltages!
**TX**: Transmit-data pin for the serial communication (UART). This is a 3.3 Volt pin, so do not connect higher voltages!

Updates
-------

|Date|Release|Description|
|----|-------|-----------|
|2016-12-10|v1.00|First release |
|2018-03-23|v1.10|Frontpanel PCB update + new backplane PCB added|

