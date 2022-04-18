<!-- Please do not change this html logo with link -->

<a href="https://www.microchip.com" rel="nofollow"><img src="images/microchip.png" alt="MCHP" width="300"/></a>

#  Getting Started with Inter-Integrated Circuit (I<sup>2</sup>C) Using the AVR64DD32 Microcontroller

This example shows the basic functionality of the Two-Wire Interface (TWI) peripheral with a basic I<sup>2</sup>C communication, both reading and writing from or to the Client devices. The device is connected to the PICkit™ Serial I<sup>2</sup>C Demo Board to communicate with the MCP3221 and MCP23008 I<sup>2</sup>C Client devices. The AVR64DD32 first scans the I<sup>2</sup>C bus to discover all devices available on the bus. A message is transmitted through the Universal Synchronous and Asynchronous Receiver and Transmitter (USART) to indicate the addresses of the devices that respond with the Acknowledge (ACK) signal. A voltage is read by the MCP3221 (equipped with a 12-bit Analog-to-Digital Converter (ADC)) and the result is transmitted through I<sup>2</sup>C to the AVR64DD32 device. Then, the MCU transmits an 8-bit sequence to the MCP23008, equipped with an I/O Expander, to drive 8 LEDs on an LED bar. The LED display (consisting of 8 LEDs) indicates the potentiometer voltage value (divided in subranges).
<br><br>

## Related Documentation

More details and code examples on the AVR64DD32 can be found at the following links:

- [AVR64DD32 Product Page](https://www.microchip.com/wwwproducts/en/AVR64DD32)
- [AVR64DD32 Code Examples on GitHub](https://github.com/microchip-pic-avr-examples?q=AVR64DD32)
- [AVR64DD32 Project Examples in START](https://start.atmel.com/#examples/AVR64DD32CuriosityNano)

## Software Used

- [MPLAB® X IDE](http://www.microchip.com/mplab/mplab-x-ide) v6.00 or newer
- [MPLAB® X IPE](https://www.microchip.com/en-us/tools-resources/production/mplab-integrated-programming-environment) v6.00 or newer
- [MPLAB® XC8](http://www.microchip.com/mplab/compilers) v2.36 or newer
- [AVR-Dx Series Device Pack](https://packs.download.microchip.com/) v2.1.152 or newer

## Hardware Used

- The AVR64DD32 Curiosity Nano Development board is used as a test platform
  <br><img src="images/AVR64DD32.PNG" width="500">
- PICkit Serial I<sup>2</sup>C Demo Board [(PKSERIAL-I<sup>2</sup>C1)](https://www.microchip.com/DevelopmentTools/ProductDetails/PKSERIAL-I2C1)
  - MCP3221
  - MCP23008


## Operation

To program the Curiosity Nano board with this MPLAB® X project, follow the steps provided in the [How to Program the Curiosity Nano Board](#how-to-program-the-curiosity-nano-board) chapter.<br><br>

## Setup

The AVR64DD32 Curiosity Nano Development board is used as a test platform. The 12-bit ADC is used to transmit an analog value.

<br>The following configurations must be made:

|    Pin    |   Configuration    |
| :-------: |   :-----------:    |
| PA2 (SDA) |    Digital I/O     |
| PA3 (SCL) |    Digital I/O     |
| PD4 (TX)  |    Digital output  |

- The AVR64DD32 Microcontroller is connected to the PICkit Serial I<sup>2</sup>C Demo Board as in the figure below:
<br><img src="images/avr64dd32-i2c.jpg" width="800">

## Demo

- The software diagram of this application is presented in the figure below:
<br><img src="images/flow-diagram.png" width="250">

- After initializing the I<sup>2</sup>C and USART, the microcontroller will scan the bus to discover the Client devices available. After detecting the devices, a read command and a write command will be performed using those devices. The flow of the application is presented in the figure below:
 <br><img src="images/block-diagram.png" width="800">

- Visualize the serial output on the terminal window. In this example, the I<sup>2</sup>C device scanning phase is described. The Client devices connected respond with Acknowledge (ACK).
<br><img src="images/serial-scan.png">

- After turning the potentiometer rotor one way and the other, the LEDs will indicate the level of that voltage, as shown below. 
<br><img src="images/I2C_LED_bar_Result.gif">

- The ADC conversion result is received from the 12-bit ADC I<sup>2</sup>C Client device, it is normalized (obtaining a 3-bit value to be indicated using the 8-LED bar), and the result is transmitted to the 8-bit I/O Expander I<sup>2</sup>C Client device.
<br><img src="images/serial-capture.png">

## Summary 

This application shows how to use the TWI0 as an I<sup>2</sup>C compatible Host to communicate with two I<sup>2</sup>C Client devices:

- MCP3221 – equipped with a 12-bit ADC
- MCP23008 – equipped with an I/O Expander
<br>

[Back to top](#getting-started-with-inter-integrated-circuit-isup2supc-using-the-avr64dd32-microcontroller)<br>

##  How to Program the Curiosity Nano Board

This chapter shows how to use the MPLAB® X IDE to program an AVR® device with an Example_Project.X. This can be applied for any other project.

1.  Connect the board to the PC.

2.  Open the Example_Project.X project in MPLAB® X IDE.

3.  Set the Example_Project.X project as main project.
    <br>Right click the project in the **Projects** tab and click **Set as Main Project**.
    <br><img src="images/Program_Set_as_Main_Project.PNG" width="600">

4.  Clean and build the Example_Project.X project.
    <br>Right click the **Example_Project.X** project and select **Clean and Build**.
    <br><img src="images/Program_Clean_and_Build.PNG" width="600">

5.  Select **AVRxxxxx Curiosity Nano** in the Connected Hardware Tool section of the project settings:
    <br>Right click the project and click **Properties**.
    <br>Click the arrow under the Connected Hardware Tool.
    <br>Select **AVRxxxxx Curiosity Nano** (click the **SN**), click **Apply** and then **OK**:
    <br><img src="images/Program_Tool_Selection.PNG" width="600">

6.  Program the project to the board.
    <br>Right click the project and click **Make and Program Device**.
    <br><img src="images/Program_Make_and_Program_Device.PNG" width="600">

<br>

- [Back to top](#getting-started-with-inter-integrated-circuit-isup2supc-using-the-avr64dd32-microcontroller)
