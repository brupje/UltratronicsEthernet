# UltratronicsEthernet

This is a library for Arduino adding support for ENC424J600 chip.

# Releases

2016/05/30: Initial release (alpha). Not considered production ready, but the TCP server and client work.<br/>
2016/10/05: Update for stability. Should work good enough, included a flood test program. <br/>
2018/08/26: Changes to support latest Arduino. Increased default buffer size to handle larger frames.<br/>


# Installation

Download the latest release from github https://github.com/brupje/UltratronicsEthernet. In the Arduino menu, find Sketch -> include Library -> Add .ZIP Library and select the downloaded file.

# Testing

To test the board, you can set-up a test. Open up the TcpFloodtest example in Arduino and change the line IPAddress myIP(192,168,0,1); into the desired IP address. Download the source code from Github https://github.com/brupje/UltratronicsEthernet. Compile the test executable in the source folder gcc ./UltratronicsEthernet/test/test.c -o test. Run the test code using:<br/><br/>

./test [ip address]

# Debugging

In UIPEthernet.h uncomment:<br/>
//#define UIPETHERNET_DEBUG<br/>
//#define UIPETHERNET_DEBUG_CHKSUM<br/>
//#define UIPETHERNET_DEBUG_UDP<br/>
//#define UIPETHERNET_DEBUG_CLIENT<br/><br/>

In Enc424J600Network.h uncomment:<br/>
//#define ENC28J60DEBUG 1<br/>
//#define DEBUGSERIAL Serial<br/>
Set DEBUGSERIAL to the appropiate Serial (or SerialUSB)<br/>

# Changing the MAC address

To change the MAC address from the default, you either need to change the UIP_ETHADDRx constants in uipethernet-conf.h and change the line uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05}; accordingly. Or in uipopt.h change #define UIP_FIXEDETHADDR 1 to #define UIP_FIXEDETHADDR 0, so you only have to edit the line uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};.

# Copyright

This library is based on the work of many others. See https://github.com/ntruchsess/arduino_uip for details.
