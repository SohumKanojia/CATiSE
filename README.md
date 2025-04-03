# CATiSE

## Ground Station Repository
https://github.com/ReeceW18/cubecats-calico-ec2/tree/main

## Wiring and Notes
Set Serial to 9600

### LED Pin Connection
 Digital Pins is top right, when low, lights are on, when high lights are off. resistors 10k, smaller is 1k ohms
 * Line 4 is negative (GND)
 * Line 3 is positive (VCC)
 Arduino Lines:
 * line 2 is negative 
 * line 1 is positive
 
### Iridium Module: 
Make sure Tx and Rx are connected to Serial3 on arduino. 
(Pins start from right, see https://docs.groundcontrol.com/iot/rockblock/specification/connectors-wiring)
* Pin 1 to RX on Arduino
* Pin 6 to TX on Arduino
* Pin 8 to VCC (5v)
* Pin 10 to GND
  
### GPS: 
(VCC to 3.3v)

### Temperature/Humidity:
Pin 7 for Digital Communication.


