# uart_thrusters_studio7_SAMD21

This is the core firmware and logic for the bottomside arduino microntroller. This firmware is equiped with a new packet parsing algorith and thruster control software. Additioanlly, extra accomidations for the native USB port on the SMAD21 Cortex-M microntroller have been made.

### Goal

A reliable independant microcontroller to provide additional safety features over a direct I2C to PWM converter or onboard PWM signal on the main bottomside computer. 

### Requirments and Description

* Flawless error handling
* Clean commented code
* logical layout and good code practices 
* Proper Atmel Studio 7 Debugging capabilities

### Break down into end to end tests

* 1 hour of rigorous testing on two distinct arduinos with a mix a valid and invlaid messages
* Must be able to handle system crashes and worst case scenario events while providing good debug inforamtion

### Coding detials 

* Built with Atmel Studio 7 
* Run on SAMD21 (Arduino M0)
* C++, Arduino
* No classes

## Contributing and Project Ownership

* Initial owner: Michael Equi
* Current maintaner:
* packet_reader algorithm contributer: 
* uart_thrusters_arduino calculateThrusterValues contributer:

## Versioning

* Out Dated Version 
* Active

## Authors

* Michael Equi

## Helpful Resources

* NONE
