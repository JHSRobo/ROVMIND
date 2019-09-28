# uart_thrusters_arduino

## Description

This is the core firmware and logic for the bottomside arduino microntroller. This firmware is equiped with a new packet parsing algorith and thruster control software.

## Goal

Allow a reliable independant microcontroller to provide additional safety features over a direct I2C to PWM converter or PWM signal generation on the main bottomside computer. 

## Instructions 

* Upload to an arduino and plug the default serial into /dev/ttyACM0 

## Communication Architecture
* Uses custom communication framework over a seiral port 
* Works with the hw_thruster_controller_interface.py under the hardware_interfaces package
* Wathcdog timer monitors if communication has been lost and will turn the thrsters OFF


## Troubleshooting

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michae Equi - inital work

## Helpful Resources

* Links, information, external articles that were helpful in creating anything in this package

