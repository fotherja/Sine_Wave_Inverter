# Sine_Wave_Inverter

This software runs on an atmega328p inside a repurposed inverter. The aim of the project was to have an inverter that could power a mini-fridge inside a campervan from a 24v battery. It draws 250uA when the fridge is off. 

Bought inverters have a quiescent power draw due to the DC/DC converter inside them. Even ~6 Watts = 150wh/day which is horrid when you reliant on solar power. I wanted one that could sense when the fridge is off and enter a low power mode. I also wanted a sine wave output for smooth and efficient running. And hey, why not soft start the fridge to reduce the current stresses at startup?

I bought a £30 2nd hand 24v 300w modified sine wave inverter for it's parts and enclosure. I've tapped off the 320v DC it generates and use this to provide a DC rail for a simple H-Bridge circuit on a custom PCB from OSHpark. The Atmega328p generates the PWM signals to drive this H-Bridge that after filtering form a clean sinewave. 

This software is responsible for generating the PWM. It also enables and disables the inverter's DC-DC (24v-320v) converter which is responsible for most of the quiescent current draw. It monitors the DC input current measured by the ACS712 hall effect sensor to know whether the inverter is powering anything or not and enter a low power mode accordingly. Voltage sensing prevents overdischarge of the battery.

Pin descriptions are clearly defined in the software.



