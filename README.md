# Sine_Wave_Inverter

This software runs on an atmega328p inside a repurposed inverter. The aim of the project was to have an inverter that could power a mini-fridge inside a campervan from a 24v battery. 

Bought inverters have a high quiescent power draw. (~6 Watts = 150wh/day). I wanted one that could sense when the fridge is off and enter a low power mode. I also wanted a sine wave output for smooth and efficient running. And hey, why not soft start the fridge to reduce the current stresses at startup?

I bought a 2nd hand 24v 300w modified sine wave inverter for it's parts and enclosure. I tap off the 320v DC it generates and use this to provide a DC rail for a simple H-Bridge circuit on a custom PCB from OSHpark. The Atmega328p generates the PWM signals to drive this H-Bridge that after filtering forms a clean sinewave. 

This software is responsible for generating the PWM. It also enables and disables the inverter's DC-DC (24v-320v) converter which is responsible for most of the quiescent current draw. It also monitors the input current measured by the ACS712 hall effect sensor. It uses the mesured input current to know whether the inverter is powering anything or not and enter a low power mode accordingly.


These are the pin connections:
#define         CURRENT_SENSE       A0
#define         INVERTER_CTRL_PIN   A1                          // This switches the DC-DC converter on/off for power savings
#define         VOLTAGE_IN          A2
#define         PHASE_B_PIN         A3                          // Our low frequency phase flipping  square wave output

#define         ACS_nENABLE         2                           // When Low, powers the ACS712, When high, shuts it down
#define         LED_PIN             9                           // Just an indicator LED
#define         PHASE_A_PIN         10                          // Our High frequency PWM pin



  1) Do nothing for 5 minute after turn on to allow compressor pressures to equalise (if device is turned off and then straight on again)
  2) Enable the inverter and pause for 2 seconds (Allows the initial inrush current to die down)
  3) Generate a 30Hz output signal
  4) Measure the 24v input current over the next 1 second (Using the ADS712-5A current sensor) 
  5) If 1 second average <1.0Amps then jump to 7), else start ramping the output from 30 to 50Hz adjusting the amplitude proportionally
  6) Monitor the previous 1 second current and voltage averages, if <1.0Amps OR < 20.0v move to step 7)
  7) Enter low power mode, wait 20 minutes
  8) If input voltage average > 21.0v return to step 2)
