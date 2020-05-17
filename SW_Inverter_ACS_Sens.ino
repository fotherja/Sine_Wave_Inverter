/*
  J.Fotherby 2020  
  Sinewave inverter: Amplitude and frequency adjustable, current detection 

  1) Do nothing for 5 minute after turn on -> to allow compressor pressures to equalise (should device be turned off and then straight on again)
  2) Enable the inverter and pause for 2 seconds (Allows the initial inrush current to die down)
  3) Generate a 30Hz output signal
  4) Measure the 24v input current over the next 1 second (Using the ADS712-5A current sensor) 
  5) If 1 second average <1.0Amps then jump to 7), else start ramping the output from 30 to 50Hz adjusting the amplitude proportionally
  6) Monitor the previous 1 second current and voltage averages, if <1.0Amps OR < 20.0v move to step 7)
  7) Enter low power mode, wait 20 minutes
  8) If input voltage average > 21.0v return to step 2)

  # ADC 0 -> Measures input current
  # ADC 2 -> Measures input voltage

  To Do:
   - Test, Test, Test!!!

  Possible Improvements:
   - Different Amplitude lookup tables for different input voltages. Say for 21-22, 22-23, 23-24, 24-25. 
        Built in hysterisis since increasing amplitude at lower voltages would draw more current and drop the input voltage further keeping the system in that amplitude range.
*/
 
//------------------------------------------------------------------------------------- 
//-------------------------------------------------------------------------------------
#include "LowPower.h"

//#define _DEBUG_MODE_                                            // Commenting out this will prevent all Serial port communication
 
#define         CURRENT_SENSE       A0                           
#define         INVERTER_CTRL_PIN   A1                          // This switches the DC-DC converter on/off for power savings
#define         VOLTAGE_SENSE       A2
#define         PHASE_B_PIN         A3                          // Our low frequency phase flipping square wave output

#define         ACS_nENABLE         2                           // When Low, powers the ACS712, When high, shuts it down
#define         LED_PIN             9                           // Just an indicator LED
#define         PHASE_A_PIN         10                          // Our High frequency PWM pin

#define         SINE_STEPS          256                         // Number of steps to build our sinewave in

#define         STATE_OFF           0
#define         STATE_HOLD          1
#define         STATE_RAMP          2
#define         STATE_ON            3

#define         START_FREQUENCY     42                          // 42 corrosponds to a 30Hz start frequency
#define         MAX_FREQUENCY       127
#define         RAMP_TIMESTEP       50                          // Timestep of 50ms corrosponds to a frequency ramp at a rate of ~4.5Hz/sec
#define         HOLD_TIME           1000                        // Time to stay at the startup frequency to check if fridge connected or not.
#define         SHUTDOWN_TIME       1200000                     // Try powing up the fridge after 20 minutes from when it shuts off

#define         LOW_VOLT_CUTOUT     8100                        // When voltage falls below this arbitary value we shutdown to protect the battery
#define         LOW_VOLT_CUTIN      8550                        // 8100 - 8550 corrisponds to 3.6v - 3.8v per cell
#define         NO_LOAD_I_THRESHOLD 5700                        // If current is less than this we assume the inverter is not powering anything (< ~1Amp)

void Startup ();
void Shutdown ();

//-------------------------------------------------------------------------------------
unsigned int    Sine_Lookup[128]          =       {0,20,39,59,78,98,117,137,156,175,194,213,232,251,269,288,            // Duty cycles to step through to produce a sinewave
                                                  306,324,342,359,377,394,411,427,444,460,476,492,507,522,537,551,
                                                  565,579,592,605,618,630,642,653,664,675,685,695,705,714,722,730,
                                                  738,745,752,759,765,770,775,780,784,787,790,793,795,797,798,799,
                                                  799,799,798,797,795,793,790,787,784,780,775,770,765,759,752,745,
                                                  738,730,722,714,705,695,685,675,664,653,642,630,618,605,592,579,
                                                  565,551,537,522,507,492,476,460,444,427,411,394,377,359,342,324,
                                                  306,288,269,251,232,213,194,175,156,137,117,98,78,59,39,20};

byte            Step_Delay_LOOKUP[128]    =       {195,193,191,189,187,184,182,180,178,177,175,173,171,169,168,166,     // Microsecond delays to produce frequencies from 20 to 50Hz
                                                  164,163,161,160,158,156,155,154,152,151,149,148,147,145,144,143,
                                                  142,141,139,138,137,136,135,134,133,132,131,130,129,128,127,126,
                                                  125,124,123,122,121,120,119,118,118,117,116,115,114,114,113,112,
                                                  111,110,110,109,108,108,107,106,106,105,104,104,103,102,102,101,
                                                  100,100,99,99,98,97,97,96,96,95,95,94,94,93,93,92,
                                                  92,91,91,90,90,89,89,88,88,87,87,86,86,85,85,85,
                                                  84,84,83,83,82,82,82,81,81,80,80,80,79,79,78,78};

byte            Amp_Freq_Adj_LOOKUP[128]  =       {26,26,26,27,27,27,27,28,28,28,29,29,29,30,30,30,                     // Multiplication factors
                                                  30,31,31,31,32,32,32,33,33,33,33,34,34,34,35,35,
                                                  35,36,36,36,36,37,37,37,38,38,38,39,39,39,40,40,
                                                  40,40,41,41,41,42,42,42,43,43,43,43,44,44,44,45,
                                                  45,45,46,46,46,46,47,47,47,48,48,48,49,49,49,49,
                                                  50,50,50,51,51,51,52,52,52,53,53,53,53,54,54,54,
                                                  55,55,55,56,56,56,56,57,57,57,58,58,58,59,59,59,
                                                  59,60,60,60,61,61,61,62,62,62,62,63,63,63,64,64};

                                                
int             Index_Phase         = 0;                        // Index incremented to step through the sinewave generation duty cycles
byte            Amplitude_Adj       = 32;                       // We multiply the duty cycle value by this then >> 6 (ie. /64) It's a CPU efficient way of scaling values 
byte            Step_Delay_us       = 125;                      
byte            State               = STATE_OFF;                // Either OFF, HOLD, RAMP or ON            
byte            Frequency           = START_FREQUENCY;          // This is not in hertz. The range is 0-127 which corrosponds to 20-50Hz

unsigned long   TimeStamp_Sine      = 0;
unsigned long   TimeStamp_Ramp      = 0;
unsigned long   TimeStamp_Startup   = 0;
unsigned long   TimeStamp_Shutdown  = millis() - 900000;        // Trick into thinking we shut down 15 minutes ago so we only wait 5 minutes before trying a start up

//-------------------------------------------------------------------------------------
void setup() 
{  
  digitalWrite(INVERTER_CTRL_PIN, LOW);  
  digitalWrite(ACS_nENABLE, HIGH);    
  digitalWrite(PHASE_A_PIN, LOW);    
  digitalWrite(PHASE_B_PIN, LOW); 
  digitalWrite(LED_PIN, LOW);

  pinMode(INVERTER_CTRL_PIN, OUTPUT);
  pinMode(ACS_nENABLE, OUTPUT);       
  pinMode(PHASE_A_PIN, OUTPUT);   
  pinMode(PHASE_B_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  TCCR1A = 0b00100010;
  TCCR1B = 0b00011001;                                          // Configure for 20KHz FAST PWM with ICR1 control
  ICR1 = 799;
  TCNT1 = 0; 
  OCR1B = 0;

  ADMUX = B01000000;                                            // Turn on ADC
  ADCSRA = B10000111;                                           // 128 Prescaler -> ADC Clk = 125KHz
  ADCSRA |= B01000000;                                          // Start a conversion in the background

  #ifdef _DEBUG_MODE_
    Serial.begin(115200);
    Serial.println("Starting");
  #endif
  
  delay(100);
}

//-------------------------------------------------------------------------------------
void loop() 
{      
  if(millis() - TimeStamp_Ramp >= RAMP_TIMESTEP)
  {    
    TimeStamp_Ramp = millis();

    // 1) Read the ADCs for input voltage and current. Keep 10 value running averages of both
    static byte indexV = 0, indexI = 0;
    static int BufferV[10], BufferI[10];
    static long SumV, SumI;
    
    if(ADCSRA & B00010000)  {                                   // If ADC conversion complete 
      if(ADMUX == B01000010) {                                  // ADC2 conversion complete
        BufferV[indexV++] = ADCW;                               // Insert latest reading into Buffer and increment index
        if (indexV > 9) {                                       // Keep index within limits
          indexV = 0;
        }
        
        SumV = 0;                                               // Reset Sum
        for(byte i = 0; i < 10; i++)  {                         // Add up all values in buffer
          SumV += BufferV[i];
        }
        
        ADMUX = B01000000;                                      // Select ADC channel 0 this time       
        ADCSRA |= B00010000;                                    // This clears the ADC complete flag and allows the new MUX channel to be written
        ADCSRA |= B01000000;                                    // Start a conversion in the background          
      }
      
      else if(ADMUX == B01000000)  {                            // ADC0 conversion complete
        BufferI[indexI++] = ADCW;                               // Insert latest reading into Buffer and increment index
        if (indexI > 9) {                                       // Keep index within limits
          indexI = 0;
        }
        
        SumI = 0;                                               // Reset Sum
        for(byte i = 0; i < 10; i++)  {                         // Add up all values in buffer
          SumI += BufferI[i];
        }
                          
        ADMUX = B01000010;                                      // Select ADC channel 2    
        ADCSRA |= B00010000;
        ADCSRA |= B01000000;                                    // Start a conversion in the background  
      }
    }


    // 2) State logic
    if(State == STATE_ON) {
      if(SumI < NO_LOAD_I_THRESHOLD)  {                         // If no current is being drawn despite producing an output waveform shutdown
        Shutdown();
      }    
      else if(SumV < LOW_VOLT_CUTOUT)  {                        // If the voltage is too low then cutout and remain in a low power mode indefinitely        
        Shutdown();        
      }  
    }
    else if(Frequency == MAX_FREQUENCY) {                            
      State = STATE_ON;
    }    
    else if(State == STATE_RAMP) {
      Frequency++;
    } 
    else if(State == STATE_HOLD)  {
      if(millis() - TimeStamp_Startup > HOLD_TIME) {            // If we've been producing our 30Hz output for 1 second we'll have a good current draw average measurement
        if(SumI > NO_LOAD_I_THRESHOLD)  {                       // If there is current being drawn the fridge must be connected
          State = STATE_RAMP;                                   // So ramp up the frequency to power the fridge properly
        }
        else  {
          Shutdown();                                           // Otherwise shutdown for a further 20 minutes before trying again
        }
      }
    }
    else if(State == STATE_OFF)  {                              
      //LowPower.idle(SLEEP_1S, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
      delay(1000);

      #ifdef _DEBUG_MODE_        
        Serial.print("SumV: "); Serial.println(SumV);
        Serial.print("SumI: "); Serial.println(SumI);
        Serial.print("SD Time: "); Serial.println((millis() - TimeStamp_Shutdown)/1000); Serial.println();
      #endif

      if(SumV > LOW_VOLT_CUTIN) {                               // If our battery voltage is good
        if(millis() - TimeStamp_Shutdown > SHUTDOWN_TIME) {     // And we've completed our 20 minute shutdown period
          Startup();                                            // Then try a startup procedure
        }
      }
    }  
    

    // 3) Write delay and amplitute values from lookup tables
    Step_Delay_us = Step_Delay_LOOKUP[Frequency];               // Step_Delay_us -> 20Hz - 50Hz 
    Amplitude_Adj = Amp_Freq_Adj_LOOKUP[Frequency];             // Change the amplitude proportional to the frequency. At 25Hz the amplitude should be half that of 50Hz        
  }

  
  // Vary the duty cycle on PHASE_A. PHASE_B is a square wave. Simultaneously flip our PHASE_A & B lines at the zero crossing points (0 & 180 degrees)
  static unsigned int Next_OCR1B = 0;                           // Allows us to precalculate the OCR1A value for a snappy switch
  if(State != STATE_OFF)
  {
    // Have a constant delay in our loop for stepping through our sinewave generation
    while(micros() - TimeStamp_Sine < Step_Delay_us) {}      
    TimeStamp_Sine += Step_Delay_us; 
    
    // Sinewave generation logic
    if(Index_Phase == 0)  {         
      TCNT1 = 793;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go low simultaneously.              
      OCR1B = 0;    
      PORTC = PORTC & 0b11110111;                               // ie. digitalWrite(PHASE_B, LOW);
      Next_OCR1B = (20 * Amplitude_Adj) >> 6;                   // The index will be 1 next which corrosponds to a lookup value of 20
    }
    else if(Index_Phase < 128)  {
      OCR1B = Next_OCR1B;                                       // We can immediately write to this register because we pre-calculated its value
      Next_OCR1B = (Sine_Lookup[Index_Phase + 1] * Amplitude_Adj) >> 6;
    }    
    else if(Index_Phase == 128) {         
      TCNT1 = 793;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go high simultaneously.  
      OCR1B = 799;  
      PORTC = PORTC | 0b00001000;                               // ie. digitalWrite(PHASE_B, HIGH);
      Next_OCR1B = 799 - ((20 * Amplitude_Adj) >> 6);                          
    }
    else  {
      OCR1B = Next_OCR1B;
      Next_OCR1B = 799 - ((Sine_Lookup[Index_Phase - 127] * Amplitude_Adj) >> 6);
    }

    // Increment the step counter
    Index_Phase += 1;
    if(Index_Phase >= SINE_STEPS)  {
      Index_Phase = 0;
    }  
  }
}


void Startup ()
{
  // Startup procedure: Turn the inverter on, then start sinewave generation    
  digitalWrite(INVERTER_CTRL_PIN, HIGH);
  digitalWrite(ACS_nENABLE, LOW);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  #ifdef _DEBUG_MODE_        
    Serial.print("Startup!");
  #endif  

  Index_Phase = 0; 
  State = STATE_HOLD;
  Frequency = START_FREQUENCY;
  TimeStamp_Startup = millis(); 
  TimeStamp_Sine = micros();   
}


void Shutdown ()
{
  // Shutdown procedure: Turn off the inverter, then ensure Phase A&B both go low simultaneously 
  digitalWrite(INVERTER_CTRL_PIN, LOW); 
  digitalWrite(ACS_nENABLE, HIGH);  
  digitalWrite(LED_PIN, LOW);

  #ifdef _DEBUG_MODE_        
    Serial.print("Shutdown!");
  #endif 

  State = STATE_OFF; 
  Frequency = START_FREQUENCY;     
  TCNT1 = 793;                                                          
  OCR1B = 0;    
  PORTC = PORTC & 0b11110111;  
  TimeStamp_Shutdown = millis();
}














