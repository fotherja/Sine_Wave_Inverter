/*
  J.Fotherby 2020  
  Sinewave inverter: Amplitude and frequency adjustable, current detection 

  // Depending on whether the ACS current sensor is attached we behave differently
    DETACHED - Soft start and stay on continuously
    ATTACHED - Perform as below

  1) Do nothing for 5 minute after turn on -> to allow compressor pressures to equalise (should device be turned off and then straight on again)
  2) Enable the inverter and pause for 2 seconds (Allows the initial inrush current to die down)
  3) Generate a 30Hz output signal
  4) Measure the ~24v DC input current over the next 1 second (Using the ADS712-5A current sensor) 
  5) If 1 second average <1.0Amps then jump to 7), else start ramping the output from 30 to 50Hz adjusting the amplitude proportionally
  6) Monitor the previous 1 second current and voltage averages, if <1.0Amps OR < 20.0v move to step 7)
  7) Enter low power mode, wait 20 minutes
  8) If input voltage average > 21.0v return to step 2)

  # ADC 0 -> Measures input current
  # ADC 2 -> Measures input voltage

  In order to keep to the 4.6v/Hz that appliances expect the inverter measures the input voltage just before startup. If < AMP_ADJUST_THRES (23.2v):
    A) Limit the max_frequency to MAX_FREQUENCY_LOW (45Hz)
    B) Use a different amplitude adjust lookup table

  This should help the fridge run more efficiently. 
*/
 
//------------------------------------------------------------------------------------- 
//-------------------------------------------------------------------------------------
#include "LowPower.h"

//#define _DEBUG_MODE_                                          // Commenting out this will prevent all Serial port communication
 
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
#define         MAX_FREQUENCY_HIGH  127                         // Corrosponds to 50Hz
#define         MAX_FREQUENCY_LOW   107                         // Corrosponds to 45Hz
#define         RAMP_TIMESTEP       47                          // Timestep of 47ms corrosponds to a frequency ramp at a rate of ~4.5Hz/sec (also chosen to avoid aliasing of ADCs)
#define         HOLD_TIME           1000                        // Time to stay at the startup frequency to check if fridge connected or not.
#define         SHUTDOWN_TIME       (1200000 / 21)              // Try powing up the fridge after 20 minutes from when it shuts off (like this because powerDown stops the timers etc)

#define         LOW_VOLT_CUTOUT     8100                        // When voltage falls below this value we shutdown to protect the battery
#define         LOW_VOLT_CUTIN      8360                        // 8100 - 8360 corrisponds to 21.5v - 22.2v
#define         NO_LOAD_I_THRESHOLD 5600                        // If current is less than this we assume the inverter is not powering anything (< ~1Amp)

#define         AMP_ADJUST_THRES    8760                        // ~23.2v, less than this and we set a max frequency of 45Hz == 107 and adjust our amplitude scaling at startup

void Startup ();
void Shutdown ();
void Detect_ACS ();
void Set_Max_Freq();

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

byte            Amp_Freq_Adj_LOOKUP_HIGH[128]  =  {26,26,26,27,27,27,27,28,28,28,29,29,29,30,30,30,                     // Multiplication factors (>23.5)
                                                  30,31,31,31,32,32,32,33,33,33,33,34,34,34,35,35,
                                                  35,36,36,36,36,37,37,37,38,38,38,39,39,39,40,40,
                                                  40,40,41,41,41,42,42,42,43,43,43,43,44,44,44,45,
                                                  45,45,46,46,46,46,47,47,47,48,48,48,49,49,49,49,
                                                  50,50,50,51,51,51,52,52,52,53,53,53,53,54,54,54,
                                                  55,55,55,56,56,56,56,57,57,57,58,58,58,59,59,59,
                                                  59,60,60,60,61,61,61,62,62,62,62,63,63,63,64,64};

byte            Amp_Freq_Adj_LOOKUP_LOW[108]  =   {28,29,29,29,30,30,30,31,31,31,32,32,32,33,33,33,                     // Multiplication factors (<23.5)
                                                  34,34,34,35,35,35,36,36,37,37,37,38,38,38,39,39,
                                                  39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,
                                                  45,45,45,46,46,46,47,47,47,48,48,48,49,49,49,50,
                                                  50,50,51,51,51,52,52,52,53,53,53,54,54,54,55,55,
                                                  55,56,56,56,57,57,57,58,58,58,59,59,59,60,60,60,
                                                  61,61,61,62,62,62,63,63,63,64,64,64};
            
int             Index_Phase         = 0;                        // Index incremented to step through the sinewave generation duty cycles
byte            Amplitude_Adj       = 32;                       // We multiply the duty cycle value by this then >> 6 (ie. /64) It's a CPU efficient way of scaling values 
byte            Step_Delay_us       = 125;                      
byte            State               = STATE_OFF;                // Either OFF, HOLD, RAMP or ON            
byte            ACS_Attached_YN     = 0;        
byte            Frequency           = START_FREQUENCY;          // This is not in hertz. The range is 0-127 which corrosponds to 20-50Hz
byte            Frequency_Limit     = MAX_FREQUENCY_HIGH;

unsigned long   TimeStamp_Sine      = 0;
unsigned long   TimeStamp_Ramp      = 0;
unsigned long   TimeStamp_Startup   = 0;
unsigned long   TimeStamp_Shutdown  = millis() - 42857;         // Trick into thinking we shut down 15 minutes ago so we only wait 5 minutes before trying a start up [42857]

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
    Serial.println("Performing ACS detection:");
  #endif
  
  delay(100);
  Detect_ACS();                                                 // Sets ACS_Attached_YN = 0 if not attached and 1 if attached
}

//-------------------------------------------------------------------------------------
void loop() 
{      
  if(millis() - TimeStamp_Ramp >= RAMP_TIMESTEP)                // Periodic function runs every RAMP_TIMESTEP milliseconds      
  {    
    TimeStamp_Ramp = millis();

    // #########################################################################################################################################
    // 1) Read the ADCs for input voltage and current. Keep 10 value running averages of both stored as SumV and SumI respectively
    static byte indexV = 0, indexI = 0;
    static int BufferV[10], BufferI[10];
    static int SumV, SumI;
    
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

    // #########################################################################################################################################
    // 2) State logic 
    if(ACS_Attached_YN) {                                       // If ACS712 is attached do this logic:
      if(State == STATE_ON) {                               
        if(SumI < NO_LOAD_I_THRESHOLD)  {                       // If no current is being drawn despite producing an output waveform, we shutdown
          Shutdown();
          #ifdef _DEBUG_MODE_        
            Serial.print("Low current! I: "); Serial.println(SumI);
          #endif 
        }    
        else if(SumV < LOW_VOLT_CUTOUT)  {                      // If the voltage has fallen too low then shutdown        
          Shutdown(); 
          #ifdef _DEBUG_MODE_        
            Serial.print("Undervoltage! V: "); Serial.println(SumV);
          #endif                  
        }        
      }
      else if(Frequency == Frequency_Limit) {                                            
        State = STATE_ON;
      }    
      else if(State == STATE_RAMP) {                            // If we're in the soft start stage increment our frequency if we're not at our limit yet
        Frequency++;
      } 
      else if(State == STATE_HOLD)  {
        if(millis() - TimeStamp_Startup > HOLD_TIME) {          // If we've been producing our 30Hz output for 1 second we'll have a good current draw average measurement
          if(SumI > NO_LOAD_I_THRESHOLD)  {                     // If there is current being drawn the fridge must be connected
            State = STATE_RAMP;                                 // So ramp up the frequency to power the fridge properly
          }
          else  {
            Shutdown();                                         // Otherwise shutdown for a further 20 minutes before trying again
            #ifdef _DEBUG_MODE_        
              Serial.print("Current draw test failed! I: "); Serial.println(SumI);
            #endif 
          }
        }
      }
      else if(State == STATE_OFF)  {
        delay(45);                              
        LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_ON);           // millis() doesn't count during this period! We must alter our SHUTDOWN_TIME define to account for it
        delay(5);
  
        #ifdef _DEBUG_MODE_        
          Serial.print("V: "); Serial.println(SumV);
          Serial.print("I: "); Serial.println(SumI);
          Serial.print("SD Time: "); Serial.println((millis() - TimeStamp_Shutdown)/48); Serial.println();
        #endif
  
        if(SumV > LOW_VOLT_CUTIN) {                             // If our battery voltage is good
          if(millis() - TimeStamp_Shutdown > SHUTDOWN_TIME) {   // And we've completed our [20] minute shutdown period
            Set_Max_Freq(SumV);                                 // Depending on the inverter input voltage, decide what our max fequency can be.
            Startup();                                          // Then try a startup procedure
          }
        }
      }      
    }
    
    else  {                                                     // If ACS NOT attached do this logic
      if(State == STATE_ON) {
        if(SumV < LOW_VOLT_CUTOUT)  {                                  
          Shutdown();   
          #ifdef _DEBUG_MODE_        
            Serial.print("Undervoltage! V: "); Serial.println(SumV);
          #endif     
        }  
      }
      else if(Frequency == MAX_FREQUENCY_HIGH) {                // Without ACS attached, we just run at max voltage and frequency            
        State = STATE_ON;
      }    
      else if(State == STATE_RAMP) {
        Frequency++;
      } 
      else if(State == STATE_HOLD)  {
        State = STATE_RAMP;
      }
      else if(State == STATE_OFF)  {
        delay(45);                              
        LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_ON); 
        delay(5);
  
        #ifdef _DEBUG_MODE_        
          Serial.print("V: "); Serial.println(SumV);
        #endif
  
        if(SumV > LOW_VOLT_CUTIN) {                             // If our battery voltage is good
          Startup();                                            // Then Startup
        }
      }      
    }

    // #########################################################################################################################################
    // 3) Write delay and amplitute values from lookup tables
    Step_Delay_us = Step_Delay_LOOKUP[Frequency];               // Step_Delay_us -> 20Hz - 50Hz 

    if(Frequency_Limit == MAX_FREQUENCY_HIGH) 
      Amplitude_Adj = Amp_Freq_Adj_LOOKUP_HIGH[Frequency];      // Change the amplitude proportional to the frequency. At 25Hz the amplitude should be half that of 50Hz
    else
      Amplitude_Adj = Amp_Freq_Adj_LOOKUP_LOW[Frequency];                    
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

// #########################################################################################################################################
// -------------------------------------------------------  SUB-ROUTINES -------------------------------------------------------------------
// #########################################################################################################################################
void Startup ()
{
  // Startup procedure: Turn the inverter on, then start sinewave generation    
  digitalWrite(INVERTER_CTRL_PIN, HIGH);
  digitalWrite(ACS_nENABLE, LOW);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  #ifdef _DEBUG_MODE_        
    Serial.println("Startup!");
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
    Serial.println("Shutdown!");
  #endif 

  State = STATE_OFF; 
  Frequency = START_FREQUENCY;     
  TCNT1 = 793;                                                          
  OCR1B = 0;    
  PORTC = PORTC & 0b11110111;  
  TimeStamp_Shutdown = millis();
}

void Detect_ACS ()
{
  // The ACS outputs Vdd/2 when no current is flowing. So if we get ADC readings of ~512 we know it's attached.
  static byte indexI = 0;
  static int BufferI[64];
  static long SumI;
  
  digitalWrite(ACS_nENABLE, LOW);                               // Power the ACS attached or not  
  while(indexI < 64)  {                                         // Sample the ADC pin 64 times to build up an average
     delay(5);

   if(ADCSRA & B00010000)  {                                    // If ADC conversion complete 
      BufferI[indexI++] = ADCW;                                 // Insert latest reading into Buffer and increment index
      ADCSRA |= B00010000;                                      // Clear conversion complete flag
      ADCSRA |= B01000000;                                      // Start another conversion in the background  
    }
  }

  SumI = 0;                                                     // Reset Sum
  for(byte i = 0; i < 64; i++)  {                               // Add up all values in buffer
    SumI += BufferI[i];
  }
  SumI /= 64;                                                   // Divide by number of values to get the average

  #ifdef _DEBUG_MODE_
    Serial.print("Average ACS ADC pin: "); Serial.println(SumI);
  #endif

  if(SumI > 412 && SumI < 612)                                  // Should be the case if the ACS is attached...
    ACS_Attached_YN = 1;  
  else  
    ACS_Attached_YN = 0;

  digitalWrite(ACS_nENABLE, HIGH);                              // Unpower the ACS attached or not
}

void Set_Max_Freq(int SumV)
{
  if(SumV > AMP_ADJUST_THRES) {
    Frequency_Limit = MAX_FREQUENCY_HIGH;
  }
  else  {
    Frequency_Limit = MAX_FREQUENCY_LOW;
  }  
}









