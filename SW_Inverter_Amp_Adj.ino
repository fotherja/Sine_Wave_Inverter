/*
  J.Fotherby 2020  
  Sinewave inverter: Amplitude and frequency adjustable 

  - When circuit powers on it ramps the AC frequency from 30 to 50Hz adjusting the amplitude proportionally. 

  To Do:
   - Get amplitude compensation for changes in input voltage. This will definitely help to reduce power consumption.
   
   - Start programming the logic of the circuit:
    - Do nothing for 1 minute after turn on
    - Then every 5 minutes do a load connected test -> Apply power and see whether it causes any current flow.
    - If current is being drawn, continue to power the load. If no current is drawn, enter a low power shutdown mode and switch off the inverter
    - If the current sensor measurements ever exceed a peak value, immediately shutoff the inverter. Retry 5 minutes later.


  # ADC 0 -> Used for measuring input voltage and slightly adjusting the output amplitude to compensate for fluctuations
  # ADC 2 -> Used to measure the output current for load connected testing and overcurrent protection
*/
 
//------------------------------------------------------------------------------------- 
//------------------------------------------------------------------------------------- 
#define         CURRENT_OUT         A0
#define         INVERTER_CTRL_PIN   A1
#define         VOLTAGE_IN          A2
#define         PHASE_B_PIN         A3

#define         LED_PIN             9
#define         PHASE_A_PIN         10

#define         SINE_STEPS          256
#define         UPPER_ADC_V_VAL     980
#define         LOWER_ADC_V_VAL     725

#define         STATE_OFF           0
#define         STATE_RAMP          1
#define         STATE_ON            2

#define         START_FREQUENCY     43
#define         RAMP_TIMESTEP       50

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

                                                
int             Index_Phase         = 255;                      // Index incremented to step through the sinewave generation duty cycles
byte            Amplitude_Adj       = 32;                       // We multiply the duty cycle value by this then >> 6 (ie. /64) It's a CPU efficient way of scaling values 
byte            Step_Delay_us       = 125;                      
byte            State               = STATE_RAMP;               // Either OFF, RAMP or ON            
byte            Frequency           = START_FREQUENCY;          // This is not in hertz. The range is 0-127 which corrosponds to 20-50Hz

unsigned long   TimeStamp_Sine      = 0;
unsigned long   TimeStamp_Ramp      = 0;
unsigned int    Next_OCR1B          = 0;   

unsigned int    ADC_Raw_Temp_Val;  
unsigned int    ADC_Raw_Volt_Val;

//-------------------------------------------------------------------------------------
void setup() 
{          
  pinMode(PHASE_A_PIN, OUTPUT);   
  pinMode(PHASE_B_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(INVERTER_CTRL_PIN, OUTPUT);
    
  digitalWrite(PHASE_A_PIN, LOW);    
  digitalWrite(PHASE_B_PIN, LOW); 
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(INVERTER_CTRL_PIN, HIGH); 

  delay(2000);                                                  // Delay a bit to allow the inverter high voltage capacitor to charge up  

  TCCR1A = 0b00100010;
  TCCR1B = 0b00011001;                                          // 20KHz FAST PWM with ICR1 control
  ICR1 = 799;
  TCNT1 = 0; 
  OCR1B = 0;

  ADMUX = B01000000;                                            // Turn on ADC
  ADCSRA = B10000111;                                           // 128 Prescaler -> ADC Clk = 125KHz
  ADCSRA |= B01000000;                                          // Start a conversion in the background
  
  Serial.begin(115200);
  Serial.println("starting");
  delay(100);
  
  TimeStamp_Sine = micros();
}

//-------------------------------------------------------------------------------------
void loop() 
{     
  // 1) Increment the step counter to vary our duty cycle of the PWM signal and modulate a sinewave
  Index_Phase += 1;
  if(Index_Phase >= SINE_STEPS)  {
    Index_Phase = 0;
  }

  // 2) See if any ADC conversions are complete and act accordingly:  
  if(millis() - TimeStamp_Ramp >= RAMP_TIMESTEP)
  {    
    TimeStamp_Ramp = millis();
    
    if(ADCSRA & B00010000)  {                                   // If ADC conversion complete 
      if(ADMUX == B01000010) {                                  // ADC1 conversion complete
        ADC_Raw_Temp_Val = ADCL | (ADCH << 8);                          
        
        ADMUX = B01000000;                                      // Select ADC channel 0 this time       
        ADCSRA |= B00010000;                                    // This clears the ADC complete flag and allows the new MUX channel to be written
        ADCSRA |= B01000000;                                    // Start a conversion in the background      
      }
      else if(ADMUX == B01000000)  {                            // ADC0 conversion complete
        ADC_Raw_Volt_Val = ADCL | (ADCH << 8);                  // The voltage will only vary between 20-26volts at most = 222 ADC counts with a 2.2K || 10K divider. Make 256 the range
 
        ADMUX = B01000010;                                      // Select ADC channel 1    
        ADCSRA |= B00010000;
        ADCSRA |= B01000000;                                    // Start a conversion in the background                        
      }
    }


    // Logic
    if(Frequency == 127) {
      State = STATE_ON;
    }    
    else if(State == STATE_RAMP) {
      Frequency++;
    }    


//    // Startup procedure: Turn the inverter on, then start sinewave generation    
//    digitalWrite(Inverter_On_Off, HIGH);
//    TimeStamp_Sine = micros();
//    Index_Phase = 0; 
//    State = STATE_RAMP;
//    Frequency = START_FREQUENCY;
//      
//    // Shutdown procedure: Turn off the inverter, then ensure Phase A&B both go low simultaneously 
//    digitalWrite(INVERTER_CTRL_PIN, LOW); 
//    State = STATE_OFF;      
//    TCNT1 = 793;                                                          
//    OCR1B = 0;    
//    PORTC = PORTC & 0b11110111;                             
    


    Step_Delay_us = Step_Delay_LOOKUP[Frequency];               // Step_Delay_us -> 20Hz - 50Hz 
    Amplitude_Adj = Amp_Freq_Adj_LOOKUP[Frequency];             // Change the amplitude proportional to the frequency. At 25Hz the amplitude should be half that of 50Hz        
  }

  
  // 3) Have a constant delay in our loop for stepping through our sinewave generation
  while(micros() - TimeStamp_Sine < Step_Delay_us) {}      
  TimeStamp_Sine += Step_Delay_us;
  
  
  // 4) Vary the duty cycle on PHASE_A. PHASE_B is a 50Hz square wave. Simultaneously flip our PHASE_A & B lines at the zero crossing points (0 & 180 degrees)
  if(State != STATE_OFF)
  { 
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
  }
}


















        //Amplitude_Adj *= 22v / Vin 
        //In_Voltage_Val = min(ADC_Raw_Volt_Val, UPPER_ADC_V_VAL);
        //In_Voltage_Val = max(In_Voltage_Val, LOWER_ADC_V_VAL);
        //In_Voltage_Val -= LOWER_ADC_V_VAL; 
        //In_Voltage_Val >>= 





//  Step_Delay_us = Step_Delay_LOOKUP[127];               // Step_Delay_us -> 20Hz - 50Hz 
//  Amplitude_Adj = Amp_Freq_Adj_LOOKUP[127];             // Change the amplitude proportional to the frequency. At 25Hz the amplitude should be half that of 50Hz
//  State = STATE_ON;



