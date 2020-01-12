/*
  J.Fotherby 29th December 2019
  
  Sinewave inverter: Amplitude and frequency adjustable 
*/
 
//------------------------------------------------------------------------------------- 
//------------------------------------------------------------------------------------- 
#define         PHASE_A             9
#define         PHASE_B             10

#define         VOLTAGE_IN          A0
#define         TEMPERATURE         A1

#define         SINE_STEPS          256

//-------------------------------------------------------------------------------------
unsigned int    Sine_Lookup[128]          =       {0,20,39,59,78,98,117,137,156,175,194,213,232,251,269,288,
                                                  306,324,342,359,377,394,411,427,444,460,476,492,507,522,537,551,
                                                  565,579,592,605,618,630,642,653,664,675,685,695,705,714,722,730,
                                                  738,745,752,759,765,770,775,780,784,787,790,793,795,797,798,799,
                                                  799,799,798,797,795,793,790,787,784,780,775,770,765,759,752,745,
                                                  738,730,722,714,705,695,685,675,664,653,642,630,618,605,592,579,
                                                  565,551,537,522,507,492,476,460,444,427,411,394,377,359,342,324,
                                                  306,288,269,251,232,213,194,175,156,137,117,98,78,59,39,20};

byte            Step_Delay_LOOKUP[128]    =       {195,193,191,189,187,184,182,180,178,177,175,173,171,169,168,166,
                                                  164,163,161,160,158,156,155,154,152,151,149,148,147,145,144,143,
                                                  142,141,139,138,137,136,135,134,133,132,131,130,129,128,127,126,
                                                  125,124,123,122,121,120,119,118,118,117,116,115,114,114,113,112,
                                                  111,110,110,109,108,108,107,106,106,105,104,104,103,102,102,101,
                                                  100,100,99,99,98,97,97,96,96,95,95,94,94,93,93,92,
                                                  92,91,91,90,90,89,89,88,88,87,87,86,86,85,85,85,
                                                  84,84,83,83,82,82,82,81,81,80,80,80,79,79,78,78};

byte            Amp_Freq_Adj_LOOKUP[128]  =       {26,26,26,27,27,27,27,28,28,28,29,29,29,30,30,30,
                                                  30,31,31,31,32,32,32,33,33,33,33,34,34,34,35,35,
                                                  35,36,36,36,36,37,37,37,38,38,38,39,39,39,40,40,
                                                  40,40,41,41,41,42,42,42,43,43,43,43,44,44,44,45,
                                                  45,45,46,46,46,46,47,47,47,48,48,48,49,49,49,49,
                                                  50,50,50,51,51,51,52,52,52,53,53,53,53,54,54,54,
                                                  55,55,55,56,56,56,56,57,57,57,58,58,58,59,59,59,
                                                  59,60,60,60,61,61,61,62,62,62,62,63,63,63,64,64};

                                                
int             Index_Phase         = 255;
byte            Amplitude_Adj       = 32;
byte            Perform_Next_ADC    = 0;
byte            Step_Delay_us       = 0;
unsigned long   TimeStamp           = 0;
unsigned int    Next_OCR1A          = 0;   

unsigned int    ADC_Raw_Temp_Val; 
unsigned int    ADC_Raw_Freq_Adj; 
unsigned int    ADC_Raw_Volt_Val;

unsigned int    In_Voltage_Val;


//-------------------------------------------------------------------------------------
void setup() 
{          
  pinMode(PHASE_A, OUTPUT);   
  pinMode(PHASE_B, OUTPUT);
    
  digitalWrite(PHASE_A, LOW);    
  digitalWrite(PHASE_B, LOW); 

  TCCR1A = 0b10000010;
  TCCR1B = 0b00011001;                                        // 20KHz FAST PWM with ICR1 control
  ICR1 = 799;
  TCNT1 = 0; 
  OCR1A = 0;

  ADMUX = B01000000;                                          // Turn on ADC
  ADCSRA = B10000111;                                         // 128 Prescaler -> ADC Clk = 125KHz
  ADCSRA |= B01000000;                                        // Start a conversion in the background

  TimeStamp = micros() + 100;  
  Serial.begin(115200);
}

//-------------------------------------------------------------------------------------
void loop() 
{     
  // 1) Increment the step counter to vary our duty cycle of the PWM signal and modulate a sinewave
  Index_Phase += 1;
  if(Index_Phase == SINE_STEPS)  {
    Index_Phase = 0;
  }


  // 2) See if any ADC conversions are complete and act accordingly:
  Perform_Next_ADC++;
  if(Perform_Next_ADC == 250) {
    Perform_Next_ADC = 0;
    
    if(ADCSRA & B00010000)  {                                   // True if ADC conversion complete 
      if(ADMUX & 1) {                                           // If the channel is ADC1
        ADC_Raw_Freq_Adj = ADCL | (ADCH << 8);                  // The desired frequency to run at 0-1023
        ADC_Raw_Freq_Adj >>= 3;                                  // Now between 0-127

        Serial.println(ADC_Raw_Freq_Adj);
        Step_Delay_us = Step_Delay_LOOKUP[ADC_Raw_Freq_Adj];    // Step_Delay_us -> 20Hz - 50Hz 
        Amplitude_Adj = Amp_Freq_Adj_LOOKUP[ADC_Raw_Freq_Adj];  // Change the amplitude proportional to the frequency. At 25Hz the amplitude should be half that of 50Hz                               
  
        ADMUX = B01000000;                                      // Select ADC channel 0 this time       
        ADCSRA |= B00010000;                                    // This clears the ADC complete flag and allows the new MUX channel to be written
        ADCSRA |= B01000000;                                    // Start a conversion in the background      
      }
      else  {     
        ADC_Raw_Volt_Val = ADCL | (ADCH << 8);                  // 22v ~= 700, 25v ~= 850
        In_Voltage_Val = min(ADC_Raw_Volt_Val, 900);
        In_Voltage_Val = max(In_Voltage_Val, 700);
        In_Voltage_Val -= 700;     

        Serial.println(In_Voltage_Val);
  
        ADMUX = B01000001;                                      // Select ADC channel 1    
        ADCSRA |= B00010000;
        ADCSRA |= B01000000;                                    // Start a conversion in the background                        
      }
    }
  }

  
  // 3) Have a constant delay in our loop for stepping through our sinewave generation
  TimeStamp += Step_Delay_us;
  while(micros() < TimeStamp) {}      
  
  
  // 4) Vary the duty cycle on PHASE_A. PHASE_B is a 50Hz square wave. Simultaneously flip our PHASE_A & B lines at the zero crossing points (0 & 180 degrees)
  if(Index_Phase == 0)  {         
    TCNT1 = 793;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go low simultaneously.              
    OCR1A = 0;    
    PORTB = PORTB & 0b11111011;                               // ie. digitalWrite(PHASE_B, LOW);
    Next_OCR1A = (20 * Amplitude_Adj) >> 6;                   // The index will be 1 next which corrosponds to a lookup value of 20
  }
  else if(Index_Phase < 128)  {
    OCR1A = Next_OCR1A;                                       // We can immediately write to this register because we pre-calculated its value
    Next_OCR1A = (Sine_Lookup[Index_Phase + 1] * Amplitude_Adj) >> 6;
  }    
  else if(Index_Phase == 128) {         
    TCNT1 = 793;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go high simultaneously.  
    OCR1A = 799;  
    PORTB = PORTB | 0b00000100;                               // ie. digitalWrite(PHASE_B, HIGH);
    Next_OCR1A = 799 - ((20 * Amplitude_Adj) >> 6);                          
  }
  else  {
    OCR1A = Next_OCR1A;
    Next_OCR1A = 799 - ((Sine_Lookup[Index_Phase - 127] * Amplitude_Adj) >> 6);
  }                                                             
}







