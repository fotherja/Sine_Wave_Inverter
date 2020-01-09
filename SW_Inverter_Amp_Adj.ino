/*
  J.Fotherby 29th December 2019
  
  Sinewave inverter: Amplitude and frequency adjustable 
*/
 
//------------------------------------------------------------------------------------- 
//------------------------------------------------------------------------------------- 
#define         PHASE_A             9
#define         PHASE_B             10

#define         FRQ_ADJ             A0
#define         AMP_ADJ             A1

#define         SINE_STEPS          256

//-------------------------------------------------------------------------------------
unsigned int    Sine_Lookup[128]    =             {0,20,39,59,78,98,117,137,156,175,194,213,232,251,269,288,
                                                  306,324,342,359,377,394,411,427,444,460,476,492,507,522,537,551,
                                                  565,579,592,605,618,630,642,653,664,675,685,695,705,714,722,730,
                                                  738,745,752,759,765,770,775,780,784,787,790,793,795,797,798,799,
                                                  799,799,798,797,795,793,790,787,784,780,775,770,765,759,752,745,
                                                  738,730,722,714,705,695,685,675,664,653,642,630,618,605,592,579,
                                                  565,551,537,522,507,492,476,460,444,427,411,394,377,359,342,324,
                                                  306,288,269,251,232,213,194,175,156,137,117,98,78,59,39,20};

int             Index_Phase         = 255;
unsigned int    Delay_Adj           = 0;
byte            Amplitude_Adj       = 16;
unsigned long   TimeStamp           = 0;
unsigned int    Next_OCR1A          = 0;             

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
}

//-------------------------------------------------------------------------------------
void loop() 
{
  TimeStamp = micros() + 100;
  
  while(1)
  {   
    // 1) Increment the step counter to vary our duty cycle of the PWM signal and modulate a sinewave
    Index_Phase += 1;
    if(Index_Phase == SINE_STEPS)  {
      Index_Phase = 0;
    }

    
    // 2) Have a constant delay in our loop for stepping through our sinewave generation
    TimeStamp += Delay_Adj;
    while(micros() < TimeStamp) {}      
    
    
    // 3) Vary the duty cycle on PHASE_A. PHASE_B is a 50Hz square wave. Simultaneously flip our PHASE_A & B lines at the zero crossing points (0 & 180 degrees)
    if(Index_Phase == 0)  {         
      TCNT1 = 754;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go low simultaneously.              
      OCR1A = 0;    
      PORTB = PORTB & 0b11111011;                               // ie. digitalWrite(PHASE_B, LOW);
      Next_OCR1A = (20 * Amplitude_Adj) >> 5;                   // The index will be 1 next which corrosponds to a lookup value of 20
      continue;
    }
    else if(Index_Phase < 128)  {
      OCR1A = Next_OCR1A;                                       // We can immediately write to this register because we pre-calculated its value
      Next_OCR1A = (Sine_Lookup[Index_Phase] * Amplitude_Adj) >> 5;
    }    
    else if(Index_Phase == 128) {         
      TCNT1 = 793;                                              // Allow a few clock cycles to complete the following code so PHASE_A & PHASE_B go high simultaneously.  
      OCR1A = 799;  
      PORTB = PORTB | 0b00000100;                               // ie. digitalWrite(PHASE_B, HIGH);
      Next_OCR1A = 799 - ((20 * Amplitude_Adj) >> 5);        
      continue;                  
    }
    else  {
      OCR1A = Next_OCR1A;
      Next_OCR1A = 799 - ((Sine_Lookup[Index_Phase - 128] * Amplitude_Adj) >> 5);
    }  
    
  
    // 4) Frequency and Amplitude adjust:
    if(Index_Phase == 63) {                                     // 100us to do an ADC reading! That's a couple steps in the index. Can afford to do this at indices 63 or 191  
      unsigned int ADC_Read = analogRead(FRQ_ADJ);                                                       // -> then play catchup since the top/bottom of the sinewave is flat
      Delay_Adj = 65 + (ADC_Read >> 3);                         // Delay_Adj      -> 65 - 192 
    }                                                           //                => 60Hz - 20Hz
    else if(Index_Phase == 191)  {
      unsigned int ADC_Read = analogRead(AMP_ADJ) + 16;         // ADC            -> 16-1039
      Amplitude_Adj = (ADC_Read >> 5);                          // Amplitude_Adj  -> 0-32
      Amplitude_Adj = max(Amplitude_Adj, 8);                    // Amplitude_Adj  -> 8-32
    }
  }
}







