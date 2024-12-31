#include "Filter.h"

uint16_t n1000Sound[11] = {175,196,220,233,262,294,330,349,392,400,1000};

// Auto generated sine LUT from SinCosLUTGenerator.m, credit: Alec Marshall 
uint8_t sinTable[360] = {
128,130,132,134,136,139,141,
143,145,147,150,152,154,156,158,
160,163,165,167,169,171,173,175,
177,179,181,183,185,187,189,191,
193,195,197,199,201,202,204,206,
208,209,211,213,214,216,218,219,
221,222,224,225,227,228,229,231,
232,233,234,236,237,238,239,240,
241,242,243,244,245,246,247,247,
248,249,249,250,251,251,252,252,
253,253,253,254,254,254,255,255,
255,255,255,255,255,255,255,255,
255,254,254,254,253,253,253,252,
252,251,251,250,249,249,248,247,
247,246,245,244,243,242,241,240,
239,238,237,236,234,233,232,231,
229,228,227,225,224,222,221,219,
218,216,214,213,211,209,208,206,
204,202,201,199,197,195,193,191,
189,187,185,183,181,179,177,175,
173,171,169,167,165,163,160,158,
156,154,152,150,147,145,143,141,
139,136,134,132,130,128,125,123,
121,119,116,114,112,110,108,105,
103,101,99,97,95,92,90,88,
86,84,82,80,78,76,74,72,
70,68,66,64,62,60,58,56,
54,53,51,49,47,46,44,42,
41,39,37,36,34,33,31,30,
28,27,26,24,23,22,21,19,
18,17,16,15,14,13,12,11,
10,9,8,8,7,6,6,5,
4,4,3,3,2,2,2,1,
1,1,0,0,0,0,0,0,
0,0,0,0,0,1,1,1,
2,2,2,3,3,4,4,5,
6,6,7,8,8,9,10,11,
12,13,14,15,16,17,18,19,
21,22,23,24,26,27,28,30,
31,33,34,36,37,39,41,42,
44,46,47,49,51,53,54,56,
58,60,62,64,66,68,70,72,
74,76,78,80,82,84,86,88,
90,92,95,97,99,101,103,105,
108,110,112,114,116,119,121,123,
125};

// Maximum value in sin table
uint8_t sinTableMax = 255;

// Potentially generate new table to iterate over?
// uint8_t sinInterpTable[369] = {0};

// Create filter for analog frequency input knob to remove value bouncing
ExponentialFilter<long> freqKnobFilter(1, 1);

// Instantiate functions definitions
void setCarrierFreq(uint16_t fCarrier);
void setupTimers(uint16_t fCarrier);
void setupPins();
void startPWM();
void stopPWM();
uint16_t map_uint16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

// Arduino CPU Clock (hz)
#define CPUCLOCK 16000000

// Analog input to adjust frequency
#define FREQKNOB A0

// Deadtime in nanoseconds between switching high and low to prevent transistor shoothrough
// Minimum value is 100
long deadTimeNanos = 1000;

// Deadtime in timer counts (calculated in setCarrierFrequency function)
uint16_t deadTimeCycles;

// Used for calculations of wave position (wavetable length)
uint16_t nSine = sizeof(sinTable)/sizeof(sinTable[0]);

// Default motor drive parameters
float carrierFreq = 1000; // Switching frequency (hz)
float driveFreq = 1; // Drive (hz)
float prevDriveFreq = driveFreq;
float dutyCycle = 100; // (%) of max (used for voltage conversion)
bool onePhase = true;

// Calculate number of required ISR cycles to produce 1 period of wave
volatile uint16_t nISR = carrierFreq/driveFreq;
// Calculate sine table step size
volatile float sineStep = ((float)nSine/(float)nISR);
// Scaling factor for sine table
volatile float scaleFactor = 1;
// Index variable for ISR
volatile uint16_t sineIndex = 0;

// Serial buffer size
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char commandFromSerial[numChars] = {0};
int integerFromSerial = 0;
float floatFromSerial = 0.0;

uint8_t prevCarrierFreqState = 0;

uint8_t prevpwmState = 0;

void setup() {
  digitalWrite(22,LOW);
  Serial.begin(115200);
  Serial.println("Motor drive online");
  Serial.print("Sine array size: ");
  Serial.println(nSine);
  Serial.print("Number of ISR Cycles: ");
  Serial.println(nISR);
  Serial.print("Sine stepsize: ");
  Serial.println(sineStep);
  Serial.println("Resetting, syncing, and configuring timers");
  // Sets up output and input pin configs
  setupPins();
  // Sets up timers with default carrier frequency and outputs disabled
  setupTimers(carrierFreq);
}

void loop() {
  recvSerCommand();
  if (strcmp("p", commandFromSerial) == 0){
    if(integerFromSerial == 1){
      Serial.println("PWM started");
      startPWM();
    }
    else if(integerFromSerial == 0){
      Serial.println("PWM stopped");
      stopPWM();
    }
  }

  uint8_t pwmState = PINB & (1<<PB7);
  if (pwmState != 0 && pwmState != prevpwmState){
    stopPWM();
  }
  else if(pwmState == 0 && pwmState != prevpwmState){
    startPWM();
  }

  // Filter current knob position
  freqKnobFilter.Filter(analogRead(FREQKNOB));
  driveFreq = ((float)map(freqKnobFilter.Current(),1,1020,1,100));
  
  uint8_t carrierFreqState = map(analogRead(A1),1,1010,0,10);

  if (carrierFreqState != prevCarrierFreqState){
    setCarrierFreq(n1000Sound[carrierFreqState]);
  }

  prevCarrierFreqState = carrierFreqState;

  prevpwmState = pwmState;

}

// void driveSPWM(float fDrive, float fCarrier, float dutyCycle){

// }

// Timer 1 TOP overflow ISR for updating timer CMP values
ISR(TIMER3_OVF_vect){

  // Define constant value to scale sine table by based on DC and carrier frequency
  float constant = scaleFactor*(dutyCycle/100.0);

  // Deterimine U phase sin LUT index
  uint16_t uPhaseTableIndex = ((float)sineIndex*sineStep);
  // Look up primary sine function value for u phase's duty cycle
  uint16_t uPhaseSin = ((float)sinTable[uPhaseTableIndex]*constant);

  if(onePhase == false){

    // V phase is offset by 120 degrees from U Phase, wrap around if too large
    uint16_t vPhaseTableIndex = (uPhaseTableIndex < 240) ? (uPhaseTableIndex + 120):(uPhaseTableIndex - 240);
    // W phase is offset by 120 degrees from V Phase, wrap around if too large
    uint16_t wPhaseTableIndex = (vPhaseTableIndex < 240) ? (vPhaseTableIndex + 120):(vPhaseTableIndex - 240);

    // Look up remaining sine function values for V and W phases' duty cycle
    uint16_t vPhaseSin = ((float)sinTable[vPhaseTableIndex]*constant);
    uint16_t wPhaseSin = ((float)sinTable[wPhaseTableIndex]*constant);

    // Timer 3 compare registers set high side duty cycle for each phase
    OCR3A = (uPhaseSin < deadTimeCycles) ? 0:uPhaseSin;
    OCR3B = (vPhaseSin < deadTimeCycles) ? 0:vPhaseSin;
    OCR3C = (wPhaseSin < deadTimeCycles) ? 0:wPhaseSin;

    // Timer 4 compare registers set low side duty cycle for each pulse with deadtime compensation
    OCR4A = (ICR3-OCR3A < deadTimeCycles) ? ICR3:(OCR3A + deadTimeCycles);
    OCR4B = (ICR3-OCR3B < deadTimeCycles) ? ICR3:(OCR3B + deadTimeCycles);
    OCR4C = (ICR3-OCR3C < deadTimeCycles) ? ICR3:(OCR3C + deadTimeCycles);

  }

  else if(onePhase == true){

    // V phase is offset by 180 degrees from U Phase, wrap around if too large
    uint16_t vPhaseTableIndex = (uPhaseTableIndex < 180) ? (uPhaseTableIndex + 180):(uPhaseTableIndex - 180);
    
    // Look up remaining sine function value for V phase's duty cycle
    uint16_t vPhaseSin = ((float)sinTable[vPhaseTableIndex]*constant);

    OCR3A = (uPhaseSin < deadTimeCycles) ? 0:uPhaseSin;
    OCR3B = (vPhaseSin < deadTimeCycles) ? 0:vPhaseSin;
    OCR3C = 0;

    OCR4A = (ICR3-OCR3A < deadTimeCycles) ? ICR3:(OCR3A + deadTimeCycles);
    OCR4B = (ICR3-OCR3B < deadTimeCycles) ? ICR3:(OCR3B + deadTimeCycles);
    OCR4C = 0;
  }

  if (driveFreq != prevDriveFreq){
    // Calculate number of required ISR cycles to produce 1 period of wave
    nISR = (CPUCLOCK/(2*ICR3))/driveFreq;
    // Calculate sine table step size
    sineStep = ((float)nSine/(float)nISR);
    // Calculate nearest ISR sine index
    sineIndex = ((float)uPhaseTableIndex/sineStep);
  }
  prevDriveFreq = driveFreq;

  //Increment sine index, reset if larger than next index
  sineIndex ++;
  if(sineIndex >= nISR){
    sineIndex = 0;
    // Generate temporary trigger signal
    PORTB ^= (1<<PB6);
  }
}

void setupPins(){
  // Ext sync pin for oscilloscope
  pinMode(12, OUTPUT);

  pinMode(13, INPUT_PULLUP);

  pinMode(22, OUTPUT);

  // Configure timer pins as outputs for inverter

  /*---------------PIN MAP------------------
  |           | U Phase | V Phase | W Phase |
  | High Side |  Pin 5  |  Pin 2  |  Pin 3  |
  | Low Side  |  Pin 6  |  Pin 7  |  PIn 8  |
  */

  // Set high side pins as output
  // Pin 5 (PE3) is OC3A, Pin 2 (PE4) is OC3B, Pin 3 (PE5) is OC3C
  DDRE = DDRE | (0b00111000);  // Timer 3 OC3A, OC3B and OC3C pins

  // Set low side pins as output
  // Pin 6 (PH3) is OC4A, Pin 7 (PH4) is OC4B, Pin 8 (PH5) is OC4C
  DDRH = DDRH | (0b00111000);  // Timer 4 OC4A, OC4B and OC4C pins
    Serial.println("Set timer pins as outputs");
}

void setupTimers(uint16_t fCarrier){
  // Disable interrupts
  cli();

  // Set synchronization register - GTCCR
  GTCCR = 0b10000011; // Halt and Sync All Timers
  Serial.println("All timers halted and synced");

  // Reset all relevant timer count values
  TCNT3 = 0;
  TCNT4 = 0;

  Serial.println("Count values reset");

  // Reset Timer 3 parameter register A, Mode 8 set partially by last two zero WGM bits
  TCCR3A = 0b00000000;

  // Set up Timer 3 paramteter register B - TCCR1B (Input capture not configured)
  TCCR3B = 0b00000000;
  TCCR3B = (1<<WGM33)|(1<<CS30);

  // Set up Timer 3 Interrupt Mask Register TIMSK3 - TOIE3 bit allows TOV3 flag to be used for triggering an interrupt
  // Using Timer 1 for interrupts breaks Serial.print statements
  TIMSK3 = (1<<TOIE3);

  // Reset Timer 4 parameter register A, Mode 8 set partially by last two zero WGM bits
  TCCR4A = 0b00000000;

  // Set up Timer 4 paramteter register B - TCCR1B (Input capture not configured)
  TCCR4B = 0b00000000;
  TCCR4B = (1<<WGM43)|(1<<CS40);


  Serial.println("Timer settings configured");

  // Reset output compare values for each timer (OCRnA/B)
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  // Set carrier frequency default - required to run timers
  setCarrierFreq(fCarrier);

  sei();

  Serial.println("Timer in halted state, ready to run");

}

void setCarrierFreq(uint16_t fCarrier){
  
  carrierFreq = fCarrier;

  // Disable interrupts temporarily to prevent possible timer glitches or TOP value corruption
  // cli();

  // Calculate TOP value to set for given carrier frequency (defines wave period)
  uint16_t TOP = CPUCLOCK/(2*fCarrier); // Equation from section 17.9.5 of atmega2560 datasheet

  // Set all TOP values for each timer (overflow value, defines frequency, ICRn defines this in Mode 8)
  ICR3 = TOP;
  ICR4 = TOP;

  Serial.print("Top value set to: ");
  Serial.println(TOP);

  // Calculate appropriate number of deadtime cycles to create the right deadtime in microseconds
  deadTimeCycles = (deadTimeNanos<100 ? 100:deadTimeNanos)*(16.0/1000.0);

  Serial.print("Deadtime in Cycles:");
  Serial.println(deadTimeCycles);

  // Serial.print("Deadtime value set to: ");
  // Serial.println(deadTimeCycles);

  // Sine table max value does not match the max value of TOP - scaleFactor defines this value 
  scaleFactor = (float)TOP/(float)sinTableMax; // Calculate scaling factor for sine table

  // Serial.print("Scale factor set to:");
  // Serial.println(scaleFactor);

  // Enable interrupts
  // sei();
}

void startPWM(){
  digitalWrite(22,HIGH);
  delayMicroseconds(65535);
  GTCCR = 0b10000011; // Halt and Sync All Timers

  // Reset all relevant timer count values
  TCNT3 = 0;
  TCNT4 = 0;

  GTCCR = 0b00000011; // Free synced timers (Prescaler syncs will automatically clear)

  // Enable all timer outputs with compare mode output settings:
  TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|(1<<COM3C1)|(0<<COM3C0);
  TCCR4A = (1<<COM4A1)|(1<<COM4A0)|(1<<COM4B1)|(1<<COM4B0)|(1<<COM4C1)|(1<<COM4C0);
  Serial.println("Timer running");
}

void stopPWM(){
  digitalWrite(22,LOW);

  GTCCR = 0b10000011; // Halt and Sync All Timers

  // Disable all timer outputs with compare mode output settings:
  TCCR3A = 0b00000000;
  TCCR4A = 0b00000000;

   // Reset output compare values for each timer (OCRnA/B)
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  // Reset all relevant timer count values
  TCNT3 = 0;
  TCNT4 = 0;
}