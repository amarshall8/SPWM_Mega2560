// uint16_t n1000Sound[11] = {175,196,220,233,262,294,330,349,392,400,1000};  // Simulates (roughly) the Keikyu N1000 do-re-mi Siemens drive (no SHE at end)
uint16_t r160Sound[5] = {380,665,1290,1430,1245};   // Simulates the R160A/B Alstrom drive from NYC

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
#define sinTableMax 255

// Instantiate functions definitions
void setCarrierFreq(uint16_t fCarrier);
void setupTimers(uint16_t fCarrier);
void setupPins();
void startPWM();
void stopPWM();

// Arduino CPU Clock (hz)
#define CPUCLOCK 16000000

// Gate enable pin
#define GATEENPIN 22

// Deadtime in nanoseconds between switching high and low to prevent transistor shoothrough
// Minimum value is 100
long deadTimeNanos = 1000;

// Deadtime in timer counts (recalculated in setCarrierFrequency function)
uint16_t deadTimeCycles = (deadTimeNanos<100 ? 100:deadTimeNanos)*(16.0/1000.0);

// Used for calculations of wave position (wavetable length)
uint16_t nSine = sizeof(sinTable)/sizeof(sinTable[0]);

// Initialized motor drive parameters
float driveFreq = 0.5;  // Drive (hz)
float prevDriveFreq = driveFreq;  // Keeps track of previous drive frequency so it is not updated unnecessarily
float vfRatio = 230.0/60.0;       // V/f ratio set by motor parameters (change to be updatable over serial)
float vRMS_DCBUS = 60/sqrt(2);    // Approximate RMS voltage of the DC Bus
float dutyCycle = (vfRatio*driveFreq/vRMS_DCBUS)*100; // (%) of max DC bus voltage (used for V/f drive)
bool onePhase = false; // Switches drive between 3 (false) and 1 (true) phase operation
bool dir3Ph = false; // Reverses drive output
bool motorRunning = false; // Motor state variable

// Initial acceleration parameters
float targetFreq = 0.5;  // Target freq variable;
float kAccel = 0.1;      // Acceleration constant (freq/code execution)
// float deltaT = 0;        // Period for ISR, dependent variable, changes with carrier freq
uint8_t accelIndex = 0;  // Acceleration index for changing frequency updates based on carrier freq

// Calculate number of required ISR cycles to produce 1 period of wave
volatile uint16_t nISR;
// Calculate sine table step size
volatile float sineStep;
// Scaling factor for sine table
volatile float scaleFactor;
// Index variable for ISR
volatile uint16_t sineIndex = 0;
// Initialize U Phase table LUT index
volatile uint16_t uPhaseTableIndex = 0;

// Serial control variables
const byte numChars = 32;        // Incoming data buffer
char receivedChars[numChars];    // Recieved characters from port
char tempChars[numChars];        // temporary array for use when parsing

// Variables to hold the parsed data
char commandFromSerial[numChars] = {0}; // Char array (command)
int integerFromSerial = 0;              // Integer recieved from serial command
float floatFromSerial = 0.0;            // Float recieved from serial command

// Keeps track of previous carrier frequency so it is not updated unnecessarily
uint8_t carrierFreqState = 0;

// Keeps track of previous carrier frequency array state (eg. n160Sound, etc...)
uint8_t prevCarrierFreqState = 0;

// 
uint8_t prevpwmState = 0;

bool prevMotorState = false;

void setup() {
  // Enable gate drivers
  digitalWrite(GATEENPIN,LOW);
  // Print out initial parameters
  Serial.begin(115200);
  Serial.println("Motor drive online");
  Serial.print("Sine array size: ");
  Serial.println(nSine);
  Serial.print("Number of ISR Cycles: ");
  Serial.println(nISR);
  Serial.print("Sine stepsize: ");
  Serial.println(sineStep);
  Serial.println("Resetting, syncing, and configuring timers");
  // Prevents duty cycle from being outside range and sets duty cycle higher if motor is commanded to low power state
  dutyCycle = (dutyCycle < 10) ? 50:(dutyCycle > 100 ? 100:dutyCycle);
  // Sets up output and input pin configs
  setupPins();
  // Sets up timers with default carrier frequency and outputs disabled
  setupTimers(r160Sound[0]);
}

void loop() {
  // Parse serial commands
  recvSerCommand();
  // "p" commands the power of the inverter, an integer of 1 turns it on and 0 turns it off
  if (strcmp("p", commandFromSerial) == 0){
    if(integerFromSerial == 1){
      startPWM();
    }
    else if(integerFromSerial == 0){
      stopPWM();
    }
  }
  // "d" commands the direction of the output when in 3 phase mode, 1 rotates CCW, 0 rotates CW
  else if (strcmp("d", commandFromSerial) == 0){
    if(integerFromSerial == 1){
      dir3Ph = true;
    }
    else if (integerFromSerial == 0){
      dir3Ph = false;
    }
  }
  // "v" commands the duty cycle or voltage of the motor. This is set with a floating point value between 0.1 and 100 (no integer required)
  else if (strcmp("v", commandFromSerial) == 0){
    dutyCycle = (floatFromSerial > 100) ? 100:floatFromSerial<0.1 ? 0.1:floatFromSerial;
  }
  // "f" commands a frequency setpoint. Once set, if the motor is enabled, it will accelerate until it reaches this value in open loop control.
  else if (strcmp("f", commandFromSerial) == 0){
      targetFreq = (floatFromSerial > 300) ? 300:floatFromSerial<0.1 ? 0.1:floatFromSerial;
  }
  // "c" commands a carrier frequency (switching frequency) in Hz betwen 123 and 15000 hz
  else if (strcmp("c", commandFromSerial) == 0){
    if(integerFromSerial >= 123 && integerFromSerial <= 15000){
      setCarrierFreq(integerFromSerial);
    }
  }
  // "a" commands a change in the acceleration constant so acceleration speed can be changed. This will eventually become a setting for hz/s
  else if (strcmp("a", commandFromSerial) == 0){
    kAccel = (floatFromSerial > 0.5) ? 0.5:floatFromSerial<0.001 ? 0.001:floatFromSerial;
  }
  // "ph" commands a switch in single or triple phase operation. Will only update when the motor is stopped
  else if (strcmp("ph", commandFromSerial) == 0){
    if(integerFromSerial == 1 && motorRunning == false){
      onePhase = true;
    }
    else if(integerFromSerial == 3 && motorRunning == false){
      onePhase = false;
    }
  }

  // Update carrier frequency depending on current motor drive frequency (This code is specific to the R160 emulation)
  if(driveFreq <= 5){
    carrierFreqState = 0;
  }
  else if(driveFreq <= 7 && driveFreq > 5){
    carrierFreqState = 1;
  }
  else if(driveFreq <= 15 && driveFreq > 7){
    carrierFreqState = 2;
  }
  else if(driveFreq <= 25 && driveFreq > 15){
    carrierFreqState = 3;
  }
  else if(driveFreq <= 40 && driveFreq > 25){
    carrierFreqState = 4;
  }
  else if(driveFreq <= 60 && driveFreq > 40){
    carrierFreqState = 3;
  }
  else if(driveFreq > 60){
    carrierFreqState = 4;
  }
  if(prevCarrierFreqState != carrierFreqState){
    setCarrierFreq(r160Sound[carrierFreqState]);
  }


  // Check enable switch and run correct function if state is set properly
  uint8_t pwmState = PINB & (1<<PB7);

  if(pwmState != 0 && prevpwmState != pwmState){
    startPWM();
  }
  else if (prevpwmState == pwmState){
    stopPWM();
  }

  prevpwmState = pwmState;

  prevCarrierFreqState = carrierFreqState;

}

// Timer 3 overflow ISR, responsible for generating SPWM signals
ISR(TIMER3_OVF_vect){

  if(driveFreq > 0.1){
    motorRunning = true;
    // Enable all timer outputs with compare mode output settings:
    TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|(1<<COM3C1)|(0<<COM3C0);
    TCCR4A = (1<<COM4A1)|(1<<COM4A0)|(1<<COM4B1)|(1<<COM4B0)|(1<<COM4C1)|(1<<COM4C0);

    // Define constant value to scale sine table by based on DC and carrier frequency
    float constant = scaleFactor*(dutyCycle/100.0);

    // Deterimine U phase sin LUT index
    uPhaseTableIndex = ((float)sineIndex*sineStep);
    // Look up primary sine function value for u phase's duty cycle
    uint16_t uPhaseSin = ((float)sinTable[uPhaseTableIndex]*constant);

    if(onePhase == false){

      // V phase is offset by 120 degrees from U Phase, wrap around to zero if too large
      uint16_t vPhaseTableIndex = (uPhaseTableIndex < 240) ? (uPhaseTableIndex + 120):(uPhaseTableIndex - 240);
      // W phase is offset by 120 degrees from V Phase, wrap around if too large
      uint16_t wPhaseTableIndex = (vPhaseTableIndex < 240) ? (vPhaseTableIndex + 120):(vPhaseTableIndex - 240);

      // Look up remaining sine function values for V and W phases' duty cycle
      uint16_t vPhaseSin = ((float)sinTable[vPhaseTableIndex]*constant);
      uint16_t wPhaseSin = ((float)sinTable[wPhaseTableIndex]*constant);

      // Timer 3 compare registers set high side duty cycle for each phase
      OCR3A = (uPhaseSin < deadTimeCycles) ? 0:uPhaseSin;
      // Serial.println(OCR3A);
      
      // Flip U and V phases to reverse motor in 3-phase mode, low side timers will correctly update without flipping
      if (!dir3Ph){
        OCR3B = (vPhaseSin < deadTimeCycles) ? 0:vPhaseSin;
        OCR3C = (wPhaseSin < deadTimeCycles) ? 0:wPhaseSin;
      }
      else{
        OCR3C = (vPhaseSin < deadTimeCycles) ? 0:vPhaseSin;
        OCR3B = (wPhaseSin < deadTimeCycles) ? 0:wPhaseSin;
      }

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
  }
  else{
    motorRunning = false;
    TCCR3A = 0b00000000;
    TCCR4A = 0b00000000;

    PORTH |= (1<<PH3)|(1<<PH4)|(1<<PH5);
  }

  float deltaF = targetFreq - driveFreq;

  if (accelIndex >= 10){
  accelIndex = 0;
  if(abs(deltaF) <= 0.1){
    driveFreq = targetFreq;
  }
  else if (targetFreq > driveFreq){
    driveFreq += kAccel;
  }
  else if (targetFreq < driveFreq){
    driveFreq -= kAccel;
  }
  }
  else{
    accelIndex ++;
  }


  if (driveFreq != prevDriveFreq){
    //Serial.println(driveFreq);
    // Calculate number of required ISR cycles to produce 1 period of wave
    nISR = (CPUCLOCK/(2*(long)ICR3))/driveFreq;
    // Calculate sine table step size
    sineStep = ((float)nSine/(float)nISR);
    // Calculate nearest ISR sine index
    sineIndex = ((float)uPhaseTableIndex/sineStep)+1;
    // Calculate duty cycle based on V/F ratio
    float dutyCycleCalc = (vfRatio*driveFreq/vRMS_DCBUS)*100;
    dutyCycle = (dutyCycleCalc < 10) ? 50:(dutyCycleCalc > 100 ? 100:dutyCycleCalc);
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

// Only needs to be called once at boot, sets up all timer and external IO pins
void setupPins(){
  // Ext sync pin for oscilloscope
  pinMode(12, OUTPUT);

  // Ext enable pin
  pinMode(13, INPUT_PULLUP);

  // Gate driver enable pin
  pinMode(GATEENPIN, OUTPUT);

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

// Only needs to be called once at boot, sets up ISR operation and PWM generation
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

  // Reset output compare values for each timer (OCRnA/B/C)
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
  // Disable interrupts to prevent glitches
  cli();

  // Calculate TOP value to set for given carrier frequency (defines wave period)
  ICR3 = CPUCLOCK/(2*fCarrier); // Equation from section 17.9.5 of atmega2560 datasheet

  // Set all TOP values for each timer (overflow value, defines frequency, ICRn defines this in Mode 8)
  ICR4 = ICR3;

  // Sine table max value does not match the max value of TOP - scaleFactor defines this value 
  scaleFactor = (float)ICR3/(float)sinTableMax; // Calculate scaling factor for sine table

  // Calculate number of required ISR cycles to produce 1 period of wave
  nISR = (CPUCLOCK/(2*(long)ICR3))/driveFreq;

  // Calculate sine table step size
  sineStep = ((float)nSine/(float)nISR);

  // Calculate nearest ISR sine index
  sineIndex = ((float)uPhaseTableIndex/sineStep)+1;

  // // Calculate ISR period (time constant):
  // deltaT = 1/fCarrier;

  // Enable interrupts to resume operation
  sei();
}

void startPWM(){
  Serial.println("PWM started");
  // Turn on gates
  digitalWrite(GATEENPIN,HIGH);

  // Small delay to let gates turn on
  delayMicroseconds(65535);

  GTCCR = 0b10000011; // Halt and Sync All Timers

  // Reset all relevant timer count values
  TCNT3 = 0;
  TCNT4 = 0;

  GTCCR = 0b00000011; // Free synced timers (Prescaler syncs will automatically clear)

  // Enable all timer outputs with compare mode output settings:
  TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3B1)|(0<<COM3B0)|(1<<COM3C1)|(0<<COM3C0);
  TCCR4A = (1<<COM4A1)|(1<<COM4A0)|(1<<COM4B1)|(1<<COM4B0)|(1<<COM4C1)|(1<<COM4C0);

  motorRunning = true;
}

void stopPWM(){
  cli();

  Serial.println("PWM stopped");

  digitalWrite(GATEENPIN,LOW);

  // Reset drive frequency to default

  driveFreq = 0.5;

  // Disable all timer outputs with compare mode output settings:
  TCCR3A = 0b00000000;
  TCCR4A = 0b00000000;

  motorRunning = false;

  GTCCR = 0b10000011; // Halt and Sync All Timers

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

  sei();
}