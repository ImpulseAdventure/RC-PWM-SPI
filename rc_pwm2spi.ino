// RC Receiver PWM to SPI slave
// ============================
//   by Calvin Hass
//   http://www.impulseadventure.com/elec/
//
// This code implements a simple SPI slave receiver interface
// combined with multi-channel pulse-width modulation (PWM)
// measurement. Each channel's pulse width is measured in
// microseconds and returned in a channelized register interface.
// This code can be useful for using a remote-control transmitter
// to control an Arduino / ATtiny microcontroller.
//
// - A watchdog timeout is used to detect the loss of the transmitter.
//
// - Optimized IO commands are used in the ISRs to keep the
//   critical sections as fast as possible.
// 
// - This example demonstrates 6 channel monitoring, but this can
//   be increased/decreased if needed.
//

// Includes for ISR and IO operations
#include <avr/interrupt.h>
#include <avr/io.h>

// -----------------------
// General configuration
// -----------------------

// - Define number of channels to support.
//   This is used when creating the array of SPI registers
#define NUM_CHAN          6

// - Enable periodic LED flashing of status
//    1 flash    = No transmitter detected
//    3 flashes  = Transmitter detected
// - Also emits 4 flashes at startup to indicate program running
// - Comment out the following line to disable
#define LED_FLASH

// TIMER INTERVALS:

// - PULSE_TIMEOUT defines the maximum delay between PWM
//   pulses on a channel before it is assumed to be inactive.
//   This is used for the watchdog timeout. Generally, RC
//   receivers should deliver pulses approximately every
//   20ms, but one should provide ample margin in case a
//   pulse gets missed.
#define PULSE_TIMEOUT   250

// - STATUS_TIMEOUT defines the delay between flashes of
//   the LED for indicating status.
#define STATUS_TIMEOUT  3000

// -----------------------------------------------------------------------
// MICROCONTROLLER-SPECIFIC CONFIGURATION:

// PIN DEFINITIONS
// The pin definitions and mapping to port & pin change interrupts
// are very specific to the microcontroller model in use. The following
// values were tested to work with an ATtiny167 (Digispark Pro).
// Modifications can be done to make this work for other Arduino / AVR
// microcontrollers (such as ATmega328).

// - Pin definitions for LED & SPI
#define PIN_LED 1
#define PIN_MOSI 10     // MOSI / PA4
#define PIN_MISO 8      // MISO / PA2
#define PIN_SCK  11     // SCK  / PA5
#define PIN_SS   12     // SSb  / PA6 / PCINT6 (ISR PCINT0)

// - Define RC receiver pin connections to ATtiny
#define PORT_A_SS   PORTA6  // PA6 / PCINT6 (ISR PCINT0)
#define PORT_A_CH5  PORTA7  // PA7 / PCINT7 (ISR PCINT0)
#define PORT_A_CH6  PORTA3  // PA3 / PCINT3 (ISR PCINT0)
#define PORT_B_CH1  PORTB0  // PB0 / PCINT8 (ISR PCINT1)
#define PORT_B_CH2  PORTB2  // PB2 / PCINT10 (ISR PCINT1)
#define PORT_B_CH3  PORTB6  // PB6 / PCINT14 (ISR PCINT1)
#define PORT_B_CH4  PORTB3  // PB3 / PCINT11 (ISR PCINT1)


// Define mapping between physical pins and the pin change interrupts


// - Pin Change interrupts in ISR PCINT0_vect / PCMSK0
#define PCINT_0_CH5 PCINT7  //PCMSK0
#define PCINT_0_CH6 PCINT3  //PCMSK0
#define PCINT_0_SS  PCINT6  //PCMSK0
// - Pin Change interrupts in ISR PCINT1_vect / PCMSK1
#define PCINT_1_CH1 PCINT8  //PCMSK1
#define PCINT_1_CH2 PCINT10 //PCMSK1
#define PCINT_1_CH3 PCINT14 //PCMSK1
#define PCINT_1_CH4 PCINT11 //PCMSK1

// Define data direction register for PIN_MISO
#define DDR_MISO_PORT   DDRA
#define DDR_MISO_FIELD  DDA2

// -----------------------------------------------------------------------


// General channel indices
// - These are used primarily for register lookup,
//   pulse counters and defining which channels are
//   included in the watchdog timeout monitoring.
#define IND_CH1   0
#define IND_CH2   1
#define IND_CH3   2
#define IND_CH4   3
#define IND_CH5   4
#define IND_CH6   5

// Pulse values
// - Default pulse width used before we receive any
//   from the receiver. Default is mid-point of RC
//   PWM range (ie. 1500us)
#define PWM_MID         1500


// -----------------------------------------------------------------------
// GENERAL PROGRAM VARIABLES

// Pulse measurement
volatile uint16_t       m_anPulseUs[NUM_CHAN];      // Pulse width measured (latest)
volatile uint8_t        m_nPulsesNew = 0x00;
volatile uint8_t        m_nPulsesTimeout = 0x00;


// Global status
uint8_t                 m_bTransmitterFail = 0;
volatile uint8_t        m_bSpiXferOvf = 0;


// Pin change events
volatile uint16_t       m_nPinRiseTimeUs[NUM_CHAN]; // Rising edge tstamp (from ISR)
uint16_t                m_nPinKickTimeMs[NUM_CHAN]; // Last watchdog kick (from main loop)

volatile uint8_t        m_nPinALast = 0x00;  // Last PINA state (from ISR)
volatile uint8_t        m_nPinBLast = 0x00;  // Last PINB state (from ISR)

// SPI slave
volatile boolean        m_bSlaveSelected = false;
volatile uint16_t       m_nXferCycle = 0;

// Latched command type and reg address from COMMAND cycle
volatile uint8_t    m_nRegAddr = 0;
volatile uint8_t    m_bRegRWb = 0;



// Register interface

#define         REGS_PER_CHAN 2
#define         REG_PART_H    0
#define         REG_PART_L    1

#define         REG_STATUS        0
#define         REG_RSVD1         1
#define         REG_FIXED1        2
#define         REG_FIXED2        3
#define         REGBASE_CHAN      4
#define         REG_MAX           REGBASE_CHAN + (NUM_CHAN*REGS_PER_CHAN)


// Declare the primary register array
volatile uint8_t m_anRegArray[REG_MAX];


// -----------------------------------------------------------------------
// START OF PROGRAM CODE
// -----------------------------------------------------------------------

// Configure the interfaces, the interrupts and assign
// the default register values
void setup() {

  // Initialize register array
  for (int nRegInd=0;nRegInd<REG_MAX;nRegInd++) {
    m_anRegArray[nRegInd] = 0x00;
  }
  // Special overrides
  m_anRegArray[REG_FIXED1] = 0x12;
  m_anRegArray[REG_FIXED2] = 0x34;

  // Reset the pulse width time on all channels
  // and the last watchdog kick timestamp
  for (int nChan=0;nChan<NUM_CHAN;nChan++) {
    // Reset pulse value
    m_anPulseUs[nChan] = PWM_MID;

    // Reset pulse timestamps
    m_nPinRiseTimeUs[nChan] = 0;
    m_nPinKickTimeMs[nChan] = 0;
  
  }
  // Default to timeout state on all channels
  m_nPulsesNew = 0x00;
  m_nPulsesTimeout = 0xFF;

  // -------------------------
  // Setup IOs
  setupConfig();

  // Indicate that bootloader is complete and we
  // are ready to accept SPI transactions
#ifdef LED_FLASH  
  doBlinkFlash(4);
#endif

  // Enable interrupts
  sei();  
}


// Initialize the IO interfaces
// - SPI interface
// - Pin change interrupts for pulse monitoring
void setupConfig() {
  
  // Define pins for SPI
  // - Start with the slave unselected
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_SCK,  INPUT);
  pinMode(PIN_SS,   INPUT);

#ifdef LED_FLASH  
  // Initialize the LED
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,LOW);
#endif  

  // Reset pin change states
  for (int nNumChan=0;nNumChan<NUM_CHAN;nNumChan++) {
    m_nPinRiseTimeUs[nNumChan] = 0;
  }  

  // Enable SPI and the SPI Transaction Complete ISR
  SPCR |= (1 << SPE) | (1 << SPIE);

  // ATtiny167:
  // - Configure the pins that will trigger Pin Change event interrupts
  PCMSK0 |= (1 << PCINT_0_CH5) | (1 << PCINT_0_CH6) | (1 << PCINT_0_SS);
  PCMSK1 |= (1 << PCINT_1_CH1) | (1 << PCINT_1_CH2) | (1 << PCINT_1_CH3) | (1 << PCINT_1_CH4);
  // - Enable the Pin Change interrupts
  PCICR  |= (1<<PCIE0) | (1<<PCIE1);

  // ATmega328
  // - PCIE0,  PCIE1,  PCIE2
  // - PCMSK0, PCMSK1, PCMSK2

}


// SPI Transaction complete ISR
//
// This ISR is responsible for handling the read and write
// transfer cycles during a SPI transaction.
ISR(SPI_STC_vect)
{

  // Local variables
  uint8_t  nBufDat = 0;      // Storage for incoming SPDR
  uint8_t  nRegCmd = 0;

  // ----------------------------------
  // Capture inputs for previous cycle
  // ----------------------------------

  // Fetch the incoming data byte from the SPI Data Register
  nBufDat = SPDR;
    
  if (m_nXferCycle == 0) {
    // COMMAND cycle

    // Latch command
    nRegCmd = nBufDat;
    
    // Decode command
    m_nRegAddr = (nRegCmd & 0x0F);  // For now, only provide 16 regs
    m_bRegRWb = (nRegCmd & 0xC0) >> 6;

    // Perform range-check
    // TODO: Handle the 16-bit read case
    if (m_nRegAddr >= REG_MAX) {
      // Disable write
      m_nRegAddr = 0;
      m_bRegRWb = 1;
    }
    
  } else if (m_nXferCycle == 1) {
    // DATA cycle

    // Latch data   
    if (m_bRegRWb == 0) {
      // Write command: Latch write data
      m_anRegArray[m_nRegAddr] = nBufDat;
    } else {
      // Read command: nothing to latch
    }
  } else if (m_nXferCycle == 2) {
    // DATA cycle #2
    if (m_bRegRWb == 0) {
      // Write command: Latch write data
      m_anRegArray[m_nRegAddr+1] = nBufDat;
    }
  }

  
  // ----------------------------------
  // Setup output for next cycle
  // ----------------------------------
  m_nXferCycle++;

  if (m_nXferCycle > 3) {
    m_bSpiXferOvf = 1;
  }

  // If we are still the selected slave, then proceed to
  // define our outputs. Otherwise, we can ignore this
  // request.
  if (m_bSlaveSelected) {
    if (m_nXferCycle == 1) {
      // DATA cycle #1
      // We only need to drive SPDR with valid data on a read
      // but for efficiency, always return the current register value
      SPDR = m_anRegArray[m_nRegAddr];
    } else if (m_nXferCycle == 2) {
      // DATA cycle #2
      // For a 16-bit read, we advance the register address
      // TODO: Consider adding array bounds checking here
      SPDR = m_anRegArray[m_nRegAddr+1];
    }
    
  } else {
    // We are not selected for next cycle
    // The pin change int on SSb will be responsible
    // for tristating the MISO pin so that other slaves
    // can respond if they are selected.
  } 

}


// =================================


// Pin Change interrupt #0 ISR
//
// This ISR is responsible for monitoring the pins
// associated with I/O bank 0.
ISR(PCINT0_vect)
{
  // ------------------------------------
  // Start

  // Latch the current pin values and deltas
  uint8_t nPinValCur = PINA;
  uint8_t nPinValChg = nPinValCur ^ m_nPinALast;

  // Latch the current timestamp
  //
  // NOTE 1: By latching the timestamp once for all channels
  // in the ISR, we are introducing timestamp error into the later
  // channels. However, the tradeoff is against calling micros()
  // multiple times which extends the ISR duration. If the ISR
  // duration is too high, then there is a greater chance of SPI
  // corruption due to the next SCLK edge arriving before we have
  // completed this ISR and prepared for the next data in/out.
  //
  // NOTE 2: micros() is only accurate to approx 4us uncertainty.
  // Other alternate timer implementations can increase this accuracy,
  // but this code uses micros() to simplify the code.
  uint16_t nCurTimeUs = micros();

  // ------------------------------------
  // Handler for SSb
  if (nPinValChg & (1 << PORT_A_SS)) {
    if (nPinValCur & (1 << PORT_A_SS)) { // rising edge
      // SSb rising edge = deassertion
      // - We are not currently selected
      // - Release the bus
      DDR_MISO_PORT &= ~(1 << DDR_MISO_FIELD);  // pinMode(PIN_MISO,INPUT)
      m_bSlaveSelected = false;    
    } else {
      // SSb falling edge = assertion
      // - We weren't selected before, but now are
      // - Take ownership over the bus
      DDR_MISO_PORT |= (1 << DDR_MISO_FIELD);  // pinMode(PIN_MISO,OUTPUT)
      // - Reset the transaction cycle
      m_nXferCycle = 0;
      m_bSlaveSelected = true;
    }
  }
  
  // ------------------------------------
  // Handler for CH5
  if (nPinValChg & (1 << PORT_A_CH5)) {
    if (nPinValCur & (1 << PORT_A_CH5)) { // rising edge
      m_nPinRiseTimeUs[IND_CH5] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH5] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH5];
      m_nPulsesNew |= (1<<IND_CH5); // Indicate a new pulse
    }
  }
  
  // ------------------------------------
  // Handler for CH6
  if (nPinValChg & (1 << PORT_A_CH6)) {
    if (nPinValCur & (1 << PORT_A_CH6)) { // rising edge
      m_nPinRiseTimeUs[IND_CH6] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH6] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH6];
      m_nPulsesNew |= (1<<IND_CH6); // Indicate a new pulse
    }
  }

  // ------------------------------------
  // Cleanup
  // - Save the latest pin state
  m_nPinALast = nPinValCur;
  
}


// Pin Change interrupt #1 ISR
//
// This ISR is responsible for monitoring the pins
// associated with I/O bank 1.
ISR(PCINT1_vect)
{
  // ------------------------------------
  // Start

  // Latch the current pin values and deltas
  uint8_t nPinValCur = PINB;
  uint8_t nPinValChg = nPinValCur ^ m_nPinBLast;
    
  // Latch the current timestamp
  uint16_t nCurTimeUs = micros(); 

  // ------------------------------------
  // Handler for CH1
  if (nPinValChg & (1 << PORT_B_CH1)) {
    if (nPinValCur & (1 << PORT_B_CH1)) { // rising edge
      m_nPinRiseTimeUs[IND_CH1] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH1] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH1];
      m_nPulsesNew |= (1<<IND_CH1); // Indicate a new pulse
    }
  }

  // ------------------------------------
  // Handler for CH2
  if (nPinValChg & (1 << PORT_B_CH2)) {
    if (nPinValCur & (1 << PORT_B_CH2)) { // rising edge
      m_nPinRiseTimeUs[IND_CH2] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH2] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH2];
      m_nPulsesNew |= (1<<IND_CH2); // Indicate a new pulse
    }
  }

  // ------------------------------------
  // Handler for CH3
  if (nPinValChg & (1 << PORT_B_CH3)) {
    if (nPinValCur & (1 << PORT_B_CH3)) { // rising edge
      m_nPinRiseTimeUs[IND_CH3] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH3] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH3];
      m_nPulsesNew |= (1<<IND_CH3); // Indicate a new pulse
    }
  }

  // ------------------------------------
  // Handler for CH4
  // NOTE: This doesn't get called if USB bootloader active
  if (nPinValChg & (1 << PORT_B_CH4)) {
    if (nPinValCur & (1 << PORT_B_CH4)) { // rising edge
      m_nPinRiseTimeUs[IND_CH4] = nCurTimeUs;
    } else {  // falling edge
      m_anPulseUs[IND_CH4] = nCurTimeUs-m_nPinRiseTimeUs[IND_CH4];
      m_nPulsesNew |= (1<<IND_CH4); // Indicate a new pulse
   }
  }

  // ------------------------------------
  // Cleanup
  // - Save the latest pin state
  m_nPinBLast = nPinValCur;
}



// =================================

// The main loop is responsible for the following operations:
// - Watchdog timeout: detects a lack of pulses on one or more
//   RC channels (which indicates that the RC transmitter has
//   been lost).
// - Updating RC pulse width registers which can be read via SPI.
// - Blinking a status LED to indicate if the transmitter link is
//   active.

void loop() {
  uint16_t   nPulseUsWidth;
  uint16_t   nTimeMsCur;
  uint16_t   nTimeMsElapsed;

  // Watchdog detector
  for (int nChan=0;nChan<NUM_CHAN;nChan++) {
    nTimeMsCur = millis();

    if (m_nPulsesNew & (1<<nChan)) {
      // New pulse detected in ISR
      // Kick the watchdog
      m_nPinKickTimeMs[nChan] = nTimeMsCur;
      // Clear the timeout flag (if any)
      m_nPulsesTimeout &= ~(1<<nChan);
      // Reset the pulse detect
      m_nPulsesNew &= ~(1<<nChan);
    } else {
      // No new pulse, so check watchdog limit
      nTimeMsElapsed = nTimeMsCur-m_nPinKickTimeMs[nChan];
      if (nTimeMsElapsed > PULSE_TIMEOUT) {
        // Set the timeout flag
        m_nPulsesTimeout |= (1<<nChan);
      }      
    }
  }

  // Watchdog timeout -> Transmitter loss:
  //
  // If the RC transmitter is turned off (but the RC receiver
  // is still powered up), then most channels will not contain
  // pulses. However, some receivers (such as the Turnigy 9XCH8v2)
  // will still output a stream of pulses on one or more channels.
  // Therefore, the watchdog must only monitor certain channels
  // for inactivity.
  // - Only look at pulse train from CH1 & CH2
  // - Note that Turnigy 9X8Cv2 seems to output very slow pulses
  //   on CH4 & CH5 during transmitter loss.
  m_bTransmitterFail = 0;
  if (m_nPulsesTimeout & ( (1<<IND_CH1) | (1<<IND_CH2) ) ) {
    m_bTransmitterFail = 1;
  }

  // Store the pulses into the registers
  for (int nChan=0;nChan<NUM_CHAN;nChan++) {

    // Disable ints for 16b coherency
    cli();
    nPulseUsWidth = m_anPulseUs[nChan];
    sei();
    
    // Just report last saved pulse width. If there is a transmitter
    // fail, then the status flag can be used as a failsafe.
    m_anRegArray[REGBASE_CHAN+(nChan*REGS_PER_CHAN)+REG_PART_H] = nPulseUsWidth/256;
    m_anRegArray[REGBASE_CHAN+(nChan*REGS_PER_CHAN)+REG_PART_L] = nPulseUsWidth%256;
  } // nChan

  // Now set overall status register
  uint8_t nNewStatus;
  nNewStatus = 0x00;
  nNewStatus |= (m_bTransmitterFail)?(0x00):(0x80);
  nNewStatus |= (m_bSpiXferOvf)?(0x00):(0x40);
  cli();
  m_anRegArray[REG_STATUS] = nNewStatus;
  sei();

  m_anRegArray[REG_RSVD1] = 0x00;

  // Periodically flash the link status
#ifdef LED_FLASH  
  if (m_bTransmitterFail) {
    setBlinkState(3);
  } else {
    setBlinkState(1);
  }
  doBlinkFsm();
#endif

  
}

// =================================

#define BLINK_FLASH_ON  20
#define BLINK_FLASH_OFF 150

enum teBlinkFsm {E_BLFSM_IDLE,E_BLFSM_ON,E_BLFSM_OFF};

volatile uint8_t  nBlinkStatus = 0;
volatile uint8_t  nBlinkRemain = 0;
volatile uint8_t  eBlinkFsmState = E_BLFSM_IDLE;
volatile uint16_t nBlinkFsmTimeMsNxt = 0;

// Define the number of blinks to periodically flash
void setBlinkState(int nNum) {
  nBlinkStatus = nNum;
}

// Update the LED status based on the blink FSM
// - The purpose of this blink routine is to be
//   basically non-blocking. It relies upon defining
//   timer thresholds for each stage of the blink
//   request.
void doBlinkFsm() {
  uint16_t nTimeMsCur = millis();
  // Calculate next state
  switch (eBlinkFsmState) {
    case E_BLFSM_IDLE:
      digitalWrite(PIN_LED,LOW);
      if (nTimeMsCur > nBlinkFsmTimeMsNxt) {
        if (nBlinkStatus > 0) {
          eBlinkFsmState = E_BLFSM_ON;
          nBlinkFsmTimeMsNxt = nTimeMsCur + BLINK_FLASH_ON;
          nBlinkRemain = nBlinkStatus;
        }
      }
      break;
    case E_BLFSM_ON:
      digitalWrite(PIN_LED,HIGH);
      if (nTimeMsCur > nBlinkFsmTimeMsNxt) {
        if (nBlinkRemain==0) {
          // Should never get here
          eBlinkFsmState = E_BLFSM_IDLE;
          nBlinkFsmTimeMsNxt = nTimeMsCur + STATUS_TIMEOUT;
        } else {
          nBlinkRemain--;
          eBlinkFsmState = E_BLFSM_OFF;
          nBlinkFsmTimeMsNxt = nTimeMsCur + BLINK_FLASH_OFF;          
        }
      }
      break;
    case E_BLFSM_OFF:
      digitalWrite(PIN_LED,LOW);
      if (nTimeMsCur > nBlinkFsmTimeMsNxt) {
        if (nBlinkRemain==0) {
          eBlinkFsmState = E_BLFSM_IDLE;
          nBlinkFsmTimeMsNxt = nTimeMsCur + STATUS_TIMEOUT;
        } else {
          eBlinkFsmState = E_BLFSM_ON;
          nBlinkFsmTimeMsNxt = nTimeMsCur + BLINK_FLASH_ON;
        }
      }
      break;
  }
}

// Flash the LED quickly [nNum] times
// - Blocking waits
void doBlinkFlash(int nNum) {
  pinMode(PIN_LED, OUTPUT);
  for (int i=0;i<nNum;i++) {
    digitalWrite(PIN_LED,HIGH);
    delay(20);
    digitalWrite(PIN_LED,LOW);
    delay(150);
  }
}


// =================================
