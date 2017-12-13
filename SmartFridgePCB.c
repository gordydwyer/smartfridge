//******************************************************************************
//!
//! *** Function 1 - Sample A0-3 Input (force sensors analog signals)
//!                  using Repeated Sequence of Conversions ***
//!
//!  - perform a repeated sequence of conversions
//!    using "repeat sequence-of-channels" mode on A0 - A3.
//!  - AVcc (3.3V) = reference.
//!  - each conversion result is stored in ADC12MEM0, ADC12MEM1, ADC12MEM2,
//!    and ADC12MEM3 respectively.
//!  - after each sequence, the 4 conversion results are added to A0sum, A1sum,
//!    A2sum, and A3sum respectively.
//!  - after sampling Num_of_Results times, average the four sums
//!    and store in A0update - A3update respectively.
//!  - at last, copy the updated values to A0prev - A3prev respectively.
//!
//!                MSP430F552x
//!             -----------------
//!         /|\|                 |
//!          | |          P6.0/A0|<- Vin1 ---- send UART message
//!          --|RST       P6.1/A1|<- Vin2 ---- send UART message
//!            |          P6.2/A2|<- Vin3 ---- send UART message
//!            |          P6.3/A3|<- Vin4 ---- send UART message
//!            |                 |Vref is Vcc in all cases
//!
//!  - peripherals and I/O signals: ADC12_A peripheral,
//!    Analog input A0 - A3, i.e. P6.0, P6.1, P6.2, P6.3
//!  - interrupt handlers used: ADC12_A_VECTOR
//!
//! *** Function 2 - Send messages to computer using UART by USCI_A1 module ***
//!
//!  - use UART to transmit specific messages to COMx on computer
//!  - COMx: check device management on computer to get the COM number
//!  - use Putty to see the serial outputs on COMx
//£¡
//£¡ *** Function 3 - Perform serial read by clocking out each bit of data from load amplifier ***
//!  - P2.4 = DOUT from load amplifier, GPIO input
//!  - P2.5 = SCK to load amplifier, GPIO output
//!  - as soon as DOUT is low, meaning data is ready, SCK starts to generate pulses
//!  - each pulse clocks out one bit of data on DOUT, MSB first
//!  - the array "data" is used to store the raw data, data[2] filled first, data[0] filled last
//!  - weight = (average reading - ZERO FACTOR) / CALIBRATION FACTOR,
//!    where ZERO FACTOR and CALIBRATION FACTOR are recorded by running Arduino calibration sketch first
//!
//******************************************************************************
#include "driverlib.h"
#include "delay.h"

//******************************************************************************
//             consts and variables for reading 4 force sensors
//******************************************************************************
//#define   Num_of_Results  100         // for averaging
//
//// DOOR_STATE = 0: Door is closed
//// DOOR_STATE = 1: Door is open
//volatile bool DOOR_STATE = 0;
//
//volatile uint32_t A4sum = 0;
//volatile uint32_t A5sum = 0;
//volatile uint32_t A6sum = 0;
//volatile uint32_t A7sum = 0;
//volatile uint32_t A12sum = 0;
//
//volatile uint16_t A4prev = 0;
//volatile uint16_t A5prev = 0;
//volatile uint16_t A6prev = 0;
//volatile uint16_t A7prev = 0;
//volatile uint16_t A12prev = 0;
//
//volatile uint16_t A4update;
//volatile uint16_t A5update;
//volatile uint16_t A6update;
//volatile uint16_t A7update;
//volatile uint16_t A12update;
//
//const uint16_t THRESHOLD = 0xBFF;    // threshold is set to ~50% of 3.3V
#define   Num_of_Results  1         // for averaging

// DOOR_STATE = 0: Door is closed
// DOOR_STATE = 1: Door is open
volatile bool DOOR_STATE = 1;

//CLOSED_FLAG = 1: first time
//CLOSED_FLAG = 0: after first time
volatile bool CLOSED_FLAG = 1;

uint8_t streakNum = 250;

uint16_t streak6A = 0;
uint16_t streak6R = 0;
uint16_t streak5A = 0;
uint16_t streak5R = 0;
uint16_t streak7A = 0;
uint16_t streak7R = 0;
uint16_t streak4A = 0;
uint16_t streak4R = 0;
uint16_t streak12O = 0;
uint16_t streak12C = 0;

volatile uint32_t A6current = 0;
volatile uint32_t A5current = 0;
volatile uint32_t A7current = 0;
volatile uint32_t A4current = 0;
volatile uint32_t A12current = 0;
volatile uint32_t A14current = 0;

volatile uint32_t A4sum = 0;
volatile uint32_t A5sum = 0;
volatile uint32_t A6sum = 0;
volatile uint32_t A7sum = 0;
volatile uint32_t A12sum = 0;

volatile uint16_t A4prev = 0;
volatile uint16_t A5prev = 0;
volatile uint16_t A6prev = 0;
volatile uint16_t A7prev = 0;
volatile uint16_t A12prev = 0;

volatile uint16_t A4update;
volatile uint16_t A5update;
volatile uint16_t A6update;
volatile uint16_t A7update;
volatile uint16_t A12update;

volatile uint8_t A4on = 0;
volatile uint8_t A5on = 0;
volatile uint8_t A6on = 0;
volatile uint8_t A7on = 0;

const uint16_t THRESHOLD = 1500;    // threshold is set to ~50% of 3.3V - was 0xBFF

//******************************************************************************
//                     variables and messages for UART
//******************************************************************************
char pos0 = '0';                     // position 0 change detected
char pos1 = '1';                     // position 1 change detected
char pos2 = '2';                     // position 2 change detected
char pos3 = '3';                     // position 3 change detected

char add = '1';                      // item added
char rem = '0';                      // item removed

volatile char c[5] = {0};            // weight in 4 separate bytes + "\0"

volatile uint8_t received = 0;

#define F2DW(floatNum)  *((unsigned long *) & floatNum )  // float to double word conversion
                                                          // for sending weight in bytes
// Function prototypes
//void sendWeight(float input);        // send weight data using UART
//void rotate90();
void sendWeight(float input);        // send weight data using UART
void rotate90();
void rotate1();
uint16_t Fletcher16( uint8_t *data, int count );

//******************************************************************************
//          consts and variables for serial read from load amplifier
//******************************************************************************
const uint8_t TIMES = 5;            // read the raw data 10 times and calculate average
volatile uint8_t fill = 0;           // padding unsigned value to signed value

// Structure to hold its OFFSET and SCALE value
struct HX711
{
    uint8_t gain;                    // generate 1 extra pulse to mark as end
    long OFFSET;                     // minus offset to get 0 value
    float SCALE;                     // scale to expected unit: gram
} amp;

volatile uint8_t data[3] = {0};      // store raw 24-bit data that are read
volatile unsigned long value = 0;    // store raw 32-bit data in readRawData()
volatile long averageValue = 0;      // store average value raw 32-bit data in calAverage()
//volatile float newWeight = 0;        // new total weight obtained by calling getFinalWeight()
//volatile float preWeight = 0;        // previous total weight assigned by the last newWeight

volatile float preWeight = 0;
volatile int cycleNum = 0;

volatile float newWeight = 0;

// Function prototypes
void begin();                        // configure GPIOs, set gain, get ready
long readRawData();                  // read and return raw 32-bit data stored in "value" once
long calAverage(uint8_t times);      // call readRawData() "times" times, calculate and return average
float getFinalWeight(uint8_t times); // call calAverage(), return actual total weight in correct unit
void powerOn();                      // turn on load amplifier reading
void powerDown();                    // turn off load amplifier reading for reset

void main (void)
{
    // Stop Watchdog Timer
    WDT_A_hold(WDT_A_BASE);

    // Set up UART module: USCI_A0 setup, to computer
    P3SEL = BIT3+BIT4;                        // P3.4,5 = USCI_A0 TXD/RXD
        UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
        UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
        UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
        UCA0BR1 = 0x00;                           //
        UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
        UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
        UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

        delayMicroSecond(10000000);

    P4SEL |= 0x30;          // P4.4 & P4.5: USCI_A1 TXD/RXD
    UCA1CTL1 |= UCSWRST;    // in reset mode
    UCA1CTL1 |= UCSSEL_2;   // choose 32768Hz   UCSSEL_2
    UCA1BR0 = 0x6D;            // baud rate: 9600  0x6D
    UCA1BR1 = 0;         // choose 32768Hz   0
    UCA1MCTL = 0x04;        // modulation UCBRSx = 3, UCBRFx = 0   0x04
    UCA1CTL0 = 0x00;        // not used   0x00
    UCA1CTL1 &= ~UCSWRST;   // in operation mode
    // UCA1IE |= UCRXIE;

    delayMicroSecond(10000000);

    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN6);    //ready to send uart packet

    GPIO_setAsInputPin(GPIO_PORT_P1,
                GPIO_PIN5); //RESET

    // Enable GPIO debug: P2.1 - P2.5, p7.4 is step, p5.7 is dir
    GPIO_setAsOutputPin(GPIO_PORT_P2,
            GPIO_PIN1 + //ENABLE
            GPIO_PIN2 + //SLEEP
            GPIO_PIN3 + //MS2
            GPIO_PIN4 + //MS1
            GPIO_PIN5); //RESET
    GPIO_setAsOutputPin(GPIO_PORT_P5,
                GPIO_PIN7); //DIR
    GPIO_setAsOutputPin(GPIO_PORT_P7,
                    GPIO_PIN4); //STEP

    GPIO_setOutputLowOnPin(GPIO_PORT_P2,
            GPIO_PIN3 +
            GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7,
            GPIO_PIN4);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2,
            GPIO_PIN1 +
            GPIO_PIN2 +
            GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5,
            GPIO_PIN7);

    // Enable A/D channel inputs for 4 force sensors
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                               GPIO_PIN4 +
                                               GPIO_PIN5 +
                                               GPIO_PIN6 +
                                               GPIO_PIN7);
    // Enable A/D channel input for door sensor & home position sensor
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P7,
                                               GPIO_PIN0 +
                                               GPIO_PIN2);

    // Initialize the ADC12_A Module: for 4 force sensors
    /*
     * Base address of ADC12_A Module
     * Use internal ADC12_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC12_A_init(ADC12_A_BASE,
        ADC12_A_SAMPLEHOLDSOURCE_SC,
        ADC12_A_CLOCKSOURCE_ADC12OSC,
        ADC12_A_CLOCKDIVIDER_1
        );

    ADC12_A_enable(ADC12_A_BASE);

    /*
     * Base address of ADC12_A Module
     * For memory buffers 0-7 sample/hold for 256 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Enable Multiple Sampling
     */
    ADC12_A_setupSamplingTimer(ADC12_A_BASE,
        ADC12_A_CYCLEHOLD_256_CYCLES,
        ADC12_A_CYCLEHOLD_4_CYCLES,
        ADC12_A_MULTIPLESAMPLESENABLE);

    // Configure Memory Buffers for 4 force sensors
    /* Position # 0 of lazy susan
     * Base address of the ADC12_A Module
     * Configure memory buffer 0
     * Map input A0 to memory buffer 0
     * Vref+ = AVcc
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_A_configureMemoryParam param4 = {0};
    param4.memoryBufferControlIndex = ADC12_A_MEMORY_4;
    param4.inputSourceSelect = ADC12_A_INPUT_A4;
    param4.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param4.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param4.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param4);
    /* Position # 1 of lazy susan
     * Base address of the ADC12_A Module
     * Configure memory buffer 1
     * Map input A1 to memory buffer 1
     * Vref+ = AVcc
     * Vref- = AVss
     * Memory buffer 1 is not the end of a sequence
     */
    ADC12_A_configureMemoryParam param5 = {0};
    param5.memoryBufferControlIndex = ADC12_A_MEMORY_5;
    param5.inputSourceSelect = ADC12_A_INPUT_A5;
    param5.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param5.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param5.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param5);
    /* Position # 2 of lazy susan
     * Base address of the ADC12_A Module
     * Configure memory buffer 2
     * Map input A2 to memory buffer 2
     * Vr+ = AVcc
     * Vr- = AVss
     * Memory buffer 2 IS the end of a sequence
     */
    ADC12_A_configureMemoryParam param6 = {0};
    param6.memoryBufferControlIndex = ADC12_A_MEMORY_6;
    param6.inputSourceSelect = ADC12_A_INPUT_A6;
    param6.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param6.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param6.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param6);
    /* Position # 3 of lazy susan
     * Base address of the ADC12_A Module
     * Configure memory buffer 3
     * Map input A3 to memory buffer 3
     * Vr+ = AVcc
     * Vr- = AVss
     * Memory buffer 3 IS the end of a sequence
     */
    ADC12_A_configureMemoryParam param7 = {0};
    param7.memoryBufferControlIndex = ADC12_A_MEMORY_7;
    param7.inputSourceSelect = ADC12_A_INPUT_A7;
    param7.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param7.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param7.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param7);

    //Door sensor force sensor ADC configuration
    ADC12_A_configureMemoryParam param12 = {0};
    param12.memoryBufferControlIndex = ADC12_A_MEMORY_12;
    param12.inputSourceSelect = ADC12_A_INPUT_A12;
    param12.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param12.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param12.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param12);

    //Home sensor force sensor ADC configuration
    ADC12_A_configureMemoryParam param14 = {0};
    param14.memoryBufferControlIndex = ADC12_A_MEMORY_14;
    param14.inputSourceSelect = ADC12_A_INPUT_A14;
    param14.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param14.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param14.endOfSequence = ADC12_A_ENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE ,&param14);

    // As long as finishing reading door sensor, trigger interrupt
    // Enable memory buffer 4 interrupt
    ADC12_A_clearInterrupt(ADC12_A_BASE,
        ADC12IFG14);
    ADC12_A_enableInterrupt(ADC12_A_BASE,
        ADC12IE14);

    // Set OFFSET and SCALE values that are recorded during calibration
    amp.OFFSET = 255000;
    amp.SCALE = 50.0;
    begin();

    // Enable/Start first sampling and conversion cycle
    // Once start, ADC loops infinitely
    /*
     * Base address of ADC12_A Module
     * Start the conversion into memory buffer 0
     * Use the repeated sequence of channels
     */
    ADC12_A_startConversion(ADC12_A_BASE,
                            ADC12_A_MEMORY_4,
                            ADC12_A_REPEATED_SEQOFCHANNELS);

    __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled

    // Enter LPM0, Enable interrupts
    __bis_SR_register(LPM0_bits + GIE);
    // For debugger
    __no_operation();

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif

// ADC ISR
void ADC12ISR (void)
{
    //static uint16_t index = 0;
    switch (__even_in_range(ADC12IV,34)){
        case  0: break;   //Vector  0:  No interrupt
        case  2: break;   //Vector  2:  ADC overflow
        case  4: break;   //Vector  4:  ADC timing overflow
        case  6: break;   //Vector  6:  ADC12IFG0
        case  8: break;   //Vector  8:  ADC12IFG1
        case 10: break;   //Vector 10:  ADC12IFG2
        case 12: break;   //Vector 12:  ADC12IFG3
        case 14: break;   //Vector 14:  ADC12IFG4
        case 16: break;   //Vector 16:  ADC12IFG5
        case 18: break;   //Vector 18:  ADC12IFG6
        case 20: break;   //Vector 20:  ADC12IFG7
        case 22: break;   //Vector 22:  ADC12IFG8
        case 24: break;   //Vector 24:  ADC12IFG9
        case 26: break;   //Vector 26:  ADC12IFG10
        case 28: break;   //Vector 28:  ADC12IFG11
        case 30: break;   //Vector 30:  ADC12IFG12
        case 32: break;   //Vector 32:  ADC12IFG13
        case 34:
            // After finishing reading all 5 ADCs:
            ADC12_A_disableInterrupt (ADC12_A_BASE, ADC12IE14);

            // check whether door is open or closed
            // open if A4 is high, closed if low
            A12current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_12);
            if ((A12current < THRESHOLD) && (A12prev > THRESHOLD)) streak12C = 1;
            if ((streak12C > 0) && (A12current < THRESHOLD)) streak12C++;
            else streak12C = 0;
            if ((A12current > THRESHOLD) && (A12prev < THRESHOLD)) streak12O = 1;
            if ((streak12O > 0) && (A12current > THRESHOLD)) streak12O++;
            else streak12O = 0;

            A12prev = A12current;

            if (streak12O == streakNum)
            {
                DOOR_STATE = 1;
            }
            else if (streak12C == streakNum)
            {
                DOOR_STATE = 0;
            }

            if (DOOR_STATE)
            {
                CLOSED_FLAG = 1;

                if((cycleNum % 600 == 0) && ((streak4A < 1)||(streak4A > streakNum)) && ((streak5A < 1)||(streak5A > streakNum)) && ((streak6A < 1)||(streak6A > streakNum)) && ((streak7A < 1)||(streak7A > streakNum)))
                    preWeight = getFinalWeight(TIMES);
                cycleNum++;

                // *** A4 ***
                A4current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_4);
                if ((A4current < THRESHOLD) && (A4prev > THRESHOLD)) streak4A = 1;
                if ((streak4A > 0) && (A4current < THRESHOLD)) streak4A++;
                else streak4A = 0;
                if ((A4current > THRESHOLD) && (A4prev < THRESHOLD)) streak4R = 1;
                if ((streak4R > 0) && (A4current > THRESHOLD)) streak4R++;
                else streak4R = 0;

                // if adding item on position 2
                                //A6update < THRESHOLD && A6prev > THRESHOLD
                                if ((streak4A == streakNum) && (A4on == 0))
                                {
                                  A4on = 1;
                                  newWeight = getFinalWeight(TIMES);
                                  sendWeight(newWeight - preWeight);
                                  //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                  // send A6 info through UART
                                  // 1. position 2
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = pos0;
                                  // 2. add vs remove
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = add;
                                  // 3. weight
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = c[0];
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = c[1];
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = c[2];
                                  while(!(UCA1IFG&UCTXIFG));
                                  UCA1TXBUF = c[3];
                                  while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;
                                                                                }
                                                                                // if removing item on position 2
                                                                                //A6update > THRESHOLD && A6prev < THRESHOLD
                                                                                else if ((streak4R == streakNum) && (A4on == 1))
                                                                                {
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    A4on = 0;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    // send A2 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos0;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = rem;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;

                                                                                }
                                                                                A4prev = A4current;   // copy updated value to previous
                                                                                A4sum = 0;           // clear sum

                                                                                // Do the same for A5, A6, A7
                                                                                // *** A5 ***
                                                                                A5current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_5);
                                                                                if ((A5current < THRESHOLD) && (A5prev > THRESHOLD)) streak5A = 1;
                                                                                if ((streak5A > 0) && (A5current < THRESHOLD)) streak5A++;
                                                                                else streak5A = 0;
                                                                                if ((A5current > THRESHOLD) && (A5prev < THRESHOLD)) streak5R = 1;
                                                                                if ((streak5R > 0) && (A5current > THRESHOLD)) streak5R++;
                                                                                else streak5R = 0;

                                                                                // if adding item on position 2
                                                                                //A6update < THRESHOLD && A6prev > THRESHOLD
                                                                                if ((streak5A == streakNum) && (A5on == 0))
                                                                                {
                                                                                    A5on = 1;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    // send A6 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos1;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = add;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;

                                                                                }
                                                                                // if removing item on position 2
                                                                                //A6update > THRESHOLD && A6prev < THRESHOLD
                                                                                else if ((streak5R == streakNum) && (A5on == 1))
                                                                                {
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    A5on = 0;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    // send A2 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos1;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = rem;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;
                                                                                }
                                                                                A5prev = A5current;   // copy updated value to previous
                                                                                A5sum = 0;           // clear sum

                                                                                // *** A6 ***
                                                                                A6update = A6sum / Num_of_Results;        // filter by averaging

                                                                                A6current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_6);
                                                                                if ((A6current < THRESHOLD) && (A6prev > THRESHOLD)) streak6A = 1;
                                                                                if ((streak6A > 0) && (A6current < THRESHOLD)) streak6A++;
                                                                                else streak6A = 0;
                                                                                if ((A6current > THRESHOLD) && (A6prev < THRESHOLD)) streak6R = 1;
                                                                                if ((streak6R > 0) && (A6current > THRESHOLD)) streak6R++;
                                                                                else streak6R = 0;

                                                                                // if adding item on position 2
                                                                                //A6update < THRESHOLD && A6prev > THRESHOLD
                                                                                if ((streak6A == streakNum) && (A6on == 0))
                                                                                {
                                                                                    A6on = 1;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    // send A6 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos2;

                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = add;

                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;

                                                                                }
                                                                                // if removing item on position 2
                                                                                //A6update > THRESHOLD && A6prev < THRESHOLD
                                                                                else if ((streak6R == streakNum) && (A6on == 1))
                                                                                {
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    A6on = 0;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    // send A2 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos2;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = rem;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;
                                                                                }
                                                                                A6prev = A6current;   // copy updated value to previous
                                                                                A6sum = 0;           // clear sum

                                                                                // *** A7 ***
                                                                                A7current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_7);
                                                                                if ((A7current < THRESHOLD) && (A7prev > THRESHOLD)) streak7A = 1;
                                                                                if ((streak7A > 0) && (A7current < THRESHOLD)) streak7A++;
                                                                                else streak7A = 0;
                                                                                if ((A7current > THRESHOLD) && (A7prev < THRESHOLD)) streak7R = 1;
                                                                                if ((streak7R > 0) && (A7current > THRESHOLD)) streak7R++;
                                                                                else streak7R = 0;

                                                                                // if adding item on position 2
                                                                                //A6update < THRESHOLD && A6prev > THRESHOLD
                                                                                if ((streak7A == streakNum) && (A7on == 0))
                                                                                {
                                                                                    A7on = 1;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    // send A6 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos3;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = add;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;

                                                                                }
                                                                                // if removing item on position 2
                                                                                //A6update > THRESHOLD && A6prev < THRESHOLD
                                                                                else if ((streak7R == streakNum) && (A7on == 1))
                                                                                {
                                                                                    //while(!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN6));
                                                                                    A7on = 0;
                                                                                    newWeight = getFinalWeight(TIMES);
                                                                                    sendWeight(newWeight - preWeight);
                                                                                    // send A2 info through UART
                                                                                    // 1. position 2
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = pos3;
                                                                                    // 2. add vs remove
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = rem;
                                                                                    // 3. weight
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[0];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[1];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[2];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = c[3];
                                                                                    while(!(UCA1IFG&UCTXIFG));
                                                                                    UCA1TXBUF = 0x0A;    //New line
                                                                                    preWeight = newWeight;
                                                                                }
                                                                                A7prev = A7current;   // copy updated value to previous
                                                                                A7sum = 0;           // clear sum
                                                                            }
                                                                            if (!DOOR_STATE)
                                                                            {
                                                                                //Door is closed, rotate when command is received from Pi
                                                                                if (CLOSED_FLAG) {
                                                                                    A14current = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_14);
                                                                                    if (A14current < 2400)
                                                                                    {
                                                                                        rotate1();
                                                                                    }
                                                                                    else
                                                                                    {
                                                                                        int q;
                                                                                        for (q = 0; q < 6; q++) {
                                                                                            while(!(UCA1IFG&UCTXIFG));
                                                                                            UCA1TXBUF = 'X';
                                                                                        }
                                                                                        while(!(UCA1IFG&UCTXIFG));
                                                                                        UCA1TXBUF = '\n';

                                                                                        delayMicroSecond(100);
                                                                                        while (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));
                                                                                        rotate90();
                                                                                        while (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));
                                                                                        rotate90();
                                                                                        while (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));
                                                                                        rotate90();
                                                                                        while (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));
                                                                                        rotate90();

                                                                                        CLOSED_FLAG = 0;
                                                                                    }
                                                                                }
                                                                            }
            //                                                            }

                                                                        ADC12_A_enableInterrupt(ADC12_A_BASE, ADC12IE14);

            break;   //Vector 34:  ADC12IFG14
        default: break;
    }
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    if (UCA0RXBUF == 'N')
    {
        received = UCA0RXBUF;
        rotate90();
    }
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}


// ********************************************************************
//                     Total Weight Load cell functions
// ********************************************************************
// Set GPIOs and gain
void begin()
{
    // Enable P2.4 and P2.5 GPIO pins
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);    // P2.4 DOUT
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);   // P2.5 SCK

    // set gain
    amp.gain = 1;    // 2 or 3 based on gain factor

    // Set P2.5 SCK to low
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN5);
}

// Read raw 24-bit data and return 32-bit signed value
// P2.4 DOUT P2.5 SCK
long readRawData ()
{
    // powerOn();                   // turn on load amplifier reading

    // Wait until DOUT is low (wait until data is ready)
    while ((P7IN & BIT6) > 0)    // while P2.4 DOUT is high, do nothing
    {}

    // Pulse the clock pin 24 times to read data from MSB
    uint8_t j = 0;
    data[2] = 0;
    data[1] = 0;
    data[0] = 0;
    // First 8
    for (j = 0; j < 8; j++)
    {
        P7OUT |= BIT5;     // P2.5 SCK = HIGH
        data[2] |= ((P7IN & BIT6) >> 6) << (7-j);
        P7OUT &= ~BIT5;    // P2.5 SCK= LOW
    }
    // Second 8
    for (j = 0; j < 8; j++)
    {
        P7OUT |= BIT5;     // P2.5 SCK= HIGH
        data[1] |= ((P7IN & BIT6) >> 6) << (7-j);
        P7OUT &= ~BIT5;    // P2.5 SCK= LOW
    }
    // Third 8
    for (j = 0; j < 8; j++)
    {
        P7OUT |= BIT5;     // P2.5 SCK= HIGH
        data[0] |= ((P7IN & BIT6) >> 6) << (7-j);
        P7OUT &= ~BIT5;    // P2.5 SCK= LOW
    }

    // Pulse 1/2/3 more times based on the value of gain
    for (j = 0; j < amp.gain; j++)
    {
        P7OUT |= BIT5;     // P2.5 SCK= HIGH
        P7OUT &= ~BIT5;    // P2.5 SCK= LOW
    }

    // Replicate to pad out a 32-bit signed integer
    if (data[2] & 0x80)
    {
        fill = 0xFF;
    }
    else
    {
        fill = 0;
    }

    // Construct 32-bit signed integer
    value = (long)((unsigned long)(fill) << 24 | (unsigned long)(data[2]) << 16
                                               | (unsigned long)(data[1]) << 8
                                               | (unsigned long)(data[0]));

    // powerDown();                 // turn off amplifier reading for reset

    return (value);
}

// Calculate average by reading raw data for 10 times
long calAverage (uint8_t times)
{
    long sum = 0;
    uint8_t i = 0;
    powerOn();                 // turn on load amplifier reading
    for (i = 0; i < times; i++)
    {
        sum += readRawData();
    }
    powerDown();               // turn off amplifier reading for reset
    averageValue = sum/times;
    return (averageValue);
}

// Get final weight in correct unit
float getFinalWeight (uint8_t times)
{
    return ((calAverage(TIMES) - amp.OFFSET)/amp.SCALE);
}

// Turn on load amplifier reading
void powerOn()
{
    P7OUT &= ~BIT5;    // P2.5 SCK= LOW
}

// Turn off load amplifier reading for reset
void powerDown()
{
    P7OUT &= ~BIT5;    // P2.5 SCK= LOW
    P7OUT |= BIT5;     // P2.5 SCK= HIGH
    delayMicroSecond(20);
}

// *****************************************************************
//     UART function to convert float and send weight difference
// *****************************************************************

// Send weight data using UART
void sendWeight(float input)
{
    unsigned long dwResult = F2DW(input);
    // c[]: Store float number into separate bytes
    // c[0] highest byte, transmitted first
    // c[3] lowest byte, transmitted last
    // c[4] = \0, mark the end, not transmitted
    c[3] = (dwResult & 0xFF);               // lowest byte
    c[2] = ((dwResult >>  8) & 0xFF);
    c[1] = ((dwResult >> 16) & 0xFF);
    c[0] = ((dwResult >> 24) & 0xFF);       // highest byte
    c[4] = '\0';
//    uint8_t i = 0;
//    for (i = 0; i < 4; i++)
//    {
//        UCA1TXBUF = c[i];
//        while(UCA1STAT&UCBUSY);
//    }
    return;
}

//Move the lazy Susan 90 degrees
void rotate90(){
    unsigned int t = 2400;
    int x = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,
                GPIO_PIN1);
    for(x= 0; x<150; x++)  //Loop the forward
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P7,
                    GPIO_PIN4); //Trigger one step forward
        delayMicroSecond(t);
        GPIO_setOutputLowOnPin(GPIO_PORT_P7,
                    GPIO_PIN4); //Pull step pin low so it can be triggered again
        delayMicroSecond(t);

        if(t>1150)
          t = t-75;
      }
    delayMicroSecond(10000000);
    delayMicroSecond(10000000);
    delayMicroSecond(10000000);

    GPIO_setOutputHighOnPin(GPIO_PORT_P2,
                    GPIO_PIN1);
}

void rotate1(){
    unsigned int t = 2400;
    int x = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,
                GPIO_PIN1);
    for(x= 0; x<10; x++)  //Loop the forward
      {
        GPIO_setOutputHighOnPin(GPIO_PORT_P7,
                    GPIO_PIN4); //Trigger one step forward
        delayMicroSecond(t);
        GPIO_setOutputLowOnPin(GPIO_PORT_P7,
                    GPIO_PIN4); //Pull step pin low so it can be triggered again
        delayMicroSecond(t);

        if(t>1150)
          t = t-75;
      }

    GPIO_setOutputHighOnPin(GPIO_PORT_P2,
                    GPIO_PIN1);
}
