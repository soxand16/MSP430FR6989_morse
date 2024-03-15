/*******************************************************************************
 *
 * ECE 431: Lab 8 Shell
 *
 ******************************************************************************/
#include <driverlib.h>
#include <string.h>
#include <hal_LCD.h>

// States
#define INITIALIZE          0
#define INPUT_LETTER        1
#define DECODE_LETTER       2
#define DISPLAY_MESSAGE     3
#define CLEAR               4

// Prototypes
void run(void);
void Init_GPIO(void);

volatile unsigned char MORSE_STATE = INITIALIZE;
// Character array (representing tree as array)
const char *characters = "* ETIANMSURWDKGOHVF?L?PJBXCYZQ??54?3???2???????16???????7???8?90";
volatile unsigned char S1buttonDebounce = 0;
volatile unsigned char S2buttonDebounce = 0;
volatile unsigned char S1buttonPressed = 0;
volatile unsigned char S2buttonPressed = 0;
volatile unsigned int holdCount = 0;
volatile unsigned int n = 1;
volatile unsigned int oldn = 0;
char msg[50] = "";
char c;

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 1MHz
        15000,                                  // 50ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PMM_unlockLPM5();

    //enable interrupts
    __enable_interrupt();

    // Initializations
    Init_GPIO();
    Init_LCD();

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    __enable_interrupt();

    displayScrollText(msg);
    // An infinite loop
    while(1)
    {
        run();
    }
}

void run()
{
    switch(MORSE_STATE)
    {
        case INITIALIZE:
            oldn = n;
            n = 1;
            MORSE_STATE = INPUT_LETTER;
            break;
        case INPUT_LETTER:
            if(S1buttonPressed)
            {
                // dot: sends us from node n on tree to node 2*n
                n *= 2;
                S1buttonPressed = 0;
            }
            // If S2 button pressed
            else if(S2buttonPressed)
            {
                // dash: sends us from node n on treen to node 2*n + 1
                n = 2*n + 1;
                S2buttonPressed = 0;
            }
            break;
        case DECODE_LETTER:
            // Initialization code goes here
            if (oldn == (n == 1))
            {
                MORSE_STATE = DISPLAY_MESSAGE;
                break;
            }
            // Decodes Letter
            c = characters[n];
            showChar(c, pos1);
            // Appends to the message
            strncat(msg, &c, 1);
            MORSE_STATE = INITIALIZE;
            break;
        case DISPLAY_MESSAGE:
            displayScrollText(msg);
            // If either button is pushed
            if(S1buttonPressed ^ S2buttonPressed)
            {
                S1buttonPressed = 0;
                S2buttonPressed = 0;
                oldn = 0;
                clearLCD();
                // Keep current message and continue writing
                MORSE_STATE = INITIALIZE;
            }
            break;
        case CLEAR:
            msg[0] = '\0';
            oldn = 0;
            clearLCD();
            MORSE_STATE = INITIALIZE;
            break;
    }
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 :    // Button S1 pressed
            P1OUT |= BIT0;    // Turn LED1 On
            if ((S1buttonDebounce) == 0)
            {
                // Set debounce flag on first high to low transition
                S1buttonDebounce = 1;

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG2 :    // Button S2 pressed
            P9OUT |= BIT7;    // Turn LED2 On
            if ((S2buttonDebounce) == 0)
            {
                // Set debounce flag on first high to low transition
                S2buttonDebounce = 1;

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 : break;
        case P1IV_P1IFG7 : break;
    }
}

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Both button S1 & S2 held down
    if (!(P1IN & BIT1) && !(P1IN & BIT2))
    {
        holdCount++;
        if (holdCount == 40)
        {

            switch(MORSE_STATE)
            {
                case DISPLAY_MESSAGE:
                    MORSE_STATE = CLEAR;
                    break;
                case INPUT_LETTER:
                    MORSE_STATE = DECODE_LETTER;
                    break;
            }
            P1OUT &= ~BIT0;
            P9OUT &= ~BIT7;


        }
    }

    if (holdCount == 0)
    {
        // Button S1 released
        if (P1IN & BIT1 && S1buttonDebounce)
        {
            S1buttonPressed = 1;
            S1buttonDebounce = 0;                                   // Clear button debounce
            P1OUT &= ~BIT0;
        }

        // Button S2 released
        if (P1IN & BIT2 && S2buttonDebounce)
        {
            S2buttonPressed = 1;
            S2buttonDebounce = 0;                                   // Clear button debounce
            P9OUT &= ~BIT7;
        }
    }

    // Both button S1 & S2 released
    if ((P1IN & BIT1) && (P1IN & BIT2))
    {
        S1buttonDebounce = 0;
        S2buttonDebounce = 0;
        // Stop timer A0
        Timer_A_stop(TIMER_A0_BASE);
        holdCount = 0;
    }
}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN5);

    // Configure button S1 (P1.1) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}
