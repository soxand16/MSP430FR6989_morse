/*******************************************************************************
 *
 *
 ******************************************************************************/
#include <driverlib.h>
#include "string.h"
// Change based on LCD Memory locations
#define pos1 9   /* Digit A1 begins at S18 */
#define pos2 5   /* Digit A2 begins at S10 */
#define pos3 3   /* Digit A3 begins at S6  */
#define pos4 18  /* Digit A4 begins at S36 */
#define pos5 14  /* Digit A5 begins at S28 */
#define pos6 7   /* Digit A6 begins at S14 */
// LCD memory map for numeric digits
const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};
const char alphabetBig[26][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};
/*
 * TimerA0 UpMode Configuration Parameter
 * Use this struct to set up a 15ms timer
 */
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

// Function foreward declarations
void show_digit(int, int);
void init_GPIO(void);
void init_LCD(void);
void show_Char(int,int);

// Button debounce variables
volatile unsigned char s1Debounce = 0; // makes flag for button Debounce value
volatile unsigned char s2Debounce = 0;
//
volatile unsigned int holdCount = 0;
//variables used to store and run through the morse array input and output
int morseArray[5]= {0,0,0,0,0}; //array of 5 max to hold up to 5 values as 5 is the longest amount of presses for any char
int count = 0; // count to be used for traversal of the array

volatile unsigned int oneTimePress = 0;

int main(void) {
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    // Initialize the input and output pins (GPIO)
    init_GPIO();
    // Initialize LCD
    init_LCD();
    // init state counter
    RTC_C_initCounter(RTC_C_BASE, RTC_C_CLOCKSELECT_32KHZ_OSC, RTC_C_COUNTERSIZE_16BIT);
   // mode = 0; done when made
    //enable interrupts
    __enable_interrupt();
    // An infinite loop
    while(1)
    {

    }
}
//used to display any digit input
void show_digit(int d, int position)
{
    // Write digits to the LCD
     //uses the given position and the digit to display a value.
    LCDMEM[position] = digit[d][0];
    LCDMEM[position + 1] = digit[d][1]; // the index of position, uses two array fields for a 2d array
}
//used to display any letter input
void show_Char(int d, int position){
    LCDMEM[position] = alphabetBig[d][0];//uses the given position and the letter to display a value.
    LCDMEM[position + 1] = alphabetBig[d][1];// the index of position, uses two array fields for a 2d array
}
//interrupt that handles the button press for button one and two independently
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7)){

            case P1IV_P1IFG1 :    // Button S1 pressed
                P1OUT |= BIT0;    // Turn LED1 On
                if ((s1Debounce) == 0){
                    // Set debounce flag on first high to low transition
                    s1Debounce = 1;
                    holdCount = 0;

                    if(oneTimePress >= 2){
                    morseArray[count] = 0;//set value 1 in position of count
                    count++;
                    }
                    oneTimePress++;
                    // Start debounce timer
                    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
                }
                break;

            case P1IV_P1IFG2 :    // Button S2 pressed
                P9OUT |= BIT7;    // Turn LED2 On
                if ((s2Debounce) == 0){
                    // Set debounce flag on first high to low transition
                    s2Debounce = 1;
                    holdCount = 0;

                    if(oneTimePress >= 2){
                    morseArray[count] = 1; ///set value of 0 at position count in the array
                    count++;//increment array position
                    }
                    oneTimePress++;
                     // Start debounce timer
                    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
                }
                break;
            default: break;
    }

}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
// Both button S1 & S2 held down
        if (!(P1IN & BIT1) && !(P1IN & BIT2)){
           holdCount++;
           if (holdCount == 40){
               count--;
               count--;
               switch (count){
                                       case 1:
                                           //display E
                                           if(morseArray[0]== 0){
                                               show_Char(4 , pos1);
                                           }
                                           //display T
                                           if(morseArray[0]==1){
                                               show_Char(19 , pos1);
                                           }

                                           break;

                                       case 2:
                                           //display I
                                           if(morseArray[0]== 0 && morseArray[1] == 0){
                                              show_Char(8 , pos1);
                                           }
                                            //display A
                                           if(morseArray[0]== 0 && morseArray[1]== 1){
                                              show_Char(0 , pos1);
                                           }
                                           //display N
                                           if(morseArray[0]==1 && morseArray[1]==0){
                                               show_Char(13 , pos1);
                                           }
                                           //display M
                                           if(morseArray[0]==1 && morseArray[1]==1){
                                               show_Char(12 , pos1);
                                           }

                                           break;

                                       case 3:
                                           //display S
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0){
                                              show_Char(18 , pos1);
                                           }
                                           //display D
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 0){
                                              show_Char(3 , pos1);
                                           }
                                           //display G
                                           if(morseArray[0]==1 && morseArray[1]== 1 && morseArray[2]== 0){
                                               show_Char(6 , pos1);
                                           }
                                           //display O
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 1){
                                               show_Char(14 , pos1);
                                           }
                                           //display W
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 1){
                                               show_Char(22, pos1);
                                           }
                                           //display U
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 1){
                                               show_Char(20 , pos1);
                                           }
                                           //display R
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 0){
                                              show_Char( 17, pos1);
                                           }
                                           //display K
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 1){
                                              show_Char( 10, pos1);
                                           }

                                           break;

                                       case 4:
                                           //display H
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 0){
                                               show_Char( 7, pos1);
                                           }
                                           //display B
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 0){
                                               show_Char(1 , pos1);
                                           }
                                           //display C
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 1 && morseArray[3]== 0){
                                              show_Char(2 , pos1);
                                           }
                                           //display F
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 1 && morseArray[3]== 0){
                                               show_Char(5,pos1);
                                           }
                                           //display J
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 1){
                                               show_Char(9,pos1);
                                           }
                                           //display L
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 0 && morseArray[3]== 0){
                                              show_Char(11,pos1);
                                           }
                                           //display P
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 0){
                                              show_Char(15,pos1);
                                           }
                                           //display Q
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 0 && morseArray[3]== 1){
                                              show_Char(16,pos1);
                                           }
                                           //display V
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 1){
                                              show_Char(21,pos1);
                                           }
                                           //display X
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 1){
                                              show_Char(23,pos1);
                                           }
                                           //display Y
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 1 && morseArray[3]== 1){
                                              show_Char(24,pos1);
                                           }
                                           //display Z
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 0 && morseArray[3]== 0){
                                              show_Char(25,pos1);
                                           }

                                           break;

                                       case 5:
                                           //display 0
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 1 && morseArray[4]== 1){
                                              show_digit(0, pos1);
                                           }
                                           //display 1
                                           if(morseArray[0]== 0 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 1 && morseArray[4]== 1){
                                               show_digit(1, pos1);
                                           }
                                           //display 2
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 1 && morseArray[3]== 1 && morseArray[4]== 1){
                                               show_digit(2, pos1);
                                           }
                                           //display 3
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 1 && morseArray[4]== 1){
                                              show_digit(3, pos1);
                                           }
                                           //display 4
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 0 && morseArray[4]== 1){
                                              show_digit(4, pos1);
                                           }
                                           //display 5
                                           if(morseArray[0]== 0 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 0 && morseArray[4]== 0){
                                               show_digit(5, pos1);
                                           }
                                           //display 6
                                           if(morseArray[0]== 1 && morseArray[1]== 0 && morseArray[2]== 0 && morseArray[3]== 0 && morseArray[4]== 0){
                                               show_digit(6, pos1);
                                           }
                                           //display 7
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 0 && morseArray[3]== 0 && morseArray[4]== 0){
                                               show_digit(7, pos1);
                                           }
                                           //display 8
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 0 && morseArray[4]== 0){
                                               show_digit(8, pos1);
                                           }
                                           //display 9
                                           if(morseArray[0]== 1 && morseArray[1]== 1 && morseArray[2]== 1 && morseArray[3]== 1 && morseArray[4]== 0){
                                               show_digit(9, pos1);
                                           }

                                           break;
                   }
               count = 0;
               Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
               }
        }
        // Button S1 released
        if (P1IN & BIT1){
            s1Debounce = 0;                                   // Clear button debounce
            P1OUT &= ~BIT0;

            }

        // Button S2 released
        if (P1IN & BIT2){
            s2Debounce = 0;                                   // Clear button debounce
           P9OUT &= ~BIT7;

        }
        // Both button S1 & S2 released
        if ((P1IN & BIT1) && (P1IN & BIT2)){
               // Stop timer A0
               Timer_A_stop(TIMER_A0_BASE);
        }

}
/*
 * GPIO Initialization
 */
void init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating
    // input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P1, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P2, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P3, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P4, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P5, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P6, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P7, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P8, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P9, 0xFF);
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
    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}
void init_LCD()
{
    LCD_C_initParam initParams = {0};
    initParams.clockSource = LCD_C_CLOCKSOURCE_ACLK;
    initParams.clockDivider = LCD_C_CLOCKDIVIDER_1;
    initParams.clockPrescalar = LCD_C_CLOCKPRESCALAR_16;
    initParams.muxRate = LCD_C_4_MUX;
    initParams.waveforms = LCD_C_LOW_POWER_WAVEFORMS;
    initParams.segments = LCD_C_SEGMENTS_ENABLED;
    LCD_C_init(LCD_C_BASE, &initParams);
    // LCD Operation - VLCD generated internally, V2-V4 generated internally, v5 to ground
    /*  'FR6989 LaunchPad LCD1 uses Segments S4, S6-S21, S27-S31 and S35-S39 */
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_4,
                                LCD_C_SEGMENT_LINE_4);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_6,
                                LCD_C_SEGMENT_LINE_21);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_27,
                                LCD_C_SEGMENT_LINE_31);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_35,
                                LCD_C_SEGMENT_LINE_39);
    LCD_C_setVLCDSource(LCD_C_BASE, LCD_C_VLCD_GENERATED_INTERNALLY,
                        LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS,
                        LCD_C_V5_VSS);
    // Set VLCD voltage to 3.20v
    LCD_C_setVLCDVoltage(LCD_C_BASE,
                         LCD_C_CHARGEPUMP_VOLTAGE_3_02V_OR_2_52VREF);
    // Enable charge pump and select internal reference for it
    LCD_C_enableChargePump(LCD_C_BASE);
    LCD_C_selectChargePumpReference(LCD_C_BASE,
                                    LCD_C_INTERNAL_REFERENCE_VOLTAGE);
    LCD_C_configChargePump(LCD_C_BASE, LCD_C_SYNCHRONIZATION_ENABLED, 0);
    // Clear LCD memory
    LCD_C_clearMemory(LCD_C_BASE);
    //Turn LCD on

    LCD_C_on(LCD_C_BASE);
    LCD_C_selectDisplayMemory(LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY);
}
