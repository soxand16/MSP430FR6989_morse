/*******************************************************************************
 *
 * ECE 431: Lab 8 Shell
 *
 ******************************************************************************/
#include <driverlib.h>

/*
 * TODO:
 *  1. Include any additional sources you want.
 *  2. Initialize Variables needed.
 *  3. Prototypes for any functions you write.
 */

int main(void)
{
    // Stop watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);

    // TODO: Configure board (Are you using GPIO, LCD, and/or interrupts?)

    // An infinite loop
    while(1)
    {

    }
}


/*
 * TODO:
 *  1. Write/copy in any functions you want to use.
 *  2. Write code to register when
 *      a. S1 pressed
 *      b. S2 pressed
 *      c. S1 and S2 held (Look at OutOfBox code)
 *  3. Handle debouncing
 */


