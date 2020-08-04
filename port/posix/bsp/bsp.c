/*****************************************************************************
MIT License

Copyright (c) 2020 Yahia Farghaly Ashour

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

/*
 * Author   : Yahia Farghaly Ashour
 *
 * Purpose  : Wrapper implementation of BSP APIs according to POSIX standards.
 *
 * Language:  C
 */

/*
*******************************************************************************
*                               Includes Files                                *
*******************************************************************************
*/

#include <bsp.h>                    /* BSP Exposed APIs.                        */
#include <stdio.h>					/* Standard I/O C routines.					*/
#include <stdlib.h>					/* Standard C routines.						*/
#if (_POSIX_C_SOURCE >= 199309L)
	#include <time.h>   			/* for nanosleep 							*/
#else
	#include <unistd.h> 			/* for usleep 								*/
#endif

/*
*******************************************************************************
*                               BSP Macros                                    *
*******************************************************************************
*/

/* Running System Clock in Hertz. */
#define SYS_CLOCK_HZ    16000000U

void
BSP_HardwareSetup(void) {
		/* 	EMPTY	*/
}

void delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}

void
BSP_DelayMilliseconds (unsigned long ms) {
//#if (_POSIX_C_SOURCE >= 199309L)
//    struct timespec ts;
//    ts.tv_sec = ms / 1000U;
//    ts.tv_nsec = (ms % 1000U) * 1000000U;
//    nanosleep(&ts, NULL);
//#else
//    usleep(ms * 1000U);
//#endif
	delay(ms);
}

/*
 * Simple implementation like what should happens if it was on a bare metal.
 * */
void
BSP_UART_SendByte(const unsigned char cData)
{
	putchar((int)cData);				/* Send the char to stdout.												*/
	fflush(stdout);						/* Flush the write buffer. So it doesn't wait for buffer to be full.	*/
}

void
BSP_UART_ClearVirtualTerminal (void)
{
	system("clear");
}

void
BSP_LED_RedOn(void) {
	/* 	EMPTY	*/
}

void
BSP_LED_RedOff(void) {
	/* 	EMPTY	*/
}

void
BSP_LED_BlueOn(void) {
	/* 	EMPTY	*/
}

void
BSP_LED_BlueOff(void) {
	/* 	EMPTY	*/
}

void
BSP_LED_GreenOn(void) {
	/* 	EMPTY	*/
}

void
BSP_LED_GreenOff(void) {
	/* 	EMPTY	*/
}

unsigned long
BSP_CPU_FrequencyGet(void)
{
    return SYS_CLOCK_HZ;
}

void
BSP_onFailure(char const *module, int location)
{
    printf("BSP Failure at module: %s, LOC: %d\n",module,location);
    for(;;);
}

void
BSP_CPU_WFI (void)
{
	/* 	EMPTY	*/
}

void BSP_CPU_NOP (void)
{
	/* 	EMPTY	*/
}
