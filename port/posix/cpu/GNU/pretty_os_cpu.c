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
*******************************************************************************
*                               Includes Files                                *
*******************************************************************************
*/
#include "pretty_arch.h"

/*
*******************************************************************************
*                               Global/External Data                          *
*******************************************************************************
*/
extern void OS_TaskReturn (void);
extern void OS_TimerTick  (void);
extern void OS_IntEnter   (void);
extern void OS_IntExit    (void);


void CPU_InterruptDisable (void)
{

}

void CPU_InterruptEnable (void)
{

}

void OS_CPU_ContexSwitch (void)
{

}

void OS_CPU_InterruptContexSwitch (void)
{

}

void OS_CPU_FirstStart (void)
{

}

/*
 * Function:  OS_CPU_TaskInit
 * --------------------
 * Initialize the stack frame of the task being created.
 * This function is a processor specific.
 *
 * Called By: OS_CreateTask()
 *
 * Arguments:
 *          TASK_Handler            is a function pointer to the task code.
 *          params                  is a pointer to the user supplied data which is passed to the task.
 *          pStackBase              is a pointer to the bottom of the task stack.
 *          stackSize               is the task stack size.
 *
 * Returns: The new location of the top of stack after the processor insert the registers in the stack
 *          in the proper of order as specified in the Technical Manual of the CPU.
 *
 * Notes:   ARM Cortex-M stack grows from high to low memory address.
 */
CPU_tWORD*
OS_CPU_TaskInit(void (*TASK_Handler)(void* params),
                             void *params,
                             CPU_tWORD* pStackBase,
                             CPU_tWORD  stackSize)
{
    return (pStackBase);
}

/*
 * Function:  OS_CPU_SystemTimerHandler
 * --------------------
 * Handle the system tick interrupt which is used for signaling the system tick
 * to OS_TimerTick().
 *
 * Arguments    : None.
 *
 * Returns      : None.
 */
void OS_CPU_SystemTimerHandler  (void)
{
    CPU_SR_ALLOC();

    OS_CRTICAL_BEGIN();

    OS_IntEnter();          /* Notify that we are entering an ISR.          */

    OS_CRTICAL_END();

    OS_TimerTick();         /* Signal the tick to the OS_timerTick().       */

    OS_IntExit();           /* Notify that we are leaving the ISR.          */
}

/*
 * Function:  OS_CPU_SystemTimerSetup
 * --------------------
 * Initialize the timer which will be used as a system ticker for the OS.
 *
 * Arguments    :   ticks   is the number of ticks count between two OS tick interrupts.
 *
 * Returns      :   None.
 */
void  OS_CPU_SystemTimerSetup (CPU_t32U ticks)
{

}
