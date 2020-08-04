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
 * Purpose  : POSIX Port.
 *
 * Note(s)	: 1) Requires a Single UNIX Specification, Version 3 compliant operating environment and as POSIX:2001 (formally: IEEE Std 1003.1-2001)
 * 			  		On Linux _XOPEN_SOURCE must be defined to at least 600, generally by passing the -D_XOPEN_SOURCE=600 command line option to GCC.
 *            2) Useful reference links:
 *            		- https://man7.org/linux/man-pages/man7/feature_test_macros.7.html
 *            		- http://www.unix.org/version3/single_unix_v3.pdf
 *            		- https://en.wikipedia.org/wiki/POSIX#Versions
 *            		- https://www.gnu.org/software/libc/manual/html_node/Feature-Test-Macros.html
 *            		- https://en.wikipedia.org/wiki/Single_UNIX_Specification
 *            		- https://stackoverflow.com/questions/5582211/what-does-define-gnu-source-imply
 *
 * Language:  C
 */

/*
*******************************************************************************
*                               Includes Files                                *
*******************************************************************************
*/

/*
 *  Quoted from: https://man7.org/linux/man-pages/man7/feature_test_macros.7.html
 *
 * "Defining _XOPEN_SOURCE with a value of 600 or greater produces the same effects as defining _POSIX_C_SOURCE with a value of 200112L
 * 	 or greater.  Where one sees _POSIX_C_SOURCE >= 200112L "
 * */
#ifndef _XOPEN_SOURCE
	#define _XOPEN_SOURCE	600
#endif

#include  <stdio.h>
#include  <pthread.h>
#include  <stdint.h>
#include  <signal.h>
#include  <semaphore.h>
#include  <time.h>
#include  <string.h>
#include  <unistd.h>
#include  <stdlib.h>
#include  <sys/types.h>
#include  <sys/syscall.h>
#include  <sys/resource.h>
#include  <errno.h>
#include "pretty_arch.h"
#include "../../../../kernel/pretty_os.h"

#if  (_POSIX_C_SOURCE < 199309L)	/* Minimal requirement: POSIX.1b standard (IEEE Standard 1003.1b-1993) which includes
 	 	 	 	 	 	 	 	 	 	Priority Scheduling, Real-Time Signals, Clocks, Semaphores ... etc */
#error  "_POSIX_C_SOURCE is required to be at least 199309L"
#endif

/*
*******************************************************************************
*                               Extern Variables	                          *
*******************************************************************************
*/

extern CPU_tWORD    volatile        OS_Running;
extern OS_TASK_TCB* volatile        OS_currentTask;
extern OS_TASK_TCB* volatile        OS_nextTask;

/*
*******************************************************************************
*                          Extern Function Prototypes	                      *
*******************************************************************************
*/

extern void OS_TimerTick  (void);
extern void OS_IntEnter   (void);
extern void OS_IntExit    (void);

/*
*******************************************************************************
*                               Local Macros                                  *
*******************************************************************************
*/

#define PRIO_THREAD_CREATION	50U		/* Priority value for all POSIX threads.								*/

										/* A common macro to terminate in case if error is returned.			*/
#define ERROR_CHECK(func)      do {	int res = func; \
									if (res != 0u) { \
										printf("Error in call '%s' from %s(): %s­\r\n", #func, __FUNCTION__, strerror(res)); \
										perror("'errno' indicates "); \
										raise(SIGABRT); \
									} \
								} while(0)


/*
*******************************************************************************
*                               Local Structures                              *
*******************************************************************************
*/

typedef struct os_task_tcb_posix	OS_TCB_POSIX;

struct os_task_tcb_posix
{
	pthread_t 	thread;						/*POSIX thread that acts as a wrapper for PrettyOS task.										*/
	sem_t		sem_TaskCreated;			/*Protect task creation critical section.														*/
	sem_t		sem_CtxSW;					/* Stop/Resume POSIX thread using a semaphore, acting like a context switcher to other threads. */
};

/*
*******************************************************************************
*                              Local Variables                                *
*******************************************************************************
*/



/*
*******************************************************************************
*                           Local Function Prototypes                         *
*******************************************************************************
*/

static void* OS_TaskPosixWrapper 		 (void  *p_arg);
static void* CPU_TaskPosixTimerInterrupt (void  *p_arg);

/*
*******************************************************************************
*                           Functions Implementation                          *
*******************************************************************************
*/

/*
*******************************************************************************
*                         Critical Section Functions	   					  *
*******************************************************************************
*/

void CPU_InterruptInit (void)
{

}

void CPU_InterruptDisable (void)
{

}

void CPU_InterruptEnable (void)
{

}

/*
*******************************************************************************
*                           	Hook Functions	   							  *
*******************************************************************************
*/

/*
 * Function:  OS_Init_CPU_Hook
 * --------------------------------
 * This function is called at the beginning of OS_Init().
 *
 * Note(s)	:	1)	Interrupts should be disabled during this call.
 *
 */
void OS_Init_CPU_Hook (void)
{
    struct  rlimit  rtprio_limits;

    ERROR_CHECK(getrlimit(RLIMIT_RTPRIO, &rtprio_limits));
    if (rtprio_limits.rlim_cur != RLIM_INFINITY) {
        printf("Error: The maximum real time priority for processes must be increased. Set to 'unlimited' via 'ulimit -r' or modify /etc/security/limits.conf\r\n");
        printf("by adding to the end of the file: <user_name> - rtprio unlimited.\r\n");
        printf("Replace <user_name> with your login name.\r\nOnce you save the changes, log out of your original session and then log back in.\r\n");
        exit(-1);
    }

    CPU_InterruptInit();													/*	Setup the fake critical section scheme.													*/
}

/*
 * Function:  OS_TaskCreate_CPU_Hook
 * --------------------------------
 * This function is called when a task is created.
 *
 * Arguments:	ptcb	is a Pointer to the task TCB of the task being created.
 *
 * Returns	:	None.
 *
 * Note(s)	:	1)	Interrupts should be disabled during this call.
 *
 */
void OS_TaskCreate_CPU_Hook (OS_TASK_TCB*	ptcb)
{
	OS_TCB_POSIX*			ptcbPosix;
    pthread_attr_t       	attr;
    struct sched_param   	param_sched;
    int 					ret;

	ptcbPosix	=	(OS_TCB_POSIX*)malloc(sizeof(OS_TCB_POSIX));			/* Allocate a storage for OS_TCB_POSIX Object.												*/

	if(ptcbPosix == OS_NULL(OS_TCB_POSIX))
	{
		printf("Cannot Allocate a memory for OS_TCB_POSIX object\n");
		raise(SIGABRT);
	}

	ptcb->OSTCBExtension = (void*)ptcbPosix;								/* Save OS_TCB_POSIX object for later use.		 											*/
	ERROR_CHECK(sem_init(&ptcbPosix->sem_TaskCreated, 0u, 0u));				/* Initial semaphore value to 0.															*/
	ERROR_CHECK(sem_init(&ptcbPosix->sem_CtxSW, 0u, 0u));					/* Initial semaphore value to 0.															*/


    if (PRIO_THREAD_CREATION < sched_get_priority_min(SCHED_RR) ||			/* Is the priority value in the allowable range. ? 											*/
    		PRIO_THREAD_CREATION > sched_get_priority_max(SCHED_RR)) {
        printf("Cannot Create a POSIX thread with the specified priority = %d\n",PRIO_THREAD_CREATION);
        raise(SIGABRT);
    }

    param_sched.__sched_priority = PRIO_THREAD_CREATION;					/* Set the priority of the POSIX thread.													*/

    ERROR_CHECK(pthread_attr_init(&attr));
    ERROR_CHECK(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED));	/*Take scheduling attributes from &attr object.											*/
    ERROR_CHECK(pthread_attr_setschedpolicy(&attr, SCHED_RR));				/* Set Round-Robin Scheduler for the created Thread.										*/
    ERROR_CHECK(pthread_attr_setschedparam(&attr, &param_sched));			/* Set the scheduling attributes from &param object.										*/

    ERROR_CHECK(pthread_create(&ptcbPosix->thread, &attr,					/* Create a POSIX thread for the created OS_TASK_TCB object.								*/
    		OS_TaskPosixWrapper, (void*)ptcb));

    do {
        ret = sem_wait(&ptcbPosix->sem_TaskCreated);                    	/* Wait for the POSIX thread to pass its initial critical section. 							*/
        if (ret != 0 && errno != EINTR) {									/* Continue to wait even if you're interrupted by another system call errno	value.			*/
			ERROR_CHECK(ret);												/* This explains the possible issue:
            																 * https://stackoverflow.com/questions/41474299/checking-if-errno-eintr-what-does-it-mean	*/
        }
    } while (ret != 0);
}

/*
 * Function:  OS_TaskCreate_CPU_Hook
 * --------------------------------
 * This function is called when a task is deleted.
 *
 * Arguments:	ptcb	is a Pointer to the task TCB of the task being deleted.
 *
 * Returns	:	None.
 *
 * Note(s)	:	1)	Interrupts should be disabled during this call.
 *
 */
void OS_TaskDelete_CPU_Hook	(OS_TASK_TCB*	ptcb)
{
	OS_TCB_POSIX* ptcbPosix = (OS_TCB_POSIX*)ptcb->OSTCBExtension;

	if (!pthread_equal(pthread_self(), ptcbPosix->thread)) {				/* If current thread is not the requested ptcbPosix thread, then ... 						*/
		ERROR_CHECK(pthread_cancel(ptcbPosix->thread));						/* ... Send a cancellation request to ptcbPosix thread to terminate.						*/
	 }

	free(ptcb->OSTCBExtension);												/* Deallocate the object from the memory.													*/
}

void OS_TaskCtxSW_CPU_Hook (void)
{

}

void OS_TimerTick_CPU_Hook (void)
{

}

/*
*******************************************************************************
*                          OS_CPU_* Functions	   							  *
*******************************************************************************
*/

CPU_tSTK* OS_CPU_TaskStackInit(void (*TASK_Handler)(void* params),
                             	 void *params,
								 CPU_tSTK* pStackBase,
								 CPU_tSTK_SIZE  stackSize)
{
    return (pStackBase);									/* Return the base address of the stack since we're not need the defined user stack area.  				*/
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

    OS_IntEnter();          												/* Notify that we are entering an ISR.         		 					*/

    OS_CRTICAL_END();

    OS_TimerTick();         												/* Signal the tick to the OS_timerTick().       						*/

    OS_IntExit();           												/* Notify that we are leaving the ISR.          						*/
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
    pthread_t            thread;
    pthread_attr_t       attr;
    struct  sched_param  param;

    param.__sched_priority = sched_get_priority_max(SCHED_RR);				/* Set the timer POSIX thread to has the max priority among other threads*/

    ERROR_CHECK(pthread_attr_init(&attr));
    ERROR_CHECK(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED));
    ERROR_CHECK(pthread_attr_setschedpolicy(&attr, SCHED_RR));
    ERROR_CHECK(pthread_attr_setschedparam(&attr, &param));


    ERROR_CHECK(pthread_create(&thread, &attr,
    		CPU_TaskPosixTimerInterrupt, (void*)&ticks));					/* Create the timer thread.												*/
}

void OS_CPU_ContexSwitch (void)
{
	OS_TCB_POSIX*	ptcbPosix_old;
	OS_TCB_POSIX*	ptcbPosix_new;
    CPU_t08U        current_deleted;
    int             ret;

    OS_TaskCtxSW_CPU_Hook();												/* Call Task Context Switch Hook.										*/

    ptcbPosix_new = (OS_TCB_POSIX*)OS_nextTask->OSTCBExtension;				/* Retrieve the saved TCB POSIX structure. 								*/
    ptcbPosix_old = (OS_TCB_POSIX*)OS_currentTask->OSTCBExtension;			/* Retrieve the saved TCB POSIX structure. 								*/

    current_deleted = OS_FAlSE;
    if (OS_currentTask->TASK_Stat == OS_TASK_STAT_DELETED) {				/*Are we context switched out from a deleted task ? 					*/
    	current_deleted = OS_TRUE;											/*... Yes.																*/
    }

    OS_currentTask = OS_nextTask;											/* Set the next scheduled task to be the current.				 		*/

    ERROR_CHECK(sem_post(&ptcbPosix_new->sem_CtxSW));						/* Post Context Switch semaphore to switch to the new task.				*/

    if (current_deleted == OS_FAlSE) {										/* If we're not switched out from a deleted task ...					*/
        do {
            ret = sem_wait(&ptcbPosix_old->sem_CtxSW);						/* ... pend on it until it's scheduled again.					 		*/
            if (ret != 0 && errno != EINTR) {								/* even we're interrupted a by another system call errno value.			*/
                raise(SIGABRT);
            }
        } while (ret != 0);
    }																		/* If it was deleted or accidentally returned, OS_TaskDelete() is called
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * whether in OS_TaskPosixWrapper() or OS_TaskDelete_CPU_Hook() which is
    																		 * called by OS_TaskDelete().											*/
}

void OS_CPU_InterruptContexSwitch (void)
{
    if(OS_nextTask != OS_currentTask)       								/* No context switch is required if the current task is the highest.    */
    {
        OS_CPU_ContexSwitch();
    }
}

void OS_CPU_FirstStart (void)
{
    OS_TCB_POSIX*	ptcbPosix;
    sigset_t        sig_set;
    int             signo;


    OS_TaskCtxSW_CPU_Hook();												/* Call Task Context Switch Hook.										*/

    OS_currentTask = OS_nextTask;											/* Since it's the first Context Switch, OS_currentTask should be NULL.	*/

    ptcbPosix 	= (OS_TCB_POSIX*)OS_currentTask->OSTCBExtension;			/* Retrieve the saved TCB POSIX structure. 								*/
    OS_Running	= OS_TRUE;													/* Active OS_Running state.												*/

    ERROR_CHECK(sem_post(&ptcbPosix->sem_CtxSW));							/* Active the first task context switch by posting the semaphore value.	*/

    														/*     The following code mimics CPU interrupts are enabled and the game of context switch has begun.  		*/
    ERROR_CHECK(sigemptyset(&sig_set));										/* Clear an empty set of POSIX signals.									*/
    ERROR_CHECK(sigaddset(&sig_set, SIGTERM));								/* Add a termination request signal to the set.							*/
    ERROR_CHECK(sigwait(&sig_set, &signo));									/* Pend here until we receive a termination signal.						*/
    														/*     Never reaches here, since it has been received a TERM signal like from `kill` or `killall` CLI.		*/
}

/*
*******************************************************************************
*                          		Local Functions	   							  *
*******************************************************************************
*/

/*
 * Function:  OS_TaskPosixWrapper
 * --------------------
 * This function works as a generic POSIX task wrapper for prettyOS tasks.
 *
 * Arguments    : p_arg_tcb		is a pointer to the argument of type OS_TASK_TCB* which represent the prettyOS task internal structure.
 *
 * Returns      : NULL.			If the prettyOS launched task has been returned.
 */
static void* OS_TaskPosixWrapper (void  *p_arg_tcb)
{
	OS_TCB_POSIX*	ptcbPosix;
	OS_TASK_TCB*	ptcb;
	int 			ret;

	ptcb		= (OS_TASK_TCB*)p_arg_tcb;
	ptcbPosix	= (OS_TCB_POSIX*)ptcb->OSTCBExtension;		/* Retrieve the saved TCB POSIX structure.	 															*/

	ERROR_CHECK(sem_post(&ptcbPosix->sem_TaskCreated));		/* Ends the creation of the task's critical section.													*/

	ret = -1U;
	while (ret != 0u) {
		ret = sem_wait(&ptcbPosix->sem_CtxSW);              /* Wait until first context switch is triggered by OS_CPU_FirstStart().                          		*/
		if (ret != 0 && errno != EINTR) {					/* Continue to wait even if you're interrupted by another system call errno	value.						*/
			ERROR_CHECK(ret);
		}
	}
															/* Call the real user task.																				*/
	((void (*)(void *))ptcb->TASK_EntryAddr)(ptcb->TASK_EntryArg);

	OS_TaskDelete(ptcb->TASK_priority);						/* The task can be ended after its context switch.
																Internally, it calls OS_TaskDelete_CPU_Hook() which cleans pthread resources.						*/

	return NULL;
}

/*
 * Function:  CPU_TaskPosixTimerInterrupt
 * --------------------
 * This function simulates the occurrence of timer interrupt every definable time.
 *
 * Arguments    : p_arg			(not used)
 *
 * Returns      : NULL.
 */
static void* CPU_TaskPosixTimerInterrupt (void  *p_arg)
{
    struct  timespec    tspec, tspec_rem;
    int                 res;

    (void)p_arg;																 /* Not used																		*/

    tspec.tv_nsec = (10e9)/OS_TICKS_PER_SEC;									 /* Regardless the CPU frequency, This simple formula gets the right periodic interval
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	for this POSIX thread to sleep and fire the system timer interrupt handler at
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	the end of the period.
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	For OS_TICKS_PER_SEC = 100, gives a timer interrupt every 10 milliseconds.
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	The math prove:
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	Timer_Load_Value = F_CPU/tickRate, tickRate is OS_TICKS_PER_SEC
     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	fire_time = Timer_Load_Value/F_CPU = 1/tickRate seconds. = 10e9/tickRate ns.	*/
    tspec.tv_sec  = 0U;

    do {
    	tspec_rem = tspec;

    	do {
    		res = clock_nanosleep(CLOCK_MONOTONIC, 0u, &tspec_rem, &tspec_rem);	/* Wait for the specified time from now by using  CLOCK_MONOTONIC					*/
    		/*
    		 * Quoted from : https://man7.org/linux/man-pages/man2/clock_nanosleep.2.html
    		 *
    		 * "clock_nanosleep() suspends the execution of the calling thread until either at least the
    		 * time specified by request has elapsed, or a signal is delivered that causes a signal handler
    		 * to be called or that terminates the process.
    		 *
    		 * If the call is interrupted by a signal handler, clock_nanosleep() fails with the error EINTR.
        	 * In addition, if remain is not NULL, and flags was not TIMER_ABSTIME, it returns the remaining
        	 * unslept time in remain.  This value can then be used to call clock_nanosleep() again and complete a (relative) sleep."
        	 *
        	 * */

    	} while (res == EINTR);													/* If you get interrupt by a system call, continue to wait for the remaining time.	*/


    	if (res != 0U)															/* Raise abort signal if unexpected return is found.								*/
    	{
    		raise(SIGABRT);
    	}

    	OS_CPU_SystemTimerHandler();											/* Call the port specific function to signal a system tick occurrence.				*/

    } while (1);																/* Forever loop to acts as a multi shot timer.										*/

    pthread_exit(NULL);															/* EXIT in case of unexpected behavior.												*/

    return (NULL);																/* Should never return !															*/
}
