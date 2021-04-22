/*-
 * $Copyright$
-*/
#include <phisch/log.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if !defined(HOSTBUILD)
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#else
    void taskDISABLE_INTERRUPTS(void);

    typedef unsigned TaskHandle_t;
#endif /* defined(HOSTBUILD) */

void
vAssertCalled(const char * /* p_file */, unsigned int /* p_line */) {
    taskDISABLE_INTERRUPTS();
    for( ;; );
};

void
vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {
    (void) pcTaskName;
    (void) xTask;

    PHISCH_LOG("STACK OVERFLOW in Task '%s'\r\n", pcTaskName);
    
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

void
vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

void
vApplicationIdleHook(void) {
/*
 * From http://www.freertos.org/a00016.html (Sep 3rd, 2014):
 *
 * The idle task can optionally call an application defined hook (or callback)
 * function - the idle hook. The idle task runs at the very lowest priority, so
 * such an idle hook function will only get executed when there are no tasks of
 * higher priority that are able to run. This makes the idle hook function an
 * ideal place to put the processor into a low power state - providing an
 * automatic power saving whenever there is no processing to be performed.
 *
 * The idle hook will only get called if configUSE_IDLE_HOOK is set to 1 within
 * FreeRTOSConfig.h. When this is set the application must provide the hook
 * function with the following prototype:
 *
 *      void vApplicationIdleHook( void );
 *
 * The idle hook is called repeatedly as long as the idle task is running. It is
 * paramount that the idle hook function does not call any API functions that
 * could cause it to block. Also, if the application makes use of the
 * vTaskDelete() API function then the idle task hook must be allowed to
 * periodically return (this is because the idle task is responsible for
 * cleaning up the resources that were allocated by the RTOS kernel to the task
 * that has been deleted).
 */
}

void
vApplicationTickHook(void) {
/*
 * From http://www.freertos.org/a00016.html (Sep 3rd, 2014):
 *
 * The tick interrupt can optionally call an application defined hook (or
 * callback) function - the tick hook. The tick hook provides a convenient place
 * to implement timer functionality.
 *
 * The tick hook will only get called if configUSE_TICK_HOOK is set to 1 within
 * FreeRTOSConfig.h. When this is set the application must provide the hook
 * function with the following prototype:
 *
 *      void vApplicationTickHook( void );
 *
 * vApplicationTickHook() executes from within an ISR so must be very short, not
 * use much stack, and not call any API functions that don't end in "FromISR" or
 * "FROM_ISR".
 *
 * See the demo application file crhook.c for an example of how to use a tick
 * hook.
 */
}

#if defined(HOSTBUILD)

void
vTaskDelete(void) {

}

void
vTaskStartScheduler(void) {

}

void
xTaskGenericNotifyFromISR(void) {

}

void
xTaskNotifyWait(void) {
    
}

#endif /* defined(HOSTBUILD) */

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
