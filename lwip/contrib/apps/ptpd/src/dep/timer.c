/* timer.c */

#include "../ptpd.h"
#include "timers.h"

TimerHandle_t ptpd_timers[TIMER_ARRAY_SIZE];
Boolean ptpd_timers_expired[TIMER_ARRAY_SIZE];

static void timerCallback(TimerHandle_t xTimer)
{
    UInteger32 index;

    configASSERT(xTimer);

    index = (UInteger32)pvTimerGetTimerID(xTimer);
    if (index < TIMER_ARRAY_SIZE)
    {
        ptpd_timers_expired[index] = TRUE;
        ptpdEventNotify();
    }
}

void initTimer(void)
{
    DBG("initTimer\n");

    /* Create the various timers used in the system. */
    for (UInteger32 i = 0; i < TIMER_ARRAY_SIZE; i++)
    {
        ptpd_timers_expired[i] = FALSE;
        ptpd_timers[i] = xTimerCreate(NULL, 1, pdTRUE, (void *)i, timerCallback);
    }
}

void timerStop(UInteger16 index, IntervalTimer *itimer)
{
    if (index >= TIMER_ARRAY_SIZE)
        return;

    if (xTimerIsTimerActive(ptpd_timers[index]) == pdTRUE)
    {
        xTimerStop(ptpd_timers[index], 0);
    }
    ptpd_timers_expired[index] = FALSE;
}

void timerStart(UInteger16 index, UInteger32 interval_ms, IntervalTimer *itimer)
{
    if (index >= TIMER_ARRAY_SIZE)
        return;

    if (interval_ms == 0)
        return;

    ptpd_timers_expired[index] = FALSE;
    if (xTimerChangePeriod(ptpd_timers[index], interval_ms, 0) == pdPASS)
    {
        DBGV("osTimer: set timer %d to %d\n", index, interval_ms);
    }
}

Boolean timerExpired(UInteger16 index, IntervalTimer *itimer)
{
    if (index >= TIMER_ARRAY_SIZE)
        return FALSE;

    if (!ptpd_timers_expired[index])
        return FALSE;

    ptpd_timers_expired[index] = FALSE;
    DBGV("osTimer: timer %u expired\n", index);

    return TRUE;
}
