/* ptpd.c */

#include "ptpd.h"
#include "lwip/sys.h"

RunTimeOpts rtOpts;  /* statically allocated run-time configuration data */
PtpClock ptpClock;
ForeignMasterRecord ptpForeignRecords[DEFAULT_MAX_FOREIGN_RECORDS];

sys_sem_t sem;
sys_thread_t thread;

static void ptpdThread(void *arg)
{
    Integer16 ret;

    /* initialize run-time options to default values */
    rtOpts.announceInterval = DEFAULT_ANNOUNCE_INTERVAL;
    rtOpts.syncInterval = DEFAULT_SYNC_INTERVAL;
    rtOpts.clockQuality.clockAccuracy = DEFAULT_CLOCK_ACCURACY;
    rtOpts.clockQuality.clockClass = DEFAULT_CLOCK_CLASS;
    rtOpts.clockQuality.offsetScaledLogVariance = DEFAULT_CLOCK_VARIANCE; /* 7.6.3.3 */
    rtOpts.priority1 = DEFAULT_PRIORITY1;
    rtOpts.priority2 = DEFAULT_PRIORITY2;
    rtOpts.domainNumber = DEFAULT_DOMAIN_NUMBER;
    rtOpts.slaveOnly = SLAVE_ONLY;
    rtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;
    rtOpts.servo.noResetClock = DEFAULT_NO_RESET_CLOCK;
    rtOpts.servo.noAdjust = NO_ADJUST;
    rtOpts.inboundLatency.nanoseconds = DEFAULT_INBOUND_LATENCY;
    rtOpts.outboundLatency.nanoseconds = DEFAULT_OUTBOUND_LATENCY;
    rtOpts.servo.sDelay = DEFAULT_DELAY_S;
    rtOpts.servo.sOffset = DEFAULT_OFFSET_S;
    rtOpts.servo.ap = DEFAULT_AP;
    rtOpts.servo.ai = DEFAULT_AI;
    rtOpts.maxForeignRecords = sizeof(ptpForeignRecords) / sizeof(ptpForeignRecords[0]);
    rtOpts.stats = PTP_TEXT_STATS;
    rtOpts.delayMechanism = DEFAULT_DELAY_MECHANISM;

    /* Initialize run time options with command line arguments*/
    ret = ptpdStartup(&ptpClock, &rtOpts, ptpForeignRecords);
    if (ret != 0)
    {
        ERROR("PTPd initialize failed\n");
        return;
    }

    for (;;)
    {
        if (sys_arch_sem_wait(&sem, 0) != SYS_ARCH_TIMEOUT)
        {
            do
            {
                doState(&ptpClock);
                displayStats(&ptpClock);
                /* if there are still some packets - run stack again */
            } while (netSelect(&ptpClock.netPath, 0) > 0);
        }

        taskYIELD();
    }
}

void ptpdEventNotify(void)
{
    sys_sem_signal(&sem);
}

int ptpdInit(void)
{
    err_t err;

    err = sys_sem_new(&sem, 4);
    if (err != ERR_OK)
    {
        ERROR("PTPd semaphore create failed\n");
        return err;
    }

    thread = sys_thread_new("ptpd_thread", ptpdThread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    if (thread == NULL)
    {
        ERROR("PTPd thread create failed\n");
        return -1;
    }

    return 0;
}
