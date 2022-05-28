/* sys.c */

#include "../ptpd.h"
#include "enet_ethernetif.h"
#include "board.h"

void displayStats(const PtpClock *ptpClock)
{
#if defined(FRDM_K64F)
    int leds = 0;

    switch (ptpClock->portDS.portState)
    {
    case PTP_INITIALIZING:  leds = 1; break;
    case PTP_FAULTY:        leds = 0; break;
    case PTP_LISTENING:     leds = 1; break;
    case PTP_PASSIVE:       leds = 1; break;
    case PTP_UNCALIBRATED:  leds = 1; break;
    case PTP_SLAVE:         leds = 3; break;
    case PTP_PRE_MASTER:    leds = 2; break;
    case PTP_MASTER:        leds = 2; break;
    case PTP_DISABLED:      leds = 0; break;
    default:                leds = 0; break;
    }

    /* Error */
    if (leds == 0)
    {
        LED_RED_ON();
        LED_GREEN_OFF();
        LED_BLUE_OFF();
    }

    /* Unhealth */
    if (leds == 1)
    {
        LED_RED_ON();
        LED_GREEN_ON();
        LED_BLUE_OFF();
    }

    /* Master mode health */
    if (leds == 2)
    {
        LED_RED_OFF();
        LED_GREEN_OFF();
        LED_BLUE_ON();
    }

    /* Slave mode health */
    if (leds == 3)
    {
        LED_RED_OFF();
        LED_GREEN_ON();
        LED_BLUE_OFF();
    }

#else
    char buffer[80];

    const char *s;

    unsigned char * uuid;

    char sign;

    uuid = (unsigned char*)ptpClock->parentDS.parentPortIdentity.clockIdentity;

    /* master clock UUID */
    sprintf(buffer, "%02X%02X:%02X%02X:%02X%02X:%02X%02X",
            uuid[0], uuid[1],
            uuid[2], uuid[3],
            uuid[4], uuid[5],
            uuid[6], uuid[7]);

    LCD_DisplayStringLine(Line4, (uint8_t *)buffer);

    switch (ptpClock->portDS.portState)
    {
    case PTP_INITIALIZING:  s = "init";  break;
    case PTP_FAULTY:        s = "faulty";   break;
    case PTP_LISTENING:     s = "listening";  break;
    case PTP_PASSIVE:       s = "passive";  break;
    case PTP_UNCALIBRATED:  s = "uncalibrated";  break;
    case PTP_SLAVE:         s = "slave";   break;
    case PTP_PRE_MASTER:    s = "pre master";  break;
    case PTP_MASTER:        s = "master";   break;
    case PTP_DISABLED:      s = "disabled";  break;
    default:                s = "?";     break;
    }

    /* state of the PTP */
    sprintf(buffer, "state: %s          \n", s);

    LCD_DisplayStringLine(Line5, (uint8_t *)buffer);

    /* one way delay */
    switch (ptpClock->portDS.delayMechanism)
    {
    case E2E:
        sprintf(buffer, "path delay: %dns          \n", ptpClock->currentDS.meanPathDelay.nanoseconds);
        break;
    case P2P:
        sprintf(buffer, "path delay: %dns          \n", ptpClock->portDS.peerMeanPathDelay.nanoseconds);
        break;
    default:
        sprintf(buffer, "path delay: unknown       \n");
        /* none */
        break;
    }
    LCD_DisplayStringLine(Line6, (uint8_t *)buffer);

    /* offset from master */
    if (ptpClock->currentDS.offsetFromMaster.seconds)
    {
        sprintf(buffer, "offset: %ds           \n", ptpClock->currentDS.offsetFromMaster.seconds);
    }
    else
    {
        sprintf(buffer, "offset: %dns           \n", ptpClock->currentDS.offsetFromMaster.nanoseconds);
    }

    LCD_DisplayStringLine(Line7, (uint8_t *)buffer);

    /* observed drift from master */
    sign = ' ';

    if (ptpClock->observedDrift > 0) sign = '+';

    if (ptpClock->observedDrift < 0) sign = '-';

    sprintf(buffer, "drift: %c%d.%03dppm       \n", sign, abs(ptpClock->observedDrift / 1000), abs(ptpClock->observedDrift % 1000));

    LCD_DisplayStringLine(Line8, (uint8_t *)buffer);

#endif

}

void getTime(TimeInternal *time)
{

    enet_ptp_time_t timestamp;
    ethernetif_enet_ptptime_gettime(&timestamp);
    time->seconds = timestamp.second;
    time->nanoseconds = timestamp.nanosecond;
}

void setTime(const TimeInternal *time)
{

    enet_ptp_time_t ts;
    ts.second = time->seconds;
    ts.nanosecond = time->nanoseconds;

    ethernetif_enet_ptptime_settime(&ts);

    DBG("resetting system clock to %ds %dns\n", time->seconds, time->nanoseconds);
}

UInteger32 getRand(UInteger32 randMax)
{
    return rand() % randMax;
}

Boolean adjFreq(Integer32 adj)
{
    DBGV("adjFreq %d\n", adj);

    if (adj > ADJ_FREQ_MAX)
        adj = ADJ_FREQ_MAX;
    else if (adj < -ADJ_FREQ_MAX)
        adj = -ADJ_FREQ_MAX;

    /* Fine update method */
    ethernetif_enet_ptptime_adjfreq(adj);

    return TRUE;
}
