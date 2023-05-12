/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_SOCKET

#include "ping.h"
#include "ptpd.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_phy.h"

#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "fsl_device_registers.h"

#include "fsl_uart.h"
#include "fsl_uart_freertos.h"

/* SEGGER SystemView includes */
#include "SEGGER_RTT.h"

#include "time.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* SEGGER SystemView definitions */
extern SEGGER_RTT_CB _SEGGER_RTT;

/* @TEST_ANCHOR */

/* IP address configuration. */
#ifndef configIP_ADDR0
#define configIP_ADDR0 192
#endif
#ifndef configIP_ADDR1
#define configIP_ADDR1 168
#endif
#ifndef configIP_ADDR2
#define configIP_ADDR2 0
#endif
#ifndef configIP_ADDR3
#define configIP_ADDR3 50
#endif

/* Netmask configuration. */
#ifndef configNET_MASK0
#define configNET_MASK0 255
#endif
#ifndef configNET_MASK1
#define configNET_MASK1 255
#endif
#ifndef configNET_MASK2
#define configNET_MASK2 255
#endif
#ifndef configNET_MASK3
#define configNET_MASK3 0
#endif

/* Gateway address configuration. */
#ifndef configGW_ADDR0
#define configGW_ADDR0 192
#endif
#ifndef configGW_ADDR1
#define configGW_ADDR1 168
#endif
#ifndef configGW_ADDR2
#define configGW_ADDR2 0
#endif
#ifndef configGW_ADDR3
#define configGW_ADDR3 1
#endif

/* MAC address configuration. */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }
#endif

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static struct netif netif;
static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

static uart_rtos_handle_t handle;
static struct _uart_handle t_handle;
static uint8_t uart_buffer[32];

SemaphoreHandle_t pps_semaphore = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t nmea_checksum(const char *sentence)
{
    uint8_t checksum = 0x00;

    if (*sentence == '$')
        sentence++;

    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

static void nmea_task(void *arg)
{
    uart_rtos_config_t uart_config;
    char rmc[128] = {0};
    int rmc_size;
    uint8_t checksum;
    enet_ptp_time_t ts;
    struct tm *utc;

    NVIC_SetPriority(UART1_RX_TX_IRQn, 5);

    uart_config.base = UART1;
    uart_config.srcclk = CLOCK_GetFreq(UART1_CLK_SRC);
    uart_config.baudrate = 115200;
    uart_config.parity = kUART_ParityDisabled;
    uart_config.stopbits = kUART_OneStopBit;
    uart_config.buffer = uart_buffer;
    uart_config.buffer_size = sizeof(uart_buffer);

    if (kStatus_Success != UART_RTOS_Init(&handle, &t_handle, &uart_config))
        vTaskSuspend(NULL);

    pps_semaphore = xSemaphoreCreateBinary();

    while (1)
    {
        if (xSemaphoreTake(pps_semaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Prepare nmea message. */
            ethernetif_enet_ptptime_gettime(&ts);
            utc = localtime((time_t *)&ts.second);
            sprintf(rmc, "$GPRMC,%02d%02d%02d.%02d,A,0000.00,N,00000.00,E,0.0,0.0,%02d%02d%02d,0.0,E,M*",
                    utc->tm_hour, utc->tm_min, utc->tm_sec, 0,
                    utc->tm_mday, utc->tm_mon + 1, utc->tm_year % 100);
            checksum = nmea_checksum(rmc);
            sprintf(rmc, "%s%X\r\n", rmc, checksum);
            /* Delay for sync time sequence */
            vTaskDelay(20);
            /* Send nmea message. */
            if (kStatus_Success != UART_RTOS_Send(&handle, (uint8_t *)rmc, strlen(rmc)))
            {
                vTaskSuspend(NULL);
            }
        }
    }
}

/*!
 * @brief PHY link check task.
 */
static void phylink_task(void *arg)
{
    phy_handle_t *handle = (phy_handle_t *)arg;
    ENET_Type *base = (ENET_Type *)handle->mdioHandle->resource.base;
    phy_speed_t speed = kPHY_Speed100M;
    phy_duplex_t duplex = kPHY_FullDuplex;
    bool autonego = false;
    bool link = false;
    bool link_last = false;

    while (1)
    {
        link_last = link;
        PHY_GetLinkStatus(handle, &link);

        if (link != link_last)
        {
            if (link)
            {
                /* Turn on LwIP stack */
                netifapi_netif_set_link_up(&netif);
                /* Set enet mac speed */
                PHY_GetAutoNegotiationStatus(handle, &autonego);
                if (autonego)
                {
                    PHY_GetLinkSpeedDuplex(handle, &speed, &duplex);
                    ENET_SetMII(base, (enet_mii_speed_t)speed, (enet_mii_duplex_t)duplex);
                    PRINTF("phylink: Link is up - %s/%s\r\n",
                           speed == kPHY_Speed10M ? "10M" : "100M",
                           duplex == kPHY_HalfDuplex ? "Half" : "Full");
                }
            }
            else
            {
                /* Turn off LwIP stack */
                netifapi_netif_set_link_down(&netif);
                PRINTF("phylink: Link is down\r\n");
            }
        }

        sys_msleep(500);
    }
}

/*!
 * @brief Initializes lwIP stack.
 */
static void stack_init(void *arg)
{
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    LWIP_UNUSED_ARG(arg);
    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" PING example\r\n");
    PRINTF("************************************************\r\n");
    PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
           ((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]);
    PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
           ((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]);
    PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
           ((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]);
    PRINTF("************************************************\r\n");

    /* Enable PHY link check */
    if (sys_thread_new("phy_thread", phylink_task, &phyHandle, 512, DEFAULT_THREAD_PRIO) == NULL)
        LWIP_ASSERT("phy_thread: Task creation failed.", 0);

    if (sys_thread_new("nmea_thread", nmea_task, NULL, 1024, DEFAULT_THREAD_PRIO) == NULL)
        LWIP_ASSERT("nmea_thread: Task creation failed.", 0);

    /* Initialize the PTP daemon. */
    ptpdInit();

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    /* Initialize SystemView */
    SEGGER_SYSVIEW_Conf();
    PRINTF("RTT block address is: 0x%x \r\n", &_SEGGER_RTT);

    /* Initialize color LEDs */
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
        LWIP_ASSERT("main(): Task creation failed.", 0);

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}
#endif
