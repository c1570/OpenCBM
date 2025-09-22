/*
 * CPU initialization and timer routines for the RP2040
 * Copyright (c) 2024 XUM1541 RP2040 Port
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef _CPU_RP2040_H
#define _CPU_RP2040_H

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "pico/bootrom.h"

// Initialize the CPU (clock rate, watchdog)
static inline void
cpu_init(void)
{
#ifdef DEBUG
    // Initialize stdio for debug output
    stdio_init_all();
    printf("\nXUM1541 initializing\n");
#endif
    // Enable watchdog timer for 1 second (1000ms)
    watchdog_enable(1000, 1);
}

static inline void
cpu_bootloader_start(void)
{
    // Reset to BOOTSEL mode (equivalent to jumping to bootloader)
    // This will put the RP2040 into USB mass storage mode for firmware updates
    printf("Rebooting to BOOTSEL mode\n");
    sleep_ms(100);
    reset_usb_boot(0, 0);

    // This function does not return
    while (true)
        ;
}

// Timer and delay functions
// RP2040 runs at 125MHz by default, providing excellent timing precision
// busy_wait_us() provides microsecond delays with sub-microsecond accuracy
#define DELAY_MS(x) sleep_ms(x)
#define DELAY_US(x) busy_wait_us(x)

#endif // _CPU_RP2040_H
