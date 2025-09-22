/*
 * Board interface routines for the RP2040
 * Copyright (c) 2024 XUM1541 RP2040 Port
 * Based on board-promicro.c
 * Copyright (c) 2015 Marko Solajic <msolajic@gmail.com>
 * Copyright (c) 2014 Thomas Kindler <mail_xum@t-kindler.de>
 * Copyright (c) 2009-2010 Nate Lawson <nate@root.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include "xum1541.h"

#ifdef DEBUG
#include <stdio.h>
// For RP2040, debug output goes to USB CDC or UART
// We'll use the standard printf which routes to USB CDC by default
#endif // DEBUG

// Timer for 100ms (10 Hz) intervals
static volatile bool timer_fired_flag = false;
static const uint32_t TIMER_INTERVAL_MS = 100;
static struct repeating_timer timer;

// Timer callback function - called every 100ms by hardware timer
static bool timer_callback(struct repeating_timer *t)
{
    (void)t; // Suppress unused parameter warning
    timer_fired_flag = true;
    return true; // Keep repeating
}

// Initialize the board (timer, indicator LED, UART)
void
xum_board_init(void)
{
    // Initialize GPIO for LED
    gpio_init(GPIO_LED);
    gpio_set_dir(GPIO_LED, GPIO_OUT);
    gpio_put(GPIO_LED, 1); // Turn on LED initially

    // Setup hardware timer for 100ms intervals (10 Hz)
    add_repeating_timer_ms(-TIMER_INTERVAL_MS, timer_callback, NULL, &timer);
}

// Initialize the board IO ports for IEC mode
// This function has to work even if the ports were left in an indeterminate
// state by a prior initialization (e.g., auto-probe for IEEE devices).
void
board_init_iec(void)
{
    // Initialize IEC GPIO pins
    gpio_init(GPIO_DATA);
    gpio_init(GPIO_CLK);
    gpio_init(GPIO_ATN);
    gpio_init(GPIO_SRQ);
    gpio_init(GPIO_RESET);

    // Set all IEC lines as inputs (released state)
    // Make them pull lines low/active if set to output
    // External pullups will handle the high state
    gpio_set_dir(GPIO_DATA, GPIO_IN);
    gpio_set_dir(GPIO_CLK, GPIO_IN);
    gpio_set_dir(GPIO_ATN, GPIO_IN);
    gpio_set_dir(GPIO_SRQ, GPIO_IN);
    gpio_set_dir(GPIO_RESET, GPIO_IN);
    gpio_put(GPIO_DATA, 0);
    gpio_put(GPIO_CLK, 0);
    gpio_put(GPIO_ATN, 0);
    gpio_put(GPIO_SRQ, 0);
    gpio_put(GPIO_RESET, 0);

    // Disable internal pullups/pulldowns on IEC lines
    // External hardware should provide appropriate pull resistors
    gpio_set_pulls(GPIO_DATA, false, false);
    gpio_set_pulls(GPIO_CLK, false, false);
    gpio_set_pulls(GPIO_ATN, false, false);
    gpio_set_pulls(GPIO_SRQ, false, false);
    gpio_set_pulls(GPIO_RESET, false, false);

    // Initialize parallel port pins
    for (int i = GPIO_PAR_D0; i <= GPIO_PAR_D7; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }
}

uint8_t
iec_poll_pins(void)
{
    uint32_t gpio_state = gpio_get_all();
    uint8_t result = 0;

    // Convert GPIO state to IEC line state
    // Remember: IEC lines are active low, so invert the logic
    if (!(gpio_state & IO_DATA))  result |= IEC_DATA;
    if (!(gpio_state & IO_CLK))   result |= IEC_CLOCK;
    if (!(gpio_state & IO_ATN))   result |= IEC_ATN;
    if (!(gpio_state & IO_SRQ))   result |= IEC_SRQ;
    if (!(gpio_state & IO_RESET)) result |= IEC_RESET;

    return result;
}

/*
 * Callback for when the timer fires.
 * Update LEDs or do other tasks that should be done about every ~100 ms
 */
void
board_update_display(uint8_t status)
{
    switch (status) {
    case STATUS_INIT:
        gpio_put(GPIO_LED, 1); // Turn on LED
        break;
    case STATUS_READY:
        gpio_put(GPIO_LED, 0); // Turn off LED
        break;
    case STATUS_ACTIVE:
    case STATUS_ERROR:
        // Toggle LED
        gpio_put(GPIO_LED, !gpio_get(GPIO_LED));
        break;
    default:
        DEBUGF(DBG_ERROR, "badstsval %d\n", status);
        break;
    }
}

/*
 * Signal that the board_update_display() should be called if the timer
 * has fired (every ~100 ms).
 */
bool
board_timer_fired()
{
    // Check if hardware timer has fired and clear the flag (like AVR's OCF1A)
    if (timer_fired_flag) {
        timer_fired_flag = false;
        return true;
    } else {
        return false;
    }
}
