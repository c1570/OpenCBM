/*
 * Board interface routines for the RP2040
 * Copyright (c) 2024 XUM1541 RP2040 Port
 * Based on board-promicro.h
 * Copyright (c) 2015 Marko Solajic <msolajic@gmail.com>
 * Copyright (c) 2014 Thomas Kindler <mail_xum@t-kindler.de>
 * Copyright (c) 2009-2010 Nate Lawson <nate@root.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef _BOARD_RP2040_H
#define _BOARD_RP2040_H

#include "hardware/gpio.h"
#include "hardware/timer.h"

// Some hacks to fix compiling on RP2040
#define strncpy_P(dest, src, n) strncpy(dest, src, n)
#define printf_P printf
#define PROGMEM
#define PSTR(a) (a)
#define pgm_read_byte(addr) (*(addr))
#define wdt_reset() watchdog_update()
// interrupt handling using pico-sdk
// Note: These functions don't maintain perfect AVR semantics since save_and_disable_interrupts()
// returns a token that should be used with restore_interrupts(). For compatibility,
// we use a global variable to store the interrupt state.
extern uint32_t saved_interrupt_state;
static inline void cli(void) { saved_interrupt_state = save_and_disable_interrupts(); }
static inline void sei(void) { restore_interrupts(saved_interrupt_state); }

// Initialize the board (timer, indicators, UART)
void xum_board_init(void);
// Initialize the IO ports for IEC mode
void board_init_iec(void);

// GPIO pin assignments for RP2040
// Using consecutive pins for efficient port operations
// Note IEC pins must be <8 since uint8_t is used for GPIO masks throughout the code

#define GPIO_DATA           2  // GP2
#define GPIO_CLK            3  // GP3
#define GPIO_ATN            4  // GP4
#define GPIO_SRQ            5  // GP5
#define GPIO_RESET          6  // GP6

#define GPIO_LED            25 // GP25 (built-in LED on Pico)

// Parallel port pins (8-bit data bus)
// Lower 4 bits: GP10-GP13
// Upper 4 bits: GP14-GP17
#define GPIO_PAR_D0         10
#define GPIO_PAR_D1         11
#define GPIO_PAR_D2         12
#define GPIO_PAR_D3         13
#define GPIO_PAR_D4         14
#define GPIO_PAR_D5         15
#define GPIO_PAR_D6         16
#define GPIO_PAR_D7         17

// Bit masks for IEC signals
#define IO_DATA             (1U << GPIO_DATA)
#define IO_CLK              (1U << GPIO_CLK)
#define IO_ATN              (1U << GPIO_ATN)
#define IO_SRQ              (1U << GPIO_SRQ)
#define IO_RESET            (1U << GPIO_RESET)

#define LED_MASK            (1U << GPIO_LED)

// Parallel port masks
#define PAR_PORT0_MASK      0b111100000000000000  /* GPIO 14-17 (upper 4 bits) */
#define PAR_PORT1_MASK      0b000011110000000000  /* GPIO 10-13 (lower 4 bits) */
#define PAR_PORT_MASK       (PAR_PORT0_MASK | PAR_PORT1_MASK)

// RP2040 GPIO register access
// Using SIO hardware for fastest GPIO access
#include "hardware/structs/sio.h"

// Emulate AVR DDR/PORT/PIN register behavior with RP2040 GPIO
#define IEC_DDR_SET(mask)   gpio_set_dir_out_masked(mask)
#define IEC_DDR_CLR(mask)   gpio_set_dir_in_masked(mask)
#define IEC_PORT_SET(mask)  gpio_set_mask(mask)
#define IEC_PORT_CLR(mask)  gpio_clr_mask(mask)
#define IEC_PIN_READ()      gpio_get_all()

#define SRQ_NIB_SUPPORT 1

/*
 * Use always_inline to override gcc's -Os option. Since we measured each
 * inline function's disassembly and verified the size decrease, we are
 * certain when we specify inline that we really want it.
 */
#define INLINE          static inline __attribute__((always_inline))

/*
 * Routines for getting/setting IEC lines and parallel port.
 */

INLINE uint8_t
iec_get(uint8_t line)
{
    return !(gpio_get_all() & line);
}

INLINE void
iec_set(uint8_t line)
{
    // Set GPIO as output (pulls line low)
    gpio_set_dir_out_masked(line);
}

INLINE void
iec_release(uint8_t line)
{
    // Set GPIO as input (releases line, external pullups handle high state)
    gpio_set_dir_in_masked(line);
}

INLINE void
iec_set_release(uint8_t s, uint8_t r)
{
    iec_set(s);
    iec_release(r);
}

// Make 8-bit port all inputs and read parallel value
INLINE uint8_t
iec_pp_read(void)
{
    uint8_t retval;
    uint32_t gpio_state;

    // Make parallel port pins inputs
    gpio_set_dir_in_masked(PAR_PORT_MASK);

    // Disable internal pullups (external hardware should handle this)
    for (int i = GPIO_PAR_D0; i <= GPIO_PAR_D7; i++) {
        gpio_set_pulls(i, false, false);
    }

    // Read GPIO state
    gpio_state = gpio_get_all();

    // Extract parallel port bits and reconstruct byte
    retval = 0;
    retval |= ((gpio_state >> GPIO_PAR_D0) & 0x01) << 0;
    retval |= ((gpio_state >> GPIO_PAR_D1) & 0x01) << 1;
    retval |= ((gpio_state >> GPIO_PAR_D2) & 0x01) << 2;
    retval |= ((gpio_state >> GPIO_PAR_D3) & 0x01) << 3;
    retval |= ((gpio_state >> GPIO_PAR_D4) & 0x01) << 4;
    retval |= ((gpio_state >> GPIO_PAR_D5) & 0x01) << 5;
    retval |= ((gpio_state >> GPIO_PAR_D6) & 0x01) << 6;
    retval |= ((gpio_state >> GPIO_PAR_D7) & 0x01) << 7;

    return retval;
}

// Make 8-bits of port output and write out the parallel data
INLINE void
iec_pp_write(uint8_t val)
{
    uint32_t gpio_mask = 0;

    // Convert byte to GPIO bit positions
    if (val & 0x01) gpio_mask |= (1U << GPIO_PAR_D0);
    if (val & 0x02) gpio_mask |= (1U << GPIO_PAR_D1);
    if (val & 0x04) gpio_mask |= (1U << GPIO_PAR_D2);
    if (val & 0x08) gpio_mask |= (1U << GPIO_PAR_D3);
    if (val & 0x10) gpio_mask |= (1U << GPIO_PAR_D4);
    if (val & 0x20) gpio_mask |= (1U << GPIO_PAR_D5);
    if (val & 0x40) gpio_mask |= (1U << GPIO_PAR_D6);
    if (val & 0x80) gpio_mask |= (1U << GPIO_PAR_D7);

    // Output val to parallel port pins
    gpio_put_masked(PAR_PORT_MASK, gpio_mask);
    // Make parallel port pins outputs
    gpio_set_dir_out_masked(PAR_PORT_MASK);
}

INLINE uint8_t
iec_srq_read(void)
{
    uint8_t i, data;

    data = 0;
    for (i = 8; i != 0; --i) {
        // Wait for the drive to pull IO_SRQ.
        while (!iec_get(IO_SRQ))
            ;

        // Wait for drive to release SRQ, then delay another 375 ns for DATA
        // to stabilize before reading it.
        while (iec_get(IO_SRQ))
            ;
        DELAY_US(0.375);

        // Read data bit
        data = (data << 1) | (iec_get(IO_DATA) ? 0 : 1);
   }

   return data;
}

/*
 * Write out a byte by sending each bit on the DATA line (inverted) and
 * clocking the CIA with SRQ.
 * At 500 Kbit/sec, each loop iteration should take 2 us.
 */
INLINE void
iec_srq_write(uint8_t data)
{
    for (uint8_t i = 8; i != 0; --i) {
        /*
         * Take the high bit of the data byte.
         * If it's 1, we want to pull DATA low.
         * At 125MHz, we can assume everything but DELAY_US completes immediately.
         */
        if (data & 0x80) {
            // Data bit is 1, so pull DATA low (set as output)
            gpio_set_dir_masked(IO_DATA|IO_SRQ, IO_DATA|IO_SRQ);
        } else {
            // Data bit is 0, so release DATA (set as input)
            gpio_set_dir_masked(IO_DATA|IO_SRQ, IO_SRQ);
        }

        data <<= 1;          // get next bit
        DELAY_US(1);         // (nibtools relies on this timing, do not change)
        gpio_set_dir_in_masked(IO_SRQ); // Release SRQ
        DELAY_US(1);         // (nibtools relies on this timing, do not change)
    }
}

// Since this is called with a runtime-specified mask, inlining doesn't help.
uint8_t iec_poll_pins(void);

// Status indicators (LEDs)
void board_update_display(uint8_t status);
bool board_timer_fired(void);

#endif // _BOARD_RP2040_H
