/*
 * Main loop for at90usb-based devices
 * Copyright (c) 2009-2010 Nate Lawson <nate@root.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include "xum1541.h"

#if MODEL != RP2040
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#endif
#include <string.h>

#if USING_TINYUSB
#include "device/usbd.h"
#include "device/usbd_pvt.h"
#include "class/vendor/vendor_device.h"
#include "common/tusb_fifo.h"

uint8_t xum_rx_fifo_buf[64];
tu_fifo_t xum_rx_fifo = TU_FIFO_INIT(xum_rx_fifo_buf, 64, uint8_t, false);
#endif

#if MODEL == RP2040
// Global variable to store interrupt state for cli()/sei() compatibility
uint32_t saved_interrupt_state;
#endif

// Flag indicating we should abort any in-progress data transfers
volatile bool doDeviceReset;

// Flag for whether we are in EOI state
volatile uint8_t eoi;

// Board status which controls the status indicators (e.g. LEDs)
static volatile uint8_t statusValue;

static bool USB_BulkWorker(void);

int
main(void)
{
    /*
     * Setup the CPU and USB configuration. Wait after power-on for VBUS.
     * This is to handle the case where we are being powered from the
     * IEC bus through the IOs without a USB connection. Nate analyzed the
     * time we run until brownout here and it defaults to 65 ms due to the
     * CKSEL/SUT fuse.
     *
     * Instead of just a delay, we can wait for VBUS to be active and then
     * be sure USB is connected. This works even when we can't use the
     * brown-out detector. On the USBKEY, the BOD can only be 2.6V or 3.4V
     * but it runs at 3.3V.
     */
    cpu_init();
#if USING_TINYUSB
    // Initialize TinyUSB device stack
    tud_init(BOARD_TUD_RHPORT);
    tu_fifo_clear(&xum_rx_fifo);

    // Wait for USB to be ready
    while (!tud_mounted()) {
        tud_task();        // Process TinyUSB events
        watchdog_update(); // Keep watchdog happy
    }
    // Note: xum_board_init() instead of board_init() to avoid symbol
    // conflict with TinyUSB's own board_init() function
    xum_board_init();
#else
    USB_Init();
    while (USB_DeviceState < DEVICE_STATE_Powered)
        wdt_reset();
    board_init();
#endif

    // Indicate device not ready
    set_status(STATUS_INIT);
    doDeviceReset = false;

    // If a CBM 153x tape drive is attached, detect it and enable tape mode.
    // If any IEC/IEEE drives are attached, detect them early.
    // This leaves the IO pins in the proper state to allow them to come
    // out of reset until we're ready to access them.
    cbm_init();

    /*
     * Process bulk transactions as they appear. Control requests are
     * handled separately via IRQs.
     */
    for (;;) {
#if USING_TINYUSB
        // TinyUSB device task
        tud_task();
#endif

        /*
        * Do periodic tasks each command. If we found the device was in
        * a stalled state, reset it before the next command.
        */
        if (!TimerWorker())
            doDeviceReset = false;

        if (usb_is_ready()) {
            /*
             * If we just got configured, we're ready now (status
             * might also be "ACTIVE" here, don't change that).
             */
            if (statusValue == STATUS_INIT)
                set_status(STATUS_READY);

            // Check for and process any commands coming in on the bulk pipe.
            USB_BulkWorker();
        } else {
#if !USING_TINYUSB
            // TODO: save power here when device is not running (LUFA only)
            if (USB_DeviceState < DEVICE_STATE_Addressed) {
#endif
                // Indicate we are not configured
                set_status(STATUS_INIT);
#if !USING_TINYUSB
            }
#endif
        }
    }
}

// Board status for indicators (e.g. LEDs)
uint8_t
get_status()
{
    return statusValue;
}

void
set_status(uint8_t status)
{
    statusValue = status;
    board_update_display(status);
}

#if !USING_TINYUSB
// LUFA USB event handlers
void
EVENT_USB_Device_ConfigurationChanged(void)
{
    DEBUGF(DBG_ALL, "usbconfchg\n");

    // Clear out any old configuration before allocating
    USB_ResetConfig();

    /*
     * Setup and enable the two bulk endpoints. This must be done in
     * increasing order of endpoints (3, 4) to avoid fragmentation of
     * the USB RAM.
     */
    Endpoint_ConfigureEndpoint(XUM_BULK_IN_ENDPOINT, EP_TYPE_BULK,
        ENDPOINT_DIR_IN, XUM_ENDPOINT_BULK_SIZE, ENDPOINT_BANK_DOUBLE);
    Endpoint_ConfigureEndpoint(XUM_BULK_OUT_ENDPOINT, EP_TYPE_BULK,
        ENDPOINT_DIR_OUT, XUM_ENDPOINT_BULK_SIZE, ENDPOINT_BANK_DOUBLE);
}

/*
 * The Linux and OSX call the configuration changed entry each time
 * a transaction is started (e.g., multiple runs of cbmctrl status).
 * We need to reset the endpoints before reconfiguring them, otherwise
 * we get a hang the second time through.
 *
 * We keep the original endpoint selected after returning.
 */
void
USB_ResetConfig()
{
    static uint8_t endpoints[] = {
        XUM_BULK_IN_ENDPOINT, XUM_BULK_OUT_ENDPOINT, 0,
    };
    uint8_t lastEndpoint, *endp;

    lastEndpoint = Endpoint_GetCurrentEndpoint();

    for (endp = endpoints; *endp != 0; endp++) {
        Endpoint_SelectEndpoint(*endp);
        Endpoint_ResetFIFO(*endp);
        Endpoint_ResetDataToggle();
        if (Endpoint_IsStalled())
            Endpoint_ClearStall();
    }

    Endpoint_SelectEndpoint(lastEndpoint);
}

void
EVENT_USB_Device_UnhandledControlRequest(void)
{
    uint8_t replyBuf[XUM_DEVINFO_SIZE];
    int8_t len;

    /*
     * Ignore non-class requests. We also only handle commands
     * that don't transfer any data or just transfer it into the host.
     */
    if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_TYPE) !=
        REQTYPE_CLASS) {
        DEBUGF(DBG_ERROR, "bad ctrl req %x\n",
            USB_ControlRequest.bmRequestType);
        return;
    }

    // Process the command and get any returned data
    memset(replyBuf, 0, sizeof(replyBuf));
    len = usbHandleControl(USB_ControlRequest.bRequest, replyBuf);
    if (len == -1) {
        DEBUGF(DBG_ERROR, "ctrl req err\n");
        set_status(STATUS_ERROR);
        return;
    }

    // Control request was handled so ack it to allow another
    Endpoint_ClearSETUP();

    // Send data back to host and finalize the status phase
    if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_DIRECTION) ==
        REQDIR_DEVICETOHOST) {
        Endpoint_Write_Control_Stream_LE(replyBuf, len);
        Endpoint_ClearOUT();
    } else {
        while (!Endpoint_IsINReady())
            ;
        Endpoint_ClearIN();
    }
}
#endif

// USB IO functions and command handlers
static bool
USB_BulkWorker()
{
    uint8_t cmdBuf[XUM_CMDBUF_SIZE], statusBuf[XUM_STATUSBUF_SIZE];
    int8_t status;
    static uint32_t debug_count = 0;

#if USING_TINYUSB
    if(tu_fifo_empty(&xum_rx_fifo))
        return false;
#else
    /*
     * If we are not connected to the host or a command has not yet
     * been sent, no more processing is required.
     */
    if (USB_DeviceState != DEVICE_STATE_Configured)
        return false;
    Endpoint_SelectEndpoint(XUM_BULK_OUT_ENDPOINT);
    if (!Endpoint_IsReadWriteAllowed())
        return false;

#ifdef DEBUG
    // Dump the status of both endpoints before getting the command
    Endpoint_SelectEndpoint(XUM_BULK_IN_ENDPOINT);
    DEBUGF(DBG_INFO, "bsti %x %x %x %x %x %x %x %x\n",
        Endpoint_GetCurrentEndpoint(),
        Endpoint_BytesInEndpoint(), Endpoint_IsEnabled(),
        Endpoint_IsReadWriteAllowed(), Endpoint_IsConfigured(),
        Endpoint_IsINReady(), Endpoint_IsOUTReceived(), Endpoint_IsStalled());
    Endpoint_SelectEndpoint(XUM_BULK_OUT_ENDPOINT);
    DEBUGF(DBG_INFO, "bsto %x %x %x %x %x %x %x %x\n",
        Endpoint_GetCurrentEndpoint(),
        Endpoint_BytesInEndpoint(), Endpoint_IsEnabled(),
        Endpoint_IsReadWriteAllowed(), Endpoint_IsConfigured(),
        Endpoint_IsINReady(), Endpoint_IsOUTReceived(), Endpoint_IsStalled());
#endif
#endif

    // Read in the command from the host now that one is ready.
    if (!USB_ReadBlock(cmdBuf, sizeof(cmdBuf))) {
        set_status(STATUS_ERROR);
        return false;
    }

    // Allow commands to leave the extended status untouched
    memset(statusBuf, 0, sizeof(statusBuf));

    /*
     * Decode and process the command.
     * usbHandleBulk() stores its extended result in the output buffer,
     * up to XUM_STATUSBUF_SIZE.
     *
     * Return values:
     *   >0: completed ok, send the return value and extended status
     *    0: completed ok, don't send any status
     *   -1: error, no status
     */
    status = usbHandleBulk(cmdBuf, statusBuf);
    DEBUGF(DBG_INFO, "usbHandleBulk status: %d\n", status);
    if (status > 0) {
        statusBuf[0] = status;
        DEBUGF(DBG_INFO, "About to call USB_WriteBlock with status %d\n", status);
        bool writeResult = USB_WriteBlock(statusBuf, sizeof(statusBuf));
        DEBUGF(DBG_INFO, "USB_WriteBlock returned %d\n", writeResult);
    } else if (status < 0) {
        DEBUGF(DBG_ERROR, "usbblk err\n");
        set_status(STATUS_ERROR);
#if USING_TINYUSB
        // TinyUSB handles stalling automatically when we return false from callbacks
#else
        Endpoint_StallTransaction();
#endif
        return false;
    }

    DEBUGF(DBG_INFO, "usbBulkWorker cmpl\n");
    return true;
}

/*
 * Stall all endpoints and set a flag indicating any current transfers
 * should be aborted. IO loops will see this flag in TimerWorker().
 */
void
SetAbortState()
{
#if USING_TINYUSB
    // TinyUSB handles stalling automatically when we return false from callbacks
    doDeviceReset = true;
#else
    uint8_t origEndpoint = Endpoint_GetCurrentEndpoint();

    doDeviceReset = true;
    Endpoint_SelectEndpoint(XUM_BULK_OUT_ENDPOINT);
    Endpoint_StallTransaction();
    Endpoint_SelectEndpoint(XUM_BULK_IN_ENDPOINT);
    Endpoint_StallTransaction();

    Endpoint_SelectEndpoint(origEndpoint);
#endif
}

/*
 * Periodic maintenance task. This code can be called at any point, but
 * at least needs to be called enough to reset the watchdog.
 *
 * If the board's timer has triggered, we also update the board's display
 * or any other functions it does when the timer expires.
 */
bool
TimerWorker()
{
#if USING_TINYUSB
    watchdog_update();
#else
    wdt_reset();
#endif

    // Inform the caller to quit the current transfer if we're resetting.
    if (doDeviceReset)
        return false;

    // If the timer has fired, update the board display
    if (board_timer_fired())
        board_update_display(statusValue);
    return true;
}

/*
 * Read a block from the host's OUT endpoint, handling aborts.
 */
bool
USB_ReadBlock(uint8_t *buf, uint8_t len)
{
#if USING_TINYUSB
    tu_fifo_read_n(&xum_rx_fifo, buf, len);
    if (doDeviceReset)
        return false;
    return true;
#else
    // Get the requested data from the host
    Endpoint_SelectEndpoint(XUM_BULK_OUT_ENDPOINT);
    Endpoint_Read_Stream_LE(buf, len, AbortOnReset);

    // Check if the current command is being aborted by the host
    if (doDeviceReset)
        return false;

    Endpoint_ClearOUT();
    return true;
#endif
}

/*
 * Send a block to the host's IN endpoint, handling aborts.
 */
bool
USB_WriteBlock(uint8_t *buf, uint8_t len)
{
#if USING_TINYUSB
    // TinyUSB vendor class write
    DEBUGF(DBG_INFO, "USB_WriteBlock: starting write of %d bytes\n", len);
    uint32_t count = 0;
    uint32_t loop_count = 0;
    while (count < len && !doDeviceReset) {
        uint32_t available = tud_vendor_n_write_available(0);
        if (available > 0) {
            uint32_t to_write = (len - count < available) ? (len - count) : available;
            uint32_t written = tud_vendor_n_write(0, buf + count, to_write);
            count += written;
            DEBUGF(DBG_INFO, "USB_WriteBlock: wrote %lu bytes, total %lu/%d\n", written, count, len);
        } else if (++loop_count % 1000 == 0) {
            DEBUGF(DBG_INFO, "USB_WriteBlock: waiting for TX space, loop %lu\n", loop_count);
        }
        // Allow TinyUSB to process
        tud_task();
    }

    // Flush and wait for transmission to complete
    if (count > 0) {
        DEBUGF(DBG_INFO, "USB_WriteBlock: flushing %lu bytes\n", count);
        tud_vendor_n_flush(0);

        // Wait for all data to be transmitted
        // Instead of checking write_available (which may not indicate completion),
        // wait until TinyUSB processes the data and the endpoint is ready for new transfers
        int timeout = 10000;  // Increased timeout
        uint32_t start_available = tud_vendor_n_write_available(0);

        while (timeout-- > 0 && !doDeviceReset) {
            tud_task();

            // Wait for the buffer to become fully available again
            // This indicates the previous transfer has completed
            uint32_t current_available = tud_vendor_n_write_available(0);
            if (current_available >= CFG_TUD_VENDOR_TX_BUFSIZE) {
                DEBUGF(DBG_INFO, "USB_WriteBlock: TX fully available (%lu), transmission complete\n", current_available);
                break;
            }
            if (timeout % 1000 == 0) {
                DEBUGF(DBG_INFO, "USB_WriteBlock: waiting for TX complete, available=%lu, timeout=%d\n", current_available, timeout);
            }
        }

        // Additional safety: ensure endpoint is properly reset for next transfer
        if (timeout <= 0) {
            DEBUGF(DBG_ERROR, "USB_WriteBlock: timeout waiting for TX completion\n");
        }

        // Debug: Check IN endpoint state after transmission
        uint32_t tx_available_post = tud_vendor_n_write_available(0);
        DEBUGF(DBG_INFO, "USB_WriteBlock: post-flush TX_available=%lu\n", tx_available_post);

        if (tx_available_post < CFG_TUD_VENDOR_TX_BUFSIZE) {
            DEBUGF(DBG_ERROR, "USB_WriteBlock: IN endpoint not fully available after cleanup!\n");
        }
    }

    if (doDeviceReset)
        return false;

    return (count == len);
#else
    // Get the requested data from the host
    Endpoint_SelectEndpoint(XUM_BULK_IN_ENDPOINT);
    Endpoint_Write_Stream_LE(buf, len, AbortOnReset);

    // Check if the current command is being aborted by the host
    if (doDeviceReset)
        return false;

    Endpoint_ClearIN();
    return true;
#endif
}

/*
 * Callback for the Endpoint_Read/Write_Stream functions. We abort the
 * current stream transfer if the user sent a reset message to the
 * control endpoint.
 */
uint8_t
AbortOnReset()
{
#if USING_TINYUSB
    // For TinyUSB, we just check the reset flag directly in the read/write loops
    return doDeviceReset ? 1 : 0;
#else
    return doDeviceReset ? STREAMCALLBACK_Abort : STREAMCALLBACK_Continue;
#endif
}

#if USING_TINYUSB
// TinyUSB device callbacks
void tud_mount_cb(void)
{
    DEBUGF(DBG_ALL, "tinyusb mount\n");
    // Reset USB transfer state on mount
    Reset_usbState();
}

void tud_umount_cb(void)
{
    DEBUGF(DBG_ALL, "tinyusb unmount\n");
    // Reset state when device is unmounted
    set_status(STATUS_INIT);
    // Reset USB transfer state on unmount
    Reset_usbState();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    DEBUGF(DBG_ALL, "tinyusb suspend\n");
    // Device suspended - reset USB transfer state to clean slate
    Reset_usbState();
}

void tud_resume_cb(void)
{
    DEBUGF(DBG_ALL, "tinyusb resume\n");
    // Device resumed from suspend - ensure clean USB transfer state
    Reset_usbState();
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    if (stage == CONTROL_STAGE_SETUP) {
        uint8_t replyBuf[XUM_DEVINFO_SIZE];
        memset(replyBuf, 0, sizeof(replyBuf));

        int8_t result = usbHandleControl(request->bRequest, replyBuf);

        if (result > 0) {
            return tud_control_xfer(rhport, request, replyBuf, result);
        } else if (result == 0) {
            return tud_control_status(rhport, request);
        } else {
            return false; // Stall
        }
    }

    return true;
}

// RX/incoming bulk data callback
// Note that the TinyUSB VENDOR class implementation assumes that
// bulk RX data is read here ONLY; it'll claim the FIFO for itself
// outside the callback.
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
    (void)itf; (void)buffer; (void)bufsize;
    printf("[XUMDEBUG] tud_vendor_rx_cb called: itf=%d, bufsize=%d\n", itf, bufsize);
    tu_fifo_write_n(&xum_rx_fifo, buffer, bufsize);
    // if using RX buffered is enabled, we need to flush the buffer to make room for new data
    #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    tud_vendor_read_flush();
    #endif
}

// TX callback to debug the relationship between TX completion and RX state
void tud_vendor_tx_cb(uint8_t itf, uint32_t count) {
    (void)itf;
    printf("[XUMDEBUG] tud_vendor_tx_cb called: itf=%d, count=%lu\n", itf, count);
}
#endif
