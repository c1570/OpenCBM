/*
 * TinyUSB descriptors for XUM1541 RP2040 port
 * Copyright (c) 2024 XUM1541 RP2040 Port
 * Based on descriptor.c
 * Copyright (c) 2009 Nate Lawson <nate@root.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include "tusb.h"
#include "device/usbd.h"
#include "class/vendor/vendor_device.h"
#include "xum1541.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,  // USB 2.0

    // Use vendor-specific class
    .bDeviceClass       = 0xFF,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,

    .bMaxPacketSize0    = XUM_ENDPOINT_0_SIZE,

    .idVendor           = XUM1541_VID,
    .idProduct          = XUM1541_PID,
    .bcdDevice          = (MODEL << 8) | XUM1541_VERSION,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
    ITF_NUM_VENDOR = 0,
    //USBDEBUG ITF_NUM_CDC,
    //USBDEBUG ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN /*USBDEBUG + TUD_CDC_DESC_LEN */)

#define EPNUM_VENDOR_OUT    XUM_BULK_OUT_ENDPOINT
#define EPNUM_VENDOR_IN     (XUM_BULK_IN_ENDPOINT | 0x80)


#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82

uint8_t const desc_configuration[] =
{
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

    // Interface number, string index, EP Out & IN address, EP size
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 0, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, XUM_ENDPOINT_BULK_SIZE),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    //USBDEBUG TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index; // for multiple configurations
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String descriptor index
enum
{
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

// Array of pointer to string descriptors
char const* string_desc_arr [] =
{
    (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
    "Nate Lawson and OpenCBM team",                     // 1: Manufacturer
    "xum1541 floppy adapter (USBKEY)",                  // 2: Product, TODO "USBKEY" for now to enable "xum1541cfg -t USBKEY bootloader"
    "000",                                              // 3: Serials, should use chip ID
    //USBDEBUG "debug serial",
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if ( index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str = string_desc_arr[index];

        // Handle serial number generation similar to original
        if (index == STRID_SERIAL) {
            // For now, use a simple serial. In production, this should use
            // the RP2040's unique ID from pico_unique_id library
            str = "001";
        }

        // Cap at max chars
        chr_count = (uint8_t) strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

        // Convert ASCII string into UTF-16
        for(uint8_t i=0; i<chr_count; i++)
        {
            _desc_str[1+i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

    return _desc_str;
}

//--------------------------------------------------------------------+
// Vendor callbacks
//--------------------------------------------------------------------+

// Forward declarations
int8_t usbHandleControl(uint8_t cmd, uint8_t *replyBuf);
void dcd_reset_endpoint_pid(uint8_t ep_addr);

// Invoked when received VENDOR control request
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    // Handle vendor-specific control requests
    if (stage == CONTROL_STAGE_SETUP) {
        uint8_t replyBuf[XUM_DEVINFO_SIZE];
        memset(replyBuf, 0, sizeof(replyBuf));

        // Check for SET_INTERFACE (bRequest=11) and reset bulk PIDs
        if (request->bRequest == 11) {
            printf("[XUM1541-SET_INTERFACE] Received SET_INTERFACE (bRequest=11), resetting bulk PIDs\n");
            dcd_reset_endpoint_pid(0x83);  // Reset IN endpoint PID
            dcd_reset_endpoint_pid(0x04);  // Reset OUT endpoint PID
            return tud_control_status(rhport, request);
        }

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

extern tu_fifo_t xum_rx_fifo;

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
