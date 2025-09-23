/*
 * USB descriptors for the xum1541
 * Copyright (c) 2009 Nate Lawson <nate@root.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include "xum1541.h"

#if !USING_TINYUSB

// LUFA descriptor

#include <avr/pgmspace.h>
#include <LUFA/Drivers/USB/USB.h>

// Convert a string to its Unicode literal
#define UNISTR(x)   L ## x

// Device descriptor
const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
    Header: {
        Size: sizeof(USB_Descriptor_Device_t),
        Type: DTYPE_Device,
    },

    USBSpecification:       VERSION_BCD(01.10),
    Class:                  0xff,
    SubClass:               0x00,
    Protocol:               0x00,

    Endpoint0Size:          XUM_ENDPOINT_0_SIZE,

    VendorID:               XUM1541_VID,
    ProductID:              XUM1541_PID,
    ReleaseNumber:          (MODEL << 8) | XUM1541_VERSION,

    ManufacturerStrIndex:   0x01,
    ProductStrIndex:        0x02,
    SerialNumStrIndex:      0x03,

    NumberOfConfigurations: 1,
};

// Entire config descriptor
typedef struct {
    USB_Descriptor_Configuration_Header_t Config;
    USB_Descriptor_Interface_t            Interface;
    USB_Descriptor_Endpoint_t             DataInEndpoint;
    USB_Descriptor_Endpoint_t             DataOutEndpoint;
} USB_Descriptor_Configuration_t;

const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
    Config: {
        Header: {
            Size: sizeof(USB_Descriptor_Configuration_Header_t),
            Type: DTYPE_Configuration,
        },

        TotalConfigurationSize: sizeof(USB_Descriptor_Configuration_t),
        TotalInterfaces:        1,
        ConfigurationNumber:    1,
        ConfigurationStrIndex:  NO_DESCRIPTOR,
        ConfigAttributes:       USB_CONFIG_ATTR_BUSPOWERED,
        MaxPowerConsumption:    USB_CONFIG_POWER_MA(100),
    },

    Interface: {
        Header: {
            Size: sizeof(USB_Descriptor_Interface_t),
            Type: DTYPE_Interface,
        },

        InterfaceNumber:   0,
        AlternateSetting:  0,
        TotalEndpoints:    2,
        Class:             0xff,
        SubClass:          0x00,
        Protocol:          0x00,
        InterfaceStrIndex: NO_DESCRIPTOR,
    },

    DataInEndpoint: {
        Header: {
            Size: sizeof(USB_Descriptor_Endpoint_t),
            Type: DTYPE_Endpoint,
        },

        EndpointAddress:  (ENDPOINT_DESCRIPTOR_DIR_IN | XUM_BULK_IN_ENDPOINT),
        Attributes:        EP_TYPE_BULK,
        EndpointSize:      XUM_ENDPOINT_BULK_SIZE,
        PollingIntervalMS: 0x00,
    },

    DataOutEndpoint: {
        Header: {
            Size: sizeof(USB_Descriptor_Endpoint_t),
            Type: DTYPE_Endpoint,
        },

        EndpointAddress:  (ENDPOINT_DESCRIPTOR_DIR_OUT | XUM_BULK_OUT_ENDPOINT),
        Attributes:        EP_TYPE_BULK,
        EndpointSize:      XUM_ENDPOINT_BULK_SIZE,
        PollingIntervalMS: 0x00,
    },
};

const USB_Descriptor_String_t PROGMEM LanguageString = {
    Header:        { Size: USB_STRING_LEN(1), Type: DTYPE_String },
    UnicodeString: { LANGUAGE_ID_ENG },
};

const USB_Descriptor_String_t PROGMEM ManufacturerString = {
    Header:        { Size: USB_STRING_LEN(28), Type: DTYPE_String },
    UnicodeString: L"Nate Lawson and OpenCBM team",
};

// Serial for attaching more than one xum1541.
static USB_Descriptor_String_t SerialNumString = {
    Header:        { Size: USB_STRING_LEN(3), Type: DTYPE_String },
    UnicodeString: L"000",
};

#define PRODSTRING(m)  L"xum1541 floppy adapter (" UNISTR(m) L")"

const USB_Descriptor_String_t PROGMEM ProductString = {
    Header: {
        Size: USB_STRING_LEN(((sizeof(PRODSTRING(MODELNAME)) / 2) - 1)),
        Type: DTYPE_String,
    },
    UnicodeString: PRODSTRING(MODELNAME),
};

uint16_t
CALLBACK_USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
  void **const DescriptorAddress, uint8_t *MemoryAddressSpace)
{
    void*    Address = NULL;
    uint16_t Size    = NO_DESCRIPTOR;
    wchar_t* ucdBuf;

    /* generally assume Flash memory access unless specified otherwise */
    *MemoryAddressSpace = MEMSPACE_FLASH;

    switch (wValue >> 8) {
    case DTYPE_Device:
        Address = (void *)&DeviceDescriptor;
        Size    = sizeof(USB_Descriptor_Device_t);
        break;
    case DTYPE_Configuration:
        Address = (void *)&ConfigurationDescriptor;
        Size    = sizeof(USB_Descriptor_Configuration_t);
        break;
    case DTYPE_String:
        switch (wValue & 0xff) {
        case 0x00:
            Address = (void *)&LanguageString;
            Size    = pgm_read_byte(&LanguageString.Header.Size);
            break;
        case 0x01:
            Address = (void *)&ManufacturerString;
            Size    = pgm_read_byte(&ManufacturerString.Header.Size);
            break;
        case 0x02:
            Address = (void *)&ProductString;
            Size    = pgm_read_byte(&ProductString.Header.Size);
            break;
        case 0x03:
            Size   = ~ eeprom_read_byte( EEPROM_SerialNumber );
            ucdBuf = SerialNumString.UnicodeString + 2;
            /*
                precondition:  SerialNumString.UnicodeString := L"000"
            */
            for( Size &= 0xff ; Size > 0 ; Size /= 10 , --ucdBuf ) {
                *ucdBuf = Size % 10 + 0x30;
            }
            Address = (void *)&SerialNumString;
            Size    = SerialNumString.Header.Size;
            *MemoryAddressSpace = MEMSPACE_RAM;
            break;
        }
        break;
    }

    *DescriptorAddress = Address;
    return Size;
}

#else

// TinyUSB descriptors
// Mostly copy/pasted from TinyUSB vendor/WebUSB example

#include "tusb.h"
#include "device/usbd.h"
#include "class/vendor/vendor_device.h"

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
#endif
