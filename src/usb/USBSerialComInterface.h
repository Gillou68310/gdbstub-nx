/*
 * Copyright (c) 2019 Gillou68310
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __USB_SERIAL_COM_INTERFACE_H
#define __USB_SERIAL_COM_INTERFACE_H

#include "USBInterface.h"
#include "USBSerialDataInterface.h"

struct usb_cdc_header {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t bcdCDC;
} PACKED;

struct usb_cdc_call_mgmt {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bmCapabilities;
    uint8_t  bDataInterface;
};

struct usb_cdc_acm {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bmCapabilities;
};

struct usb_cdc_union {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bControlInterface;
    uint8_t  bSubordinateInterface0;
};

struct line_coding {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} PACKED;

class USBSerialComInterface : public USBInterface {
private:
    struct usb_interface_association_descriptor interface_association_descriptor = {
        .bLength = 0x08,
        .bDescriptorType = 0x0B,
        .bFirstInterface = 0x00,
        .bInterfaceCount = 0x02,
        .bFunctionClass = 0x02,
        .bFunctionSubClass = 0x02,
        .bFunctionProtocol = 0x01,
        .iFunction = 0x08,
    };
    struct usb_interface_descriptor interface_descriptor = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bNumEndpoints = 0x01,
        .bInterfaceClass = 0x02,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x01,
    };
    struct usb_cdc_header cdc_com_header = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x00,
        .bcdCDC = 0x110,
    };
    struct usb_cdc_call_mgmt cdc_com_call_mgmt = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x01,
        .bmCapabilities = 0x03,
        .bDataInterface = 0x01,
    };
    struct usb_cdc_acm cdc_com_acm = {
        .bFunctionLength = 0x04,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x02,
        .bmCapabilities = 0x02,
    };
    struct usb_cdc_union cdc_com_union = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x06,
        .bControlInterface = 0x00,
        .bSubordinateInterface0 = 0x01,
    };
    struct usb_endpoint_descriptor endpoint_descriptor_interrupt = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_ENDPOINT_IN,
        .bmAttributes = USB_TRANSFER_TYPE_INTERRUPT,
        .wMaxPacketSize = 0xA,
        .bInterval = 9,
    };
    struct line_coding coding = {
        .dwDTERate = 9600,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 0x08,
    };

    bool DTE = false;
    bool Carrier = false;
    
public:
            USBSerialComInterface(int index, USBSerialDataInterface *cdc_data_interface);
    virtual ~USBSerialComInterface();
    
    Result initialize(const char* str);
    ssize_t sendEvent(const char *ptr, size_t len, u64 timestamp);
    struct line_coding * getLineCoding() {return &coding;}
    bool getDTE() {return DTE;}
    void setDTE(bool b) {DTE = b;}
    bool getCarrier() {return Carrier;}
    void setCarrier(bool b) {Carrier = b;}
    static void handle_setup_packet(bool *quit);
};

#endif /* __USB_SERIAL_COM_INTERFACE_H */
