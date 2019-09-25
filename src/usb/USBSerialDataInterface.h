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

#ifndef __USB_SERIAL_DATA_INTERFACE_H
#define __USB_SERIAL_DATA_INTERFACE_H

#include "USBInterface.h"

class USBSerialDataInterface : public USBInterface {
private:
    struct usb_interface_descriptor interface_descriptor = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0x0A,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
    };

    struct usb_endpoint_descriptor endpoint_descriptor_in = {
       .bLength = USB_DT_ENDPOINT_SIZE,
       .bDescriptorType = USB_DT_ENDPOINT,
       .bEndpointAddress = USB_ENDPOINT_IN,
       .bmAttributes = USB_TRANSFER_TYPE_BULK,
       .wMaxPacketSize = 0x200,
    };

    struct usb_endpoint_descriptor endpoint_descriptor_out = {
       .bLength = USB_DT_ENDPOINT_SIZE,
       .bDescriptorType = USB_DT_ENDPOINT,
       .bEndpointAddress = USB_ENDPOINT_OUT,
       .bmAttributes = USB_TRANSFER_TYPE_BULK,
       .wMaxPacketSize = 0x200,
    };
    
public:
            USBSerialDataInterface(int index);
    virtual ~USBSerialDataInterface();

    Result initialize(const char* str);
    ssize_t read(char *ptr, size_t len, u64 timestamp);
    ssize_t write(const char *ptr, size_t len, u64 timestamp);
};

#endif /* __USB_SERIAL_DATA_INTERFACE_H */
