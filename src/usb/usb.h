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

#ifndef __USB_H
#define __USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <switch.h>

#define TOTAL_INTERFACES 4
#define TOTAL_ENDPOINTS 4

typedef enum {
    UsbDirection_Read  = 0,
    UsbDirection_Write = 1,
} UsbDirection;

struct usb_interface_association_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bFirstInterface;
    uint8_t  bInterfaceCount;
    uint8_t  bFunctionClass;
    uint8_t  bFunctionSubClass;
    uint8_t  bFunctionProtocol;
    uint8_t  iFunction;
};

Result usbInitialize(struct usb_device_descriptor *device_descriptor, const char *manufacturer, const char *product, const char *serialNumber);
Result usbEnable(void);
void usbExit(void);

Result usbAddStringDescriptor(u8* out_index, const char* string);
Result usbInterfaceInit(u32 intf_ind, struct usb_interface_descriptor *interface_descriptor, struct usb_interface_association_descriptor *interface_association_descriptor);
Result usbInterfaceAddData(u32 intf_ind, char* data);
Result usbAddEndpoint(u32 intf_ind, u32 ep_ind, struct usb_endpoint_descriptor *endpoint_descriptor);
Result usbEnableInterface(u32 intf_ind);

Result usbGetSetupPacket(u32 intf_ind, void* buffer, size_t size, u64 timeout);
size_t usbTransfer(u32 intf_ind, u32 ep_ind, UsbDirection dir, void* buffer, size_t size, u64 timeout);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* __USB_H */
