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

#include "USBSerialComInterface.h"

void dump(void* buffer, int size);

 struct usb_ctrlrequest {
   uint8_t bRequestType;
   uint8_t bRequest;
   uint16_t wValue;
   uint16_t wIndex;
   uint16_t wLength;
 } PACKED;

static const char parity_char[] = {'N', 'O', 'E', 'M', 'S'};
static const char *stop_bits_str[] = {"1","1.5","2"};

#define EP0 0xFFFFFFFF
#define EP_INT 0

#define GET_LINE_CODING           0x21
#define SET_LINE_CODING           0x20
#define SET_CONTROL_LINE_STATE    0x22
#define SEND_BREAK                0x23

USBSerialComInterface::USBSerialComInterface(int index, USBSerialDataInterface *cdc_data_interface) : USBInterface(index)
{
    interface_association_descriptor.bFirstInterface = getInterfaceIndex();
    cdc_com_call_mgmt.bDataInterface = cdc_data_interface->getInterfaceIndex();
    cdc_com_union.bControlInterface = getInterfaceIndex();
    cdc_com_union.bSubordinateInterface0 = cdc_data_interface->getInterfaceIndex();
}

USBSerialComInterface::~USBSerialComInterface() {
}

Result USBSerialComInterface::initialize(const char* str)
{
    Result rc = usbAddStringDescriptor(&interface_association_descriptor.iFunction, str);
    if (R_SUCCEEDED(rc)) rc = usbAddStringDescriptor(&interface_descriptor.iInterface, "CDC Abstract Control Model (ACM)");
    if (R_SUCCEEDED(rc)) rc = usbInterfaceInit(getInterfaceIndex(), &interface_descriptor, &interface_association_descriptor);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(getInterfaceIndex(), (char*)&cdc_com_header);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(getInterfaceIndex(), (char*)&cdc_com_call_mgmt);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(getInterfaceIndex(), (char*)&cdc_com_acm);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(getInterfaceIndex(), (char*)&cdc_com_union);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(getInterfaceIndex(), EP_INT, &endpoint_descriptor_interrupt);
    if (R_SUCCEEDED(rc)) rc = usbEnableInterface(getInterfaceIndex());
    return rc;
}

ssize_t USBSerialComInterface::sendEvent(const char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(getInterfaceIndex(), EP_INT, UsbDirection_Write, (void*)ptr, len, timestamp);
}

void USBSerialComInterface::handle_setup_packet(bool *quit)
{
    int size;
    Result rc;
    struct usb_ctrlrequest ctrl;
    
    while(!*quit)
    {
        rc = usbGetSetupPacket(0, &ctrl, sizeof(struct usb_ctrlrequest), 1000000000LL);
        if (R_SUCCEEDED(rc)) {

            USBSerialComInterface* com_interface = dynamic_cast<USBSerialComInterface*>(getInterface(ctrl.wIndex));
            if(com_interface == NULL)
                continue;

            int interface = com_interface->getInterfaceIndex();
            struct line_coding *coding = com_interface->getLineCoding();

            switch (ctrl.bRequest) {
                case GET_LINE_CODING:
                    size = usbTransfer(interface, EP0, UsbDirection_Write, coding, sizeof(struct line_coding), U64_MAX);
                    if(size == sizeof(struct line_coding))
                        size = usbTransfer(interface, EP0, UsbDirection_Read, NULL, 0, U64_MAX);
                    /*else
                        usbDsInterface_StallCtrl();*/
                    break;
                case SET_LINE_CODING:
                    size = usbTransfer(interface, EP0, UsbDirection_Read, coding, sizeof(struct line_coding), U64_MAX);
                    if (size == sizeof(struct line_coding))
                        size = usbTransfer(interface, EP0, UsbDirection_Write, NULL, 0, U64_MAX);
                    /*else
                        usbDsInterface_StallCtrl();*/
                    break;
                case SET_CONTROL_LINE_STATE:
                    if(ctrl.wValue & 0x02) com_interface->setCarrier(true);
                    else com_interface->setCarrier(false);
                    if(ctrl.wValue & 0x01) com_interface->setDTE(true);
                    else com_interface->setDTE(false);
                    size = usbTransfer(interface, EP0, UsbDirection_Write, NULL, 0, U64_MAX);
                    break;    
                default:
                    /*usbDsInterface_StallCtrl();*/
                    break;
            }
        }
    }
}