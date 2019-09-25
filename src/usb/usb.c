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

#include <string.h>
#include <malloc.h>

#include "usb.h"

typedef struct {
    RwLock lock;
    int initialized;
    UsbDsEndpoint *endpoint;
    u8 *buffer;
} usbEndpoint;

typedef struct {
    RwLock lock;
    int initialized;
    UsbDsInterface* interface;
    usbEndpoint ctrlEndpoint[2];
    usbEndpoint endpoint[TOTAL_ENDPOINTS];
} usbInterface;

static int g_usbDeviceInitialized = 0;
static RwLock g_usbDeviceLock;
static usbInterface g_usbInterfaces[TOTAL_INTERFACES];
static int ep_in = 1;
static int ep_out = 1;

static struct usb_ss_endpoint_companion_descriptor endpoint_companion = {
    .bLength = sizeof(struct usb_ss_endpoint_companion_descriptor),
    .bDescriptorType = USB_DT_SS_ENDPOINT_COMPANION,
    .bMaxBurst = 0x0F,
    .bmAttributes = 0x00,
    .wBytesPerInterval = 0x00,
};

Result usbInitialize(struct usb_device_descriptor *device_descriptor, const char *manufacturer, const char *product, const char *serialNumber)
{
    Result rc = 0;
    rwlockWriteLock(&g_usbDeviceLock);
    
    if (g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_AlreadyInitialized);
    else
    {
        rc = usbDsInitialize();
        memset(&g_usbInterfaces, 0, sizeof(g_usbInterfaces));

        if (R_SUCCEEDED(rc)) {
            u8 iManufacturer, iProduct, iSerialNumber;
            static const u16 supported_langs[1] = {0x0409};
            // Send language descriptor
            rc = usbDsAddUsbLanguageStringDescriptor(NULL, supported_langs, sizeof(supported_langs)/sizeof(u16));
            // Send manufacturer
            if (R_SUCCEEDED(rc)) rc = usbDsAddUsbStringDescriptor(&iManufacturer, manufacturer);
            // Send product
            if (R_SUCCEEDED(rc)) rc = usbDsAddUsbStringDescriptor(&iProduct, product);
            // Send serial number
            if (R_SUCCEEDED(rc)) rc = usbDsAddUsbStringDescriptor(&iSerialNumber, serialNumber);

            // Send device descriptors
            device_descriptor->iManufacturer = iManufacturer;
            device_descriptor->iProduct = iProduct;
            device_descriptor->iSerialNumber = iSerialNumber;

            // Full Speed is USB 1.1
            if (R_SUCCEEDED(rc)) rc = usbDsSetUsbDeviceDescriptor(UsbDeviceSpeed_Full, device_descriptor);

            // High Speed is USB 2.0
            device_descriptor->bcdUSB = 0x0200;
            if (R_SUCCEEDED(rc)) rc = usbDsSetUsbDeviceDescriptor(UsbDeviceSpeed_High, device_descriptor);

            // Super Speed is USB 3.0
            device_descriptor->bcdUSB = 0x0300;
            // Upgrade packet size to 512
            device_descriptor->bMaxPacketSize0 = 0x09;
            if (R_SUCCEEDED(rc)) rc = usbDsSetUsbDeviceDescriptor(UsbDeviceSpeed_Super, device_descriptor);

            // Define Binary Object Store
            u8 bos[0x16] = {
                0x05, // .bLength
                USB_DT_BOS, // .bDescriptorType
                0x16, 0x00, // .wTotalLength
                0x02, // .bNumDeviceCaps

                // USB 2.0
                0x07, // .bLength
                USB_DT_DEVICE_CAPABILITY, // .bDescriptorType
                0x02, // .bDevCapabilityType
                0x02, 0x00, 0x00, 0x00, // dev_capability_data

                // USB 3.0
                0x0A, // .bLength
                USB_DT_DEVICE_CAPABILITY, // .bDescriptorType
                0x03, // .bDevCapabilityType
                0x00, 0x0E, 0x00, 0x03, 0x00, 0x00, 0x00
            };
            if (R_SUCCEEDED(rc)) rc = usbDsSetBinaryObjectStore(bos, sizeof(bos));
        }
        if(R_SUCCEEDED(rc)) g_usbDeviceInitialized = 1;
    }
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbAddStringDescriptor(u8* out_index, const char* string)
{
    Result rc;
    rwlockWriteLock(&g_usbDeviceLock);
    if (!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else
        rc = usbDsAddUsbStringDescriptor(out_index, string);
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbEnable(void)
{
    Result rc;
    rwlockWriteLock(&g_usbDeviceLock);
    if (!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else
        rc = usbDsEnable();
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbInterfaceInit(u32 intf_ind, struct usb_interface_descriptor *interface_descriptor, struct usb_interface_association_descriptor *interface_association_descriptor)
{
    Result rc = 0;
    rwlockWriteLock(&g_usbDeviceLock);

    if(!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else if (intf_ind >= TOTAL_INTERFACES)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];
        rwlockWriteLock(&interface->lock);
        rwlockWriteLock(&interface->ctrlEndpoint[0].lock);
        rwlockWriteLock(&interface->ctrlEndpoint[1].lock);

        // The buffer for CtrlPostBufferAsync commands must be 0x1000-byte aligned.
        interface->ctrlEndpoint[0].buffer = (u8*)memalign(0x1000, 0x1000);
        interface->ctrlEndpoint[1].buffer = (u8*)memalign(0x1000, 0x1000);

        if (interface->ctrlEndpoint[0].buffer != NULL && interface->ctrlEndpoint[1].buffer != NULL)
        {
            memset(interface->ctrlEndpoint[0].buffer, 0, 0x1000);
            memset(interface->ctrlEndpoint[1].buffer, 0, 0x1000);
        }
        else
            rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);

        if (R_SUCCEEDED(rc))
        {
            rc = usbDsRegisterInterface(&interface->interface);
            interface_descriptor->bInterfaceNumber = interface->interface->interface_index;
        }

        // Full Speed Config
        if (interface_association_descriptor != NULL && R_SUCCEEDED(rc))
            rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Full, interface_association_descriptor, interface_association_descriptor->bLength);
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Full, interface_descriptor, USB_DT_INTERFACE_SIZE);

        // High Speed Config
        if (interface_association_descriptor != NULL && R_SUCCEEDED(rc))
            rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_High, interface_association_descriptor, interface_association_descriptor->bLength);
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_High, interface_descriptor, USB_DT_INTERFACE_SIZE);

        // Super Speed Config
        if (interface_association_descriptor != NULL && R_SUCCEEDED(rc))
            rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Super, interface_association_descriptor, interface_association_descriptor->bLength);
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Super, interface_descriptor, USB_DT_INTERFACE_SIZE);

        if (R_SUCCEEDED(rc))
        {
            interface->initialized = 1;
            interface->ctrlEndpoint[0].initialized = 1;
            interface->ctrlEndpoint[1].initialized = 1;
        }

        rwlockWriteUnlock(&interface->ctrlEndpoint[1].lock);
        rwlockWriteUnlock(&interface->ctrlEndpoint[0].lock);
        rwlockWriteUnlock(&interface->lock);
    }
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbInterfaceAddData(u32 intf_ind, char* data)
{
    Result rc = 0;
    rwlockWriteLock(&g_usbDeviceLock);

    if(!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else if (intf_ind >= TOTAL_INTERFACES)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];
        rwlockWriteLock(&interface->lock);

        // Full Speed Config
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Full, data, data[0]);

        // High Speed Config
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_High, data, data[0]);

        // Super Speed Config
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Super, data, data[0]);

        rwlockWriteUnlock(&interface->lock);
    }
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbEnableInterface(u32 intf_ind)
{
    Result rc;
    rwlockWriteLock(&g_usbDeviceLock);
    if (!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else if (intf_ind >= TOTAL_INTERFACES)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];
        rwlockWriteLock(&interface->lock);
        if (!interface->initialized)
            rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
        else
            rc = usbDsInterface_EnableInterface(interface->interface);
        rwlockWriteUnlock(&interface->lock);
    }
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

Result usbAddEndpoint(u32 intf_ind, u32 ep_ind, struct usb_endpoint_descriptor *endpoint_descriptor)
{
    Result rc = 0;
    rwlockWriteLock(&g_usbDeviceLock);

    if(!g_usbDeviceInitialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else if (intf_ind >= TOTAL_INTERFACES)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else if (ep_ind >= TOTAL_ENDPOINTS)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];
        rwlockWriteLock(&interface->lock);
        rwlockWriteLock(&interface->endpoint[ep_ind].lock);

        //The buffer for PostBufferAsync commands must be 0x1000-byte aligned.
        interface->endpoint[ep_ind].buffer = (u8*)memalign(0x1000, 0x1000);
        if (interface->endpoint[ep_ind].buffer != NULL)
            memset(interface->endpoint[ep_ind].buffer, 0, 0x1000);
        else
            rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);

        if((endpoint_descriptor->bEndpointAddress & USB_ENDPOINT_IN) != 0)
            endpoint_descriptor->bEndpointAddress |= ep_in++;
        else
            endpoint_descriptor->bEndpointAddress |= ep_out++;

        // Full Speed Config
        if(endpoint_descriptor->bmAttributes == USB_TRANSFER_TYPE_BULK)
            endpoint_descriptor->wMaxPacketSize = 0x40;
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Full, endpoint_descriptor, USB_DT_ENDPOINT_SIZE);

        // High Speed Config
        if(endpoint_descriptor->bmAttributes == USB_TRANSFER_TYPE_BULK)
            endpoint_descriptor->wMaxPacketSize = 0x200;
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_High, endpoint_descriptor, USB_DT_ENDPOINT_SIZE);

        // Super Speed Config
        if(endpoint_descriptor->bmAttributes == USB_TRANSFER_TYPE_BULK)
            endpoint_descriptor->wMaxPacketSize = 0x400;
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Super, endpoint_descriptor, USB_DT_ENDPOINT_SIZE);
        if (R_SUCCEEDED(rc)) rc = usbDsInterface_AppendConfigurationData(interface->interface, UsbDeviceSpeed_Super, &endpoint_companion, USB_DT_SS_ENDPOINT_COMPANION_SIZE);

        if (R_SUCCEEDED(rc)) rc = usbDsInterface_RegisterEndpoint(interface->interface, &interface->endpoint[ep_ind].endpoint, endpoint_descriptor->bEndpointAddress);

        if (R_SUCCEEDED(rc)) interface->endpoint[ep_ind].initialized = 1;

        rwlockWriteUnlock(&interface->endpoint[ep_ind].lock);
        rwlockWriteUnlock(&interface->lock);
    }
    
    rwlockWriteUnlock(&g_usbDeviceLock);
    return rc;
}

static void _usbInterfaceFree(usbInterface *interface)
{
    rwlockWriteLock(&interface->lock);
    if (interface->initialized)
    {
        interface->initialized = 0;
        interface->interface = NULL;

        rwlockWriteLock(&interface->ctrlEndpoint[0].lock);
        if(interface->ctrlEndpoint[0].initialized)
        {
            interface->ctrlEndpoint[0].initialized = 0;
            interface->ctrlEndpoint[0].endpoint = NULL;
            free(interface->ctrlEndpoint[0].buffer);
            interface->ctrlEndpoint[0].buffer = NULL;
        }
        rwlockWriteUnlock(&interface->ctrlEndpoint[0].lock);

        rwlockWriteLock(&interface->ctrlEndpoint[1].lock);
        if(interface->ctrlEndpoint[1].initialized)
        {
            interface->ctrlEndpoint[1].initialized = 0;
            interface->ctrlEndpoint[1].endpoint = NULL;
            free(interface->ctrlEndpoint[1].buffer);
            interface->ctrlEndpoint[1].buffer = NULL;
        }
        rwlockWriteUnlock(&interface->ctrlEndpoint[1].lock);

        for (u32 i = 0; i < TOTAL_ENDPOINTS; i++)
        {
            rwlockWriteLock(&interface->endpoint[i].lock);
            if(interface->endpoint[i].initialized)
            {
                interface->endpoint[i].initialized = 0;
                interface->endpoint[i].endpoint = NULL;
                free(interface->endpoint[i].buffer);
                interface->endpoint[i].buffer = NULL;
            }
            rwlockWriteUnlock(&interface->endpoint[i].lock);
        }
    }
    rwlockWriteUnlock(&interface->lock);
}

void usbExit(void)
{
    rwlockWriteLock(&g_usbDeviceLock);
    usbDsExit();
    g_usbDeviceInitialized = 0;
    rwlockWriteUnlock(&g_usbDeviceLock);

    for (u32 i = 0; i < TOTAL_INTERFACES; i++)
        _usbInterfaceFree(&g_usbInterfaces[i]);
}

static Result _usbTransfer(usbInterface *interface, usbEndpoint *ep, UsbDirection dir, const void* buffer, size_t size, u64 timeout, size_t *transferredSize)
{
    Result rc=0;
    u32 urbId=0;
    u32 chunksize=0;
    u8 transfer_type=0;
    u8 *bufptr = (u8*)buffer;
    u8 *transfer_buffer = NULL;
    u32 tmp_transferredSize = 0;
    size_t total_transferredSize=0;
    UsbDsReportData reportdata;

    //Makes sure endpoints are ready for data-transfer / wait for init if needed.
    rc = usbDsWaitReady(U64_MAX);
    if (R_FAILED(rc)) return rc;

    do
    {
        if(bufptr == NULL || size == 0) // Zero length packet
        {
            transfer_buffer = ep->buffer;
            chunksize = 0;
            transfer_type = 1;
        }
        else if(((u64)bufptr) & 0xfff) // When bufptr isn't page-aligned copy the data into ep->buffer and transfer that, otherwise use the bufptr directly.
        {
            transfer_buffer = ep->buffer;
            memset(ep->buffer, 0, 0x1000);

            chunksize = 0x1000;
            chunksize-= ((u64)bufptr) & 0xfff; // After this transfer, bufptr will be page-aligned(if size is large enough for another transfer).
            if (size<chunksize) chunksize = size;

            if(dir == UsbDirection_Write)
                memcpy(ep->buffer, bufptr, chunksize);

            transfer_type = 0;
        }
        else
        {
            transfer_buffer = bufptr;
            chunksize = size;
            transfer_type = 1;
        }

        //Start transfer.
        if(ep->endpoint == NULL) // Control endpoint are not registered
        {
            if(dir == UsbDirection_Write)
            {
                rc = usbDsInterface_CtrlInPostBufferAsync(interface->interface, transfer_buffer, chunksize, &urbId);
                if(R_FAILED(rc))return rc;

                //Wait for the transfer to finish.
                eventWait(&interface->interface->CtrlInCompletionEvent, U64_MAX);
                eventClear(&interface->interface->CtrlInCompletionEvent);

                rc = usbDsInterface_GetCtrlInReportData(interface->interface, &reportdata);
                if (R_FAILED(rc)) return rc;
            }
            else
            {
                rc = usbDsInterface_CtrlOutPostBufferAsync(interface->interface, transfer_buffer, chunksize, &urbId);
                if(R_FAILED(rc))return rc;

                //Wait for the transfer to finish.
                eventWait(&interface->interface->CtrlOutCompletionEvent, U64_MAX);
                eventClear(&interface->interface->CtrlOutCompletionEvent);

                rc = usbDsInterface_GetCtrlOutReportData(interface->interface, &reportdata);
                if (R_FAILED(rc)) return rc;
            }
        }
        else
        {
            rc = usbDsEndpoint_PostBufferAsync(ep->endpoint, transfer_buffer, chunksize, &urbId);
            if(R_FAILED(rc))return rc;

            //Wait for the transfer to finish.
            rc = eventWait(&ep->endpoint->CompletionEvent, timeout);

            if (R_FAILED(rc))
            {
                usbDsEndpoint_Cancel(ep->endpoint);
                eventWait(&ep->endpoint->CompletionEvent, U64_MAX);
                eventClear(&ep->endpoint->CompletionEvent);
                return rc;
            }
            eventClear(&ep->endpoint->CompletionEvent);

            rc = usbDsEndpoint_GetReportData(ep->endpoint, &reportdata);
            if (R_FAILED(rc)) return rc;
        }

        rc = usbDsParseReportData(&reportdata, urbId, NULL, &tmp_transferredSize);
        if (R_FAILED(rc)) return rc;

        if (tmp_transferredSize > chunksize) tmp_transferredSize = chunksize;

        total_transferredSize+= (size_t)tmp_transferredSize;

        if ((transfer_type==0) && (dir == UsbDirection_Read))
            memcpy(bufptr, transfer_buffer, tmp_transferredSize);

        bufptr+= tmp_transferredSize;
        size-= tmp_transferredSize;

        if (tmp_transferredSize < chunksize) break;
    }
    while(size);

    if (transferredSize) *transferredSize = total_transferredSize;

    return rc;
}

size_t usbTransfer(u32 intf_ind, u32 ep_ind, UsbDirection dir, void* buffer, size_t size, u64 timeout)
{
    size_t transferredSize=-1;
    u32 state=0;
    Result rc;
    int initialized;
    usbEndpoint *ep;

    if(intf_ind < TOTAL_INTERFACES && (ep_ind < TOTAL_ENDPOINTS || ep_ind == 0xFFFFFFFF))
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];
        if(ep_ind == 0xFFFFFFFF) // Control endpoint
            ep = &interface->ctrlEndpoint[dir];
        else
            ep = &interface->endpoint[ep_ind];

        rwlockWriteLock(&g_usbDeviceLock);
        initialized = g_usbDeviceInitialized;
        rwlockWriteUnlock(&g_usbDeviceLock);

        rwlockReadLock(&interface->lock);
        initialized &= interface->initialized;
        rwlockReadUnlock(&interface->lock);

        rwlockReadLock(&ep->lock);
        initialized &= ep->initialized;
        rwlockReadUnlock(&ep->lock);

        if(initialized)
        {
            rwlockWriteLock(&ep->lock);
            rc = _usbTransfer(interface, ep, dir, buffer, size, timeout, &transferredSize);
            rwlockWriteUnlock(&ep->lock);
            if (R_FAILED(rc)) {
                rc = usbDsGetState(&state);
                //If state changed during transfer, try again. usbDsWaitReady() will be called from this.
                if (R_SUCCEEDED(rc)) {
                    if (state!=5) {
                        rwlockWriteLock(&ep->lock);
                        rc = _usbTransfer(interface, ep, dir, buffer, size, timeout, &transferredSize);
                        rwlockWriteUnlock(&ep->lock);
                    }
                }
            }
        }
    }
    return transferredSize;
}

Result usbGetSetupPacket(u32 intf_ind, void* buffer, size_t size, u64 timeout)
{
    Result rc;

    rwlockWriteLock(&g_usbDeviceLock);
    int initialized = g_usbDeviceInitialized;
    rwlockWriteUnlock(&g_usbDeviceLock);

    if(!initialized)
        rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    else if (intf_ind >= TOTAL_INTERFACES)
        rc = MAKERESULT(Module_Libnx, LibnxError_OutOfMemory);
    else
    {
        usbInterface *interface = &g_usbInterfaces[intf_ind];

        rwlockReadLock(&interface->lock);
        initialized = interface->initialized;
        rwlockReadUnlock(&interface->lock);

        if(initialized)
        {
            rc = eventWait(&interface->interface->SetupEvent, timeout);
            if (R_SUCCEEDED(rc)) {
                eventClear(&interface->interface->SetupEvent);
                rc = usbDsInterface_GetSetupPacket(interface->interface, buffer, size);
            }
        }
        else
            rc = MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    }
    return rc;
}
