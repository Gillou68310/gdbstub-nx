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

#ifndef __USB_INTERFACE_H
#define __USB_INTERFACE_H

#include <assert.h>
#include "usb.h"

class USBInterface {
private:
    static USBInterface *interfaces[TOTAL_INTERFACES];
    int interface_index;

public:
    USBInterface(int index) {
        assert(index < TOTAL_INTERFACES);
        interface_index = index;
        interfaces[index] = this;
    }
    virtual Result initialize(const char* str) = 0;
    int getInterfaceIndex(void) {return interface_index;}
    static USBInterface * getInterface(int index)
    {
        if(index < TOTAL_INTERFACES)
            return interfaces[index];
        else
            return NULL;
    }
};

#endif /* __USB_INTERFACE_H */
