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

#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#include "USBSerialComInterface.h"
#include "USBSerialDataInterface.h"

class USBSerial {
private:
    static int interface_count;
    int index;
    USBSerialComInterface *com;
    USBSerialDataInterface *data;
public:
            USBSerial();
    virtual ~USBSerial();

    Result initialize(const char* str);
    ssize_t read(char *ptr, size_t len);
    ssize_t write(const char *ptr, size_t len);
    ssize_t sendEvent(const char *ptr, size_t len);
    int getIndex() {return index;}
};

#endif /* __USB_SERIAL_H */
