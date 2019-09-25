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

#include <stdio.h>
#include <string.h>
#include <sys/iosupport.h>

#include "nxlink.h"

static USBSerial *_serial;

static ssize_t write_stdout(struct _reent *r,void *fd,const char *ptr, size_t len)
{
    (void)_serial->write(ptr, len);
    return len; // Hack for std::cout
}

static const devoptab_t dotab_stdout = {
	"usb",
	0,
	NULL,
	NULL,
	write_stdout,
	NULL,
	NULL,
	NULL
};

void nxlinkStdio(USBSerial *serial)
{
    if(serial != NULL)
    {
        _serial = serial;
        devoptab_list[STD_OUT] = &dotab_stdout;
        devoptab_list[STD_ERR] = &dotab_stdout;
        setvbuf(stdout, NULL , _IONBF, 0);
        setvbuf(stderr, NULL , _IONBF, 0);
    }
}
