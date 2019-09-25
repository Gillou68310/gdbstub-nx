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
#include <time.h>
#include <sys/time.h>
#include "Logger.h"

namespace Log
{
	int verbose_level = 0;

	DateTime::DateTime(void) :
	    year(0),
	    month(0),
	    day(0),
	    hour(0),
	    minute(0),
	    second(0),
	    ms(0),
	    us(0) {
	}

	void DateTime::make(void) {
	    struct timeval now;
	    gettimeofday(&now, nullptr);
	    struct tm* timeinfo = localtime(&now.tv_sec);

	    year    = timeinfo->tm_year + 1900;
	    month   = timeinfo->tm_mon + 1;
	    day     = timeinfo->tm_mday;
	    hour    = timeinfo->tm_hour;
	    minute  = timeinfo->tm_min;
	    second  = timeinfo->tm_sec;
	    ms      = static_cast<int>(now.tv_usec / 1000);
	    us      = static_cast<int>(now.tv_usec % 1000);
	}

	Logger::Logger(int severity) {
		mSeverity = severity;
		mTime.make();
	}
	Logger::~Logger(void) {
    	fprintf(stdout, "\x1B[%02um%.2u:%.2u:%.2u.%.3u %s\x1b[37m\n",
            toEscapeCode(mSeverity),
            mTime.hour, mTime.minute, mTime.second, mTime.ms,
            mStream.str().c_str());
	}

	unsigned int Logger::toEscapeCode(int severity) {
	    switch (severity) {
	        case VERBOSE:
				return 37;	// White
	        case INFO:
				return 32;	// Green
	        case WARNING:
				return 93;	// Yellow
	        case ERROR:
				return 31;	// Red
	        case FATAL:
				return 35;	// Purple
	        default:
				return 37;	// White
	    }
	}
}