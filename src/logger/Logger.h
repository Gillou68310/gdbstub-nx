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

#pragma once

#include <iostream>
#include <sstream>
#include <string>

#define VERBOSE 0
#define INFO 1
#define WARNING 2
#define ERROR 3
#define FATAL 4

#define LOG(severity) Log::Logger(severity)
#define VLOG_IS_ON(verboselevel) (verboselevel <= Log::verbose_level)
#define VLOG(verboselevel) if(VLOG_IS_ON(verboselevel)) Log::Logger(VERBOSE)

namespace Log {
	extern int verbose_level;

	struct DateTime {
	    DateTime(void);
	    void make(void);

	    int year;    ///< year    [0,30827]
	    int month;   ///< month   [1,12]
	    int day;     ///< day     [1,31]
	    int hour;    ///< hour    [0,23]
	    int minute;  ///< minute  [0,59]
	    int second;  ///< second  [0,59]
	    int ms;      ///< millisecond
	    int us;      ///< microsecond
	};

	class Logger {
	public:
		Logger(int severity);
		~Logger(void);

		template <typename T>
		Logger& operator<< (const T& aValue) {
			mStream << aValue;
			return (*this);
		}

	private:
		static unsigned int toEscapeCode(int severity);

	private:
		std::ostringstream mStream;
		int mSeverity;
		DateTime mTime;
	};
} // namespace Log

