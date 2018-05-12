/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi <first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef RDK2_TIME_TIME
#define RDK2_TIME_TIME

#include <string>
#include <iostream>
#include <sys/time.h>

namespace RDK2 { namespace Time {

using namespace std;

struct Timestamp : public timeval {
	Timestamp() { setToNow(); }
	Timestamp(double seconds) { setSeconds(seconds); }
	inline void setToNow() { gettimeofday(this, 0); }
	inline void setSeconds(double sec) { tv_sec = (time_t) sec;
		tv_usec = (suseconds_t)((sec-tv_sec)*1000000); }
	inline void setMs(double sec) { setSeconds(sec*0.001); }
	inline double getSeconds() const { return (double) tv_sec + 0.000001 * tv_usec; }
	inline double getMs() const { return (double) tv_sec * 1000 + tv_usec * 0.001; }
	inline operator double() const { return getSeconds(); }
	inline Timestamp operator+(const Timestamp& ts) const {
		Timestamp t;
		t.tv_sec = this->tv_sec + ts.tv_sec;
		t.tv_usec = this->tv_usec + ts.tv_usec;
		return t;
	}
	inline Timestamp operator-(const Timestamp& ts) const {
		Timestamp t; bool r = false;
		if (ts.tv_usec > this->tv_usec) r = true;
		t.tv_sec = this->tv_sec - ts.tv_sec - (r ? 1 : 0);
		t.tv_usec = this->tv_usec - ts.tv_usec + (r ? 1000000 : 0);
		return t;
	}
	inline string getStringRepresentation() const {
        struct timeval tv;
        gettimeofday(&tv, 0);

        struct tm myTm;
        localtime_r(&tv.tv_sec, &myTm);

        char buffer[30];
        strftime(buffer, 30, "%Y-%m-%d-%H.%M.%S", &myTm);
		return string(buffer);
	}
	inline unsigned long getMsFromMidnight() const {
		return (tv_sec % 86400) * 1000 + tv_usec / 1000;
	}
	inline void setMsFromMidnight(unsigned long ms) {
		tv_sec = ms / 1000;
		tv_usec = (ms % 1000) * 1000;
	}
	inline bool operator<(const Timestamp& ts) const {
		return (tv_sec < ts.tv_sec || (tv_sec == ts.tv_sec && tv_usec < ts.tv_usec));
	}
	inline bool operator>(const Timestamp& ts) const {
		return (tv_sec > ts.tv_sec || (tv_sec == ts.tv_sec && tv_usec > ts.tv_usec));
	}
};

inline ostream& operator<<(ostream& oss, const Timestamp& ts)
{
	return (oss << ts.getSeconds());
}

inline istream& operator>>(istream& iss, Timestamp& ts)
{
	double ddd;
	iss >> ddd;
	ts.setSeconds(ddd);
	return iss;
}

/// This measures the real time
class TimerR {
public:
	TimerR() { start(); }
	inline void start() { ts.setToNow(); }
	inline double getMs() { Timestamp now; return now.getMs() - ts.getMs(); }
	inline double getSeconds() { return getMs() / 1000.; }

private:
	Timestamp ts;
};

/// This measures the thread time
class TimerT {
public:
	TimerT() { start(); }
	inline void start() { tc = clock(); }
	inline double getMs() { return ((double) (clock() - tc)) / (double) CLOCKS_PER_SEC * 1000.; }
	inline double getSeconds() { return getMs() / 1000.; }

private:
	clock_t tc;
};

/// This measures the remaining time
template <typename T>
class Countdown
{
public:
	Countdown(double s=0):msec(s*1000) { start(); }
	inline void start(double s) { msec=s*1000; t.start(); }
	inline void start() { t.start(); }
	inline bool end() { return getMs() < 0; }
	inline double getMs() { return msec - t.getMs(); }
	inline double getSeconds() { return getMs() / 1000; }

private:
	T t;
	double msec;
};

typedef Countdown<TimerR> CountdownR;
typedef Countdown<TimerT> CountdownT;

}} // namespaces

#endif
