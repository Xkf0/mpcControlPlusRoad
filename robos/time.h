#ifndef ROBOS_TIME_H_
#define ROBOS_TIME_H_

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
/*********************************************************************
** Pragmas
*********************************************************************/

#ifdef _MSC_VER
// Rostime has some magic interface that doesn't directly include
// its implementation, this just disables those warnings.
#pragma warning(disable: 4244)
#pragma warning(disable: 4661)
#endif

/*********************************************************************
** Headers
*********************************************************************/

#include <iostream>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <chrono>
#include "duration.h"

/*********************************************************************
** Cross Platform Headers
*********************************************************************/
// time related includes for macOS
#if defined(__APPLE__)
#include <mach/clock.h>
#include <mach/mach.h>
#endif  // defined(__APPLE__)

#ifdef _WINDOWS
#include <chrono>
#include <thread>
#include <windows.h>
#endif

#if defined(_WIN32)
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

namespace robos {
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)
/*********************************************************************
** Functions
*********************************************************************/

static void normalizeSecNSec(uint64_t& sec, uint64_t& nsec){
	uint64_t nsec_part = nsec % 1000000000UL;
	uint64_t sec_part = nsec / 1000000000UL;

	if (sec + sec_part > std::numeric_limits<uint32_t>::max())
		throw std::runtime_error("Time is out of dual 32-bit range");

	sec += sec_part;
	nsec = nsec_part;
}

static void normalizeSecNSec(uint32_t& sec, uint32_t& nsec){
	uint64_t sec64 = sec;
	uint64_t nsec64 = nsec;

	normalizeSecNSec(sec64, nsec64);

	sec = (uint32_t)sec64;
	nsec = (uint32_t)nsec64;
}

static void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec){
	int64_t nsec_part = nsec % 1000000000L;
	int64_t sec_part = sec + nsec / 1000000000L;
	if (nsec_part < 0){
		nsec_part += 1000000000L;
		--sec_part;
	}

	if (sec_part < 0 || sec_part > std::numeric_limits<uint32_t>::max())
		throw std::runtime_error("Time is out of dual 32-bit range");

	sec = sec_part;
	nsec = nsec_part;
}

/*********************************************************************
	** Time Classes
	*********************************************************************/

	/**
	* \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
	* This should not need to be used directly.
	*/
template<class T, class D>
class TimeBase{
public:
	uint32_t sec, nsec;

	TimeBase() : sec(0), nsec(0) {}

	TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec) {
		normalizeSecNSec(sec, nsec);
	}
	explicit TimeBase(double t) {
		fromSec(t);
	}

	D operator-(const T& rhs) const {
		D d;
		return d.fromNSec(toNSec() - rhs.toNSec());
	}

	T operator+(const D& rhs) const {
		int64_t sec_sum = static_cast<uint64_t>(sec) + static_cast<uint64_t>(rhs.sec);
		int64_t nsec_sum = static_cast<uint64_t>(nsec) + static_cast<uint64_t>(rhs.nsec);

		// Throws an exception if we go out of 32-bit range
		normalizeSecNSecUnsigned(sec_sum, nsec_sum);

		// now, it's safe to downcast back to uint32 bits
		return T(static_cast<uint32_t>(sec_sum), static_cast<uint32_t>(nsec_sum));
	}

	T operator-(const D& rhs) const {
		return *static_cast<const T*>(this) + (-rhs);
	}

	T& operator+=(const D& rhs) {
		*this = *this + rhs;
		return *static_cast<T*>(this);
	}

	T& operator-=(const D& rhs) {
		*this += (-rhs);
		return *static_cast<T*>(this);
	}

	bool operator==(const T& rhs) const {
		return sec == rhs.sec && nsec == rhs.nsec;
	}

	inline bool operator!=(const T& rhs) const {
		return !(*static_cast<const T*>(this) == rhs);
	}

	bool operator>(const T& rhs) const {
		if (sec > rhs.sec)
			return true;
		else if (sec == rhs.sec && nsec > rhs.nsec)
			return true;
		return false;
	}

	bool operator<(const T& rhs) const {
		if (sec < rhs.sec)
			return true;
		else if (sec == rhs.sec && nsec < rhs.nsec)
			return true;
		return false;
	}

	bool operator>=(const T& rhs) const {
		if (sec > rhs.sec)
			return true;
		else if (sec == rhs.sec && nsec >= rhs.nsec)
			return true;
		return false;
	}

	bool operator<=(const T& rhs) const {
		if (sec < rhs.sec)
			return true;
		else if (sec == rhs.sec && nsec <= rhs.nsec)
			return true;
		return false;
	}

	double toSec()  const {
		return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
	};

	T& fromSec(double t) {
		int64_t sec64 = static_cast<int64_t>(floor(t));
		if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
			throw std::runtime_error("Time is out of dual 32-bit range");
		sec = static_cast<uint32_t>(sec64);
		nsec = static_cast<uint32_t>(std::round((t - sec) * 1e9));
		// avoid rounding errors
		sec += (nsec / 1000000000ul);
		nsec %= 1000000000ul;
		return *static_cast<T*>(this);
	}

	uint64_t toNSec() const {
		return static_cast<uint64_t>(sec) * 1000000000ull + static_cast<uint64_t>(nsec);
	}

	T& fromNSec(uint64_t t) {
		uint64_t sec64 = 0;
		uint64_t nsec64 = t;

		normalizeSecNSec(sec64, nsec64);

		sec = static_cast<uint32_t>(sec64);
		nsec = static_cast<uint32_t>(nsec64);

		return *static_cast<T*>(this);
	}

	inline bool isZero() const {
		return sec == 0 && nsec == 0;
	}

	inline bool is_zero() const {
		return isZero();
	}
};

/**
	* \brief Time representation.  May either represent wall clock time or ROS clock time.
	*
	* ros::TimeBase provides most of its functionality.
	*/
class Time : public TimeBase<Time, Duration>{
public:
	Time(): TimeBase<Time, Duration>(){}

	Time(uint32_t _sec, uint32_t _nsec)
		: TimeBase<Time, Duration>(_sec, _nsec){}

	explicit Time(double t) { fromSec(t); }

	/**
		* \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
		* ROS clock.  Otherwise returns the current wall clock time.
		*/
	static Time now() {
		Time t;
#if !defined(_WIN32)
#if HAS_CLOCK_GETTIME
		timespec start;
		clock_gettime(CLOCK_REALTIME, &start);
		if (start.tv_sec < 0 || start.tv_sec > std::numeric_limits<uint32_t>::max())
			throw std::runtime_error("Timespec is out of dual 32-bit range");
		t.sec = start.tv_sec;
		t.nsec = start.tv_nsec;
#else
		struct timeval timeofday;
		gettimeofday(&timeofday, NULL);
		if (timeofday.tv_sec < 0 || timeofday.tv_sec > std::numeric_limits<uint32_t>::max())
			throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
		t.sec = timeofday.tv_sec;
		t.nsec = timeofday.tv_usec * 1000;
#endif
#else
		uint64_t now_s = 0;
		uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		normalizeSecNSec(now_s, now_ns);

		t.sec = (uint32_t)now_s;
		t.nsec = (uint32_t)now_ns;
#endif
		return t;
	}
	/**
		* \brief Sleep until a specific time has been reached.
		* @return True if the desired sleep time was met, false otherwise.
		*/
	static bool sleepUntil(const Time& end) {
		Duration d(end - Time::now());
		if (d > Duration(0)){
			return d.sleep();
		}
		return true;
	}
};

#define  TIME_MAX Time(std::numeric_limits<uint32_t>::max(), 999999999)
#define  TIME_MIN Time(0, 1)

inline std::ostream& operator<<(std::ostream& os, const Time& rhs) {
	os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
	return os;
}
}

#endif // ROBOS_TIME_H_
