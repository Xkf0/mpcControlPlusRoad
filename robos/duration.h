#ifndef ROBOS_DURATION_H_
#define ROBOS_DURATION_H_

/*********************************************************************
 ** Pragmas
 *********************************************************************/

#ifdef _MSC_VER
  // Rostime has some magic interface that doesn't directly include
  // its implementation, this just disbales those warnings.
  #pragma warning(disable: 4244)
  #pragma warning(disable: 4661)
#endif

#include <iostream>
#include <math.h>
#include <stdexcept>
#include <climits>
#include <stdint.h>
#include <limits>
#include <thread>

namespace robos {
namespace {
static void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec) {
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + nsec / 1000000000L;
    if (nsec_part < 0) {
        nsec_part += 1000000000L;
        --sec_part;
    }

    if (sec_part < std::numeric_limits<int32_t>::min() || sec_part > std::numeric_limits<int32_t>::max())
        throw std::runtime_error("Duration is out of dual 32-bit range");

    sec = sec_part;
    nsec = nsec_part;
}

static void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec) {
    int64_t sec64 = sec;
    int64_t nsec64 = nsec;

    normalizeSecNSecSigned(sec64, nsec64);

    sec = (int32_t)sec64;
    nsec = (int32_t)nsec64;
}

/*
    * These have only internal linkage to this translation unit.
    * (i.e. not exposed to users of the time classes)
    */

    /**
    * @brief Simple representation of the rt library nanosleep function.
    */
static int ros_nanosleep(const uint32_t& sec, const uint32_t& nsec) {
#if defined(_WIN32)
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9 + nsec)));
    return 0;
#else
    timespec req = { sec, nsec };
    return nanosleep(&req, NULL);
#endif
}

/**
    * @brief Go to the wall!
    *
    * @todo Fully implement the win32 parts, currently just like a regular sleep.
    */
static bool ros_wallsleep(uint32_t sec, uint32_t nsec) {
#if defined(_WIN32)
    ros_nanosleep(sec, nsec);
#else
    timespec req = { sec, nsec };
    timespec rem = { 0, 0 };
    while (nanosleep(&req, &rem)) {
        req = rem;
}
#endif
    return true;
}
}
/**
 * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template <class T>
class DurationBase{
public:
  int32_t sec, nsec;

  DurationBase() : sec(0), nsec(0) {}

  DurationBase(int32_t _sec, int32_t _nsec) 
      : sec(_sec), nsec(_nsec) {
      normalizeSecNSecSigned(sec, nsec);
  }

  explicit DurationBase(double t){
      fromSec(t);
  };

  T operator+(const T &rhs) const {
      T t;
      return t.fromNSec(toNSec() + rhs.toNSec());
  }

  T operator-(const T &rhs) const {
      T t;
      return t.fromNSec(toNSec() - rhs.toNSec());
  }

  T operator-() const {
      T t;
      return t.fromNSec(-toNSec());
  }

  T operator*(double scale) const {
      return T(toSec() * scale);
  }

  T& operator+=(const T &rhs) {
      *this = *this + rhs;
      return *static_cast<T*>(this);
  }

  T& operator-=(const T &rhs) {
      *this += (-rhs);
      return *static_cast<T*>(this);
  }

  T& operator*=(double scale) {
      fromSec(toSec() * scale);
      return *static_cast<T*>(this);
  }

  bool operator==(const T &rhs) const {
      return sec == rhs.sec && nsec == rhs.nsec;
  }

  inline bool operator!=(const T &rhs) const {
      return !(*static_cast<const T*>(this) == rhs); 
  }

  bool operator>(const T &rhs) const {
      if (sec > rhs.sec)
          return true;
      else if (sec == rhs.sec && nsec > rhs.nsec)
          return true;
      return false;
  }

  bool operator<(const T &rhs) const {
      if (sec < rhs.sec)
          return true;
      else if (sec == rhs.sec && nsec < rhs.nsec)
          return true;
      return false;
  }

  bool operator>=(const T &rhs) const {
      if (sec > rhs.sec)
          return true;
      else if (sec == rhs.sec && nsec >= rhs.nsec)
          return true;
      return false;
  }

  bool operator<=(const T &rhs) const {
      if (sec < rhs.sec)
          return true;
      else if (sec == rhs.sec && nsec <= rhs.nsec)
          return true;
      return false;
  }

  double toSec() const { 
      return static_cast<double>(sec) + 1e-9*static_cast<double>(nsec);
  };

  int64_t toNSec() const {
      return static_cast<int64_t>(sec)*1000000000ll + static_cast<int64_t>(nsec); 
  };

  T& fromSec(double t) {
      int64_t sec64 = static_cast<int64_t>(floor(t));
      if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
          throw std::runtime_error("Duration is out of dual 32-bit range");
      sec = static_cast<int32_t>(sec64);
      nsec = static_cast<int32_t>(std::round((t - sec) * 1e9));
      int32_t rollover = nsec / 1000000000ul;
      sec += rollover;
      nsec %= 1000000000ul;
      return *static_cast<T*>(this);
  }

  T& fromNSec(int64_t t) {
      int64_t sec64 = t / 1000000000LL;
      if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
          throw std::runtime_error("Duration is out of dual 32-bit range");
      sec = static_cast<int32_t>(sec64);
      nsec = static_cast<int32_t>(t % 1000000000LL);

      normalizeSecNSecSigned(sec, nsec);

      return *static_cast<T*>(this);
  }

  bool isZero() const {
      return sec == 0 && nsec == 0;
  }
};

//class Rate;

/**
 * \brief Duration representation for use with the Time class.
 *
 * ros::DurationBase provides most of its functionality.
 */
class Duration : public DurationBase<Duration>{
public:
  Duration()
  : DurationBase<Duration>(){}

  Duration(int32_t _sec, int32_t _nsec)
  : DurationBase<Duration>(_sec, _nsec){}

  explicit Duration(double t) { fromSec(t); }

  /**
   * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
   * @return True if the desired sleep duration was met, false otherwise.
   */
  bool sleep() const {
      return ros_wallsleep(sec, nsec);
  }
};

#define DURATION_MAX Duration(std::numeric_limits<int32_t>::max(), 999999999);
#define DURATION_MIN Duration(std::numeric_limits<int32_t>::min(), 0);

inline std::ostream& operator<<(std::ostream& os, const Duration& rhs){
    if (rhs.sec >= 0 || rhs.nsec == 0){
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    }
    else{
        os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0') << (1000000000 - rhs.nsec);
    }
    return os;
}

}

#endif // ROBOS_DURATION_H_
