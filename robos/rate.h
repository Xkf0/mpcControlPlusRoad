#ifndef ROBOS_RATE_H_
#define ROBOS_RATE_H_
#include "robos/time.h"
#include "robos/duration.h"
namespace robos{
//class Duration;

/**
 * @class Rate
 * @brief Class to help run loops at a desired frequency
 */
class Rate{
public:
    /**
    * @brief  Constructor, creates a Rate
    * @param  frequency The desired rate to run at in Hz
    */
    Rate(double frequency) 
	    : start_(Time::now())
	    , expected_cycle_time_(1.0 / frequency)
	    , actual_cycle_time_(0.0){}

    explicit Rate(const Duration& d)
	    : start_(Time::now())
	    , expected_cycle_time_(d.sec, d.nsec)
	    , actual_cycle_time_(0.0){}

    /**
    * @brief  Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
    * @return True if the desired rate was met for the cycle, false otherwise.
    */
    bool sleep() {
        Time expected_end = start_ + expected_cycle_time_;

        Time actual_end = Time::now();

        // detect backward jumps in time
        if (actual_end < start_){
            expected_end = actual_end + expected_cycle_time_;
        }

        //calculate the time we'll sleep for
        Duration sleep_time = expected_end - actual_end;

        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;

        //make sure to reset our start time
        start_ = expected_end;

        //if we've taken too much time we won't sleep
        if (sleep_time <= Duration(0.0)){
            // if we've jumped forward in time, or the loop has taken more than a full extra
            // cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_){
                start_ = actual_end;
            }
            // return false to show that the desired rate was not met
            return false;
        }

        return sleep_time.sleep();
    }

    /**
    * @brief  Sets the start time for the rate to now
    */
    void reset() {
        start_ = Time::now();
    }

    /**
    * @brief  Get the actual run time of a cycle from start to sleep
    * @return The runtime of the cycle
    */
    Duration cycleTime() const {
        return actual_cycle_time_;
    }

    /**
    * @brief Get the expected cycle time -- one over the frequency passed in to the constructor
    */
    Duration expectedCycleTime() const { 
        return expected_cycle_time_; 
    }

private:
    Time start_;
    Duration expected_cycle_time_, actual_cycle_time_;
};

}
#endif // ROBOS_RATE_H_