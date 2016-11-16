#ifndef _TIME_STOPWATCH_H
#define _TIME_STOPWATCH_H

#include <iostream>
#include <fstream>

// Class to measure time
template<typename TimeT = std::chrono::milliseconds, //
        typename ClockT = std::chrono::high_resolution_clock,
        typename DurationT = double>
class Stopwatch {
private:
    std::chrono::time_point<ClockT> _start, _end;
public:
    Stopwatch() {
        start();
    }
    void start() {
        _start = _end = ClockT::now();
    }
    DurationT stop() {
        _end = ClockT::now();
        return elapsed();
    }
    DurationT elapsed() {
        auto delta = std::chrono::duration_cast < TimeT > (_end - _start);
        return (double) delta.count();
    }
};

#endif // _TIME_STOPWATCH_H
