#include "interval.hpp"

Interval::Interval(const double inf, const double sup) : interval(inf, sup)
{
    assert(inf <= sup);
}


const std::string Interval::toString() const {
    std::string a = (interval.first == -DBL_MAX) ? "-inf" : std::to_string(interval.first);
    std::string b = (interval.second == DBL_MAX) ? "+inf" : std::to_string(interval.second);
    return "[" + a + ", " + b + "]"; 
}