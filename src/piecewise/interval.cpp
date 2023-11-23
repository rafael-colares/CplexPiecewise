#include "interval.hpp"

Interval::Interval(const double inf, const double sup) : interval(inf, sup)
{
    assert(inf <= sup);
}