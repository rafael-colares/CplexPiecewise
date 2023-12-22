#ifndef __linearfunction__hpp
#define __linearfunction__hpp

#include <utility>
#include <iostream>
#include <assert.h>

typedef std::pair<double, double> Point;

/**
 * This class models a linear function in the R^2 space. The fucntion is defined as f(x) = ax + b.
 */
class LinearFunction{

    private:
        double a;
        double b;
		
    public:
        /** Constructor. @param a The slope. @param b The independent term.**/
        LinearFunction(const double a = 0.0, const double b = 0.0): a(a), b(b) {}

        /** Constructor. @param a The slope. @param p A point that the line passes through.**/
        LinearFunction(const double a, const Point &p);

        /** Constructor. Constructs the linear function that passes through the two given points. @param u First point. @param v Second point.**/
        LinearFunction(const Point &u, const Point &v);
        
        /** Copy constructor. **/
        LinearFunction(const LinearFunction &f): a(f.getA()), b(f.getB()){}

        /** Returns the slope of the linear function. **/
        const double getA() const {return a;}

        /** Returns the independent term of the linear function. **/
        const double getB() const {return b;}

        /** Sets the slope of the linear function. **/
        void setA(double a) { this->a = a; }

        /** Sets the independent term of the linear function. **/
        void setB(double b) { this->b = b; }

        /** Returns the function value at x. **/
        const double getValueAt(double x) const { return a*x + b;}

        /** Returns the point of intersection with the given linear function. **/
        Point getIntersectionPoint(const LinearFunction &f);

        std::string toString() const {return std::to_string(a) + "*x + " + std::to_string(b); }
};

#endif