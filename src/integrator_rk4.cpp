#include "../include/integrator_rk4.hpp"

using namespace solver;

integrator_rk4::integrator_rk4(first_derivative* __first_derivative, double _t0, vectorn _y0, double _h, vectorn _dy0) {
    _first_derivative = __first_derivative;
    t0 = _t0;
    y0 = _y0.copy();
    h = _h;
    dy0 = _dy0.copy();
}

void integrator_rk4::step(double* t1, vectorn* y1, vectorn* dy1) { 
    vectorn k1 = *dy0;
    vectorn k2 = _first_derivative->get_value(t0 + h/2, *y0 + h / 2 * k1);
    vectorn k3 = _first_derivative->get_value(t0 + h / 2, *y0 + h / 2 * k2);
    vectorn k4 = _first_derivative->get_value(t0 + h, *y0 + h * k3);

    *t1 = t0 + h;
    *y1 = *y0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    *dy1 = _first_derivative->get_value(*t1, *y1);

    t0 = *t1;
    y0 = y1->copy();
    dy0 = dy1->copy();
}
