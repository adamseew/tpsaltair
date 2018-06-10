#include "../include/integrator_rkn.hpp"

using namespace solver;

integrator_rkn::integrator_rkn(second_derivative* __second_derivative, double _t0, vectorn _y0, vectorn _dy0, double _h, vectorn _d2y0) {
    _second_derivative = __second_derivative;
    t0 = _t0;
    y0 = _y0.copy();
    dy0 = _dy0.copy();
    h = _h;
    d2y0 = _d2y0.copy();
}

integrator_rkn::~integrator_rkn() {
    delete y0;
    delete dy0;
    delete d2y0;
}

void integrator_rkn::step(double* t1, vectorn* y1, vectorn* dy1, vectorn* d2y1) {
    double h2 = h * h;

    vectorn k1 = *d2y0;
    vectorn k2 = *_second_derivative->get_value(t0 + h/2, *y0 + h/2 * *dy0 + h2/8 * k1, *dy0 + h/2 * k1).inherit_flags(*y0);
    vectorn k3 = *_second_derivative->get_value(t0 + h/2, *y0 + h/2 * *dy0 + h2/8 * k1, *dy0 + h/2 * k2).inherit_flags(*y0);
    vectorn k4 = *_second_derivative->get_value(t0 + h  , *y0 + h   * *dy0 + h2/2 * k3, *dy0 + h   * k3).inherit_flags(*y0);

    *t1 = t0 + h;
    *y1 = *y0 + h * *dy0 + h2/6 * (k1 + k2 + k3);
    *dy1 = *dy0 + h/6 * (k1 + 2 * k2 + 2 * k3 + k4);
    *d2y1 = _second_derivative->get_value(*t1, *y1, *dy1);

    delete y0;
    delete dy0;
    delete d2y0;

    t0 = *t1;
    y0 = y1->copy();
    dy0 = dy1->copy();
    d2y0 = d2y1->copy();
}
