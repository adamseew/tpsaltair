#include "../include/wooden_ball.hpp"
#include <math.h>

using namespace solver;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

wooden_ball::wooden_ball(double _ball_diameter, double _ball_rho, double _drag_coefficient, double _g, double _air_rho) {
    ball_diameter = _ball_diameter;
    ball_rho = _ball_rho;
    drag_coefficient = _drag_coefficient;
    g = _g;
    air_rho = _air_rho;

    double v = M_PI * pow(ball_diameter, 3) / 6;
    ball_mass = ball_rho * v;
    ball_area = 0.25 * M_PI * ball_diameter * ball_diameter;
}

wooden_ball::~wooden_ball() { }

vectorn wooden_ball::get_value(double t, vectorn y, vectorn v) {
    double f = -ball_mass * g -sgn(v.get(0)) * 0.5 * drag_coefficient * air_rho * pow(v.get(0), 2) * ball_area;
    vectorn d2y(f / ball_mass);
    return d2y;
}