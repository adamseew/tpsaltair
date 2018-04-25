#include "../include/quadrotor_1d.hpp"
#include <vector>

#include <math.h>

using namespace solver;
using namespace std;

quadrotor_1d::quadrotor_1d(double _quadrotor_mass, double _drag_coefficient, double _g, double _moment_inertia, vector<double> _u) {
    quadrotor_1d(_quadrotor_mass, _drag_coefficient, _g, _moment_inertia, _u, 1);
}

quadrotor_1d::quadrotor_1d(double _quadrotor_mass, double _drag_coefficient, double _g, double _moment_inertia, vector<double> _u, int _time_progress_unit) {
    quadrotor_mass = _quadrotor_mass;
    drag_coefficient = _drag_coefficient;
    g = _g;
    moment_inertia = _moment_inertia;
    u = _u;

    /// the function get_value is called 5 x per each integration step, 4 steps for runge kutta algorithm, one is used to store the derivative of the new initial conditions

    time_progress_unit = _time_progress_unit * 5;
    _time = 0;
    __time = -1;
}

quadrotor_1d::~quadrotor_1d() { }

vectorn quadrotor_1d::get_value(double t, vectorn q) {
    double x      = q.get(0);
    double theta  = q.get(1);
    double px     = q.get(2);
    double ptheta = q.get(3);

    vectorn pq(4);

    pq.set(0, px / quadrotor_mass);
    pq.set(1, ptheta / (moment_inertia * (1 + pow(cos(theta), 2))));
    pq.set(2, (-1) * quadrotor_mass * g * tan(theta) + (drag_coefficient / quadrotor_mass) * px);

    __time++;
    if (__time % time_progress_unit == 0) {
        _time++;
    }
    if (_time >= u.size())
        _time = u.size() - 1;

    pq.set(3, (pow(ptheta, 2) * sin(theta) * cos(theta)) / (moment_inertia * pow((1 + pow(cos(theta), 2)), 2)) + u.at(_time));

    return pq;
}
