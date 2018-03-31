#include "../include/cannon_ball.hpp"

#include <math.h>

using namespace solver;

cannon_ball::cannon_ball(double _ball_diameter, double _ball_rho, double _drag_coefficient, double _g, double _air_rho) {
    ball_diameter = _ball_diameter;
    ball_rho = _ball_rho;
    drag_coefficient = _drag_coefficient;
    g = _g;
    air_rho = _air_rho;

    double volume = M_PI * pow(ball_diameter, 3) / 6;
    ball_mass = ball_rho * volume;
    ball_area = 0.25 * M_PI * pow(ball_diameter, 2);
}

cannon_ball::~cannon_ball() { }

vectorn cannon_ball::get_value(double t, vectorn y, vectorn v) {
    double vx = v.get(0);
    double vy = v.get(1);
    double vx_sqr = pow(vx, 2);
    double vy_sqr = pow(vy, 2);
    double v_sqr = vx_sqr + vy_sqr;
    double v_total = sqrt(v_sqr);

    double cos_alpha = 1.0;
    double cos_beta = 0.0;
    if (v_total > 1e-12) {
        cos_alpha = vx / v_total;
        cos_beta = vy / v_total;
    }

    double drag = 0.5 * drag_coefficient * air_rho * v_sqr * ball_area;
    double drag_x = -drag * cos_alpha;
    double drag_y = -drag * cos_beta;

    vectorn force(2);
    force.set(0, drag_x);
    force.set(1, -ball_mass * g + drag_y);

    vectorn acceleration = force / ball_mass;

    return acceleration;
}