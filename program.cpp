#include "include/wooden_ball.hpp"
#include "include/cannon_ball.hpp"
#include "include/solver_shooting.hpp"
#include <functional>
#include <iostream>
#include <cstdio>
#include <math.h>

using namespace std;
using namespace solver;

void wooden_ball_example(double theta) {
    second_derivative* _second_derivative = new wooden_ball(0.1, 600, 0.1, 9.81, 1.29);

    double t0 = 0;
    double tmax = 10;
    vectorn start_position(0.0);
    double h = 0.01;
        
    vectorn start_velocity(50 * sin(theta));
    vectorn start_acceleration = _second_derivative->get_value(t0, start_position, start_velocity);

    integrator_rkn _integrator_rkn(_second_derivative, t0, start_position, start_velocity, h, start_acceleration);

    cout << t0 << "\t" << start_position.get(0) << "\t" << start_velocity.get(0) << "\t" << start_acceleration.get(0) << endl;

    double t; // time t [s]
    vectorn* position = new vectorn(2); // position y [m]
    vectorn* velocity = new vectorn(2); // velocity v [m/s]
    vectorn* acceleration = new vectorn(2); // accelearion a [m/s^2]

    while (t < tmax) {
        _integrator_rkn.step(&t, position, velocity, acceleration);
        cout << t << "\t" << position->get(0) << "\t" 
             << velocity->get(0) << "\t" 
             << acceleration->get(0) << endl;
    }

    delete _second_derivative;
    delete position;
    delete velocity;
    delete acceleration;
}

void cannon_ball_example(double theta) {
    second_derivative* _second_derivative = new cannon_ball(0.1, 6000, 0.5, 9.81, 1.29);

    double t0 = 0;
    double tmax = 12;
    double h = 0.01;

    vectorn start_position(2);
    start_position.set(0, 0.0);
    start_position.set(1, 0.0);

    vectorn start_velocity(2);
    start_velocity.set(0, 50 * cos(theta));
    start_velocity.set(1, 50 * sin(theta));

    vectorn start_acceleration = _second_derivative->get_value(t0, start_position, start_velocity);

    integrator_rkn _integrator_rkn(_second_derivative, t0, start_position, start_velocity, h, start_acceleration);

    cout << start_position.get(0) << "\t" << start_position.get(1) << "\t"
         << t0 << "\t" << start_velocity.get(0) << "\t" << start_velocity.get(1) << "\t"
         << start_acceleration.get(0) << "\t" << start_acceleration.get(1) << endl;

    double t; // time t [s]
    vectorn* position = new vectorn(2); // position x,y [m]
    vectorn* velocity = new vectorn(2); // velocity dx/dt,dy/dt = vx,vy [m/s]
    vectorn* acceleration = new vectorn(2); // acceleration d2x/dt,d2y/dt = dvx/dt,dvy/dt = ax,ay [m/s^2]

    while (t < tmax) {
        _integrator_rkn.step(&t, position, velocity, acceleration);
        cout << position->get(0) << "\t" << position->get(1) << "\t"
             << t << "\t" << velocity->get(0) << "\t" << velocity->get(1) << "\t"
             << acceleration->get(0) << "\t" << acceleration->get(1) << endl;
    }

    delete _second_derivative;
    delete position;
    delete velocity;
    delete acceleration;
}

void shooting_method_example(double theta) {

    second_derivative* _second_derivative = new cannon_ball(0.1, 6000, 0.5, 9.81, 1.29);

    double          t0 =                0.0000;

    /// step to be used to obtain highest precision

    //  double          h =                 0.0001;

    /// step to be used to obtain a good precision

    double          h =                 0.01;
    
    vectorn start_position(2);
    vectorn start_velocity(2);
    vectorn start_acceleration(2);
    vectorn final_position(2);

    start_position.set(0, 0.000, vectorn_flags::cost_position);
    start_position.set(1, 0.000, vectorn_flags::stop_position);

    final_position.set(0, 605.0);
    final_position.set(1, 0.000);

    start_velocity.set(0, 50 * cos(theta));
    start_velocity.set(1, 50 * sin(theta));

    start_acceleration = _second_derivative->get_value(t0, start_position, start_velocity);
    
    final_position.inherit_flags(start_position);
    start_velocity.inherit_flags(start_position);
    start_acceleration.inherit_flags(start_position);

    solver_shooting cannon_ball_solver(_second_derivative, start_position, start_velocity, start_acceleration, final_position, t0, h);

    cannon_ball_solver.set_function_adjust([](double target, double x, double dx)  { return (target * dx) / x; });
    cannon_ball_solver.set_function_cost(  [](double target, double x)             {     return target - x;    });
    cannon_ball_solver.set_function_stop(  [](double target, double x)             {     return x < target;    });

    /// epsilon to be used to obtain highest precision

    //  vectorn solution = cannon_ball_solver.shoot(0.001);

    /// epsilon to be used to obtain a good precision

    vectorn solution = cannon_ball_solver.shoot(0.5);

    delete _second_derivative;
    
    //  cout << "needed velocity on (x, y) axis: (" << solution.get(0) << ", " << solution.get(1) << ")" << endl;
}

int main(int argc, char ** argv){
    double theta = 90.0 * M_PI / 180.0;
    int example_string = 0;

    if (argc > 1)
        example_string = stoi(argv[1]);

    if (argc > 2)
        theta = stoi(argv[2]) * M_PI / 180.0;
    
    switch (example_string) {
        case 0 : 
            /// wooden_ball example

            wooden_ball_example(theta);
            break;
        case 1 : 
            /// cannon ball example

            cannon_ball_example(theta);
            break;
        case 2 : 
            /// shooting method to a desired goal implementing cannon ball example

            shooting_method_example(theta);
            break;
    }

    return 0;
}
