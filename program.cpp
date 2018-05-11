#include "include/wooden_ball.hpp"
#include "include/cannon_ball.hpp"
#include "include/quadrotor_1d.hpp"
#include "include/solver_shooting.hpp"
#include <functional>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <string>
#include <vector>
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

void shooting_method_example(double theta, int extreme_precision) {

    second_derivative* _second_derivative = new cannon_ball(0.1, 6000, 0.5, 9.81, 1.29);

    double          t0 =                0.0000,
                    h =                 0.01;
    
    if (extreme_precision)
                    h =                 0.0001;
    
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

    vectorn solution;
    if (extreme_precision)
        solution = cannon_ball_solver.shoot(0.001);
    else
        solution = cannon_ball_solver.shoot(0.5);

    delete _second_derivative;
    
    //  cout << "needed velocity on (x, y) axis: (" << solution.get(0) << ", " << solution.get(1) << ")" << endl;
}

void quadrotor_1d_example(double control_adjustment) {

#pragma region ___C_S_V___D_A_T_A___
    vector<double> u;
    ifstream fin("traj2.csv");
    string item;
    for (string line; getline(fin, line); ) {
        istringstream in(line);

        while(getline(in, item, ','))
            u.push_back(atof(item.c_str()) * control_adjustment);
    }
#pragma endregion ___C_S_V___D_A_T_A___

    first_derivative* _first_derivative = new quadrotor_1d(20.81, 0.5, 9.81, 0.0048, u, 16);

    double          t0 =                0.0000,
                    h =                 0.01;
    
    vectorn start_q(4);

    vector<vectorn_flags> _flags;
    _flags.push_back(vectorn_flags::cost_position);
    _flags.push_back(vectorn_flags::stop_position);

    start_q.set(0, 0, _flags);
    start_q.set(1, 0);
    start_q.set(2, 0);
    start_q.set(3, 0);

    vectorn final_q(4);

    final_q.set(0, 1);
    final_q.set(1, 0);
    final_q.set(2, 0.25 * 0.81);
    final_q.set(3, 0);
    final_q.inherit_flags(start_q);

    vectorn start_dq = _first_derivative->get_value(t0, start_q);
    solver_shooting quadrotor_1d_solver(_first_derivative, start_q, start_dq, final_q, t0, h);

    quadrotor_1d_solver.set_function_adjust([](double target, double x, double dx)  { return (target * dx) / x; });
    quadrotor_1d_solver.set_function_cost(  [](double target, double x)             {     return target - x;    });
    quadrotor_1d_solver.set_function_stop(  [](double target, double x)             {     return x > target;    });

    vectorn solution = quadrotor_1d_solver.shoot(0.5);

    delete _first_derivative;
    
    //  cout << "needed velocity on (x, y) axis: (" << solution.get(0) << ", " << solution.get(1) << ")" << endl;
}

void forward_bacward_integration() {
#pragma region ___C_S_V___D_A_T_A___
    vector<double> u;
    ifstream fin("traj2.csv");
    string item;
    for (string line; getline(fin, line); ) {
        istringstream in(line);

        while(getline(in, item, ','))
            u.push_back(atof(item.c_str()));
    }
#pragma endregion ___C_S_V___D_A_T_A___

    first_derivative* _first_derivative = new quadrotor_1d(20.81, 0.5, 9.81, 0.0048, u, 16);

    double                  t0 =            0.0000,
                            h =             0.0100,
                            t =             0.0000;

    vectorn fw_start_q(4);

    vector<vectorn_flags>   _flags;
    _flags.push_back(vectorn_flags::cost_position);
    _flags.push_back(vectorn_flags::stop_position);

    fw_start_q.set(0, 0, _flags);
    fw_start_q.set(1, 0);
    fw_start_q.set(2, 0);
    fw_start_q.set(3, 0);

    vectorn fw_final_q(4);

    fw_final_q.set(0, 1);
    fw_final_q.set(1, 0);
    fw_final_q.set(2, 0.25 * 0.81);
    fw_final_q.set(3, 0);
    fw_final_q.inherit_flags(fw_start_q);

    vectorn fw_start_dq = _first_derivative->get_value(t0, fw_start_q);

    integrator_rk4* fw_integrator = new integrator_rk4(_first_derivative, t0, fw_start_q, h, fw_start_dq);

    cout << "forward integration started" << endl;

    ofstream file;
    file.open(".forward.dat");

    for (int i = 0; i < fw_start_q.length()  ; i++)
        file << fw_start_q.get(i)   << "\t";
    for (int i = 0; i < fw_start_dq.length() ; i++)
        file << fw_start_dq.get(i)  << "\t";

    file << endl;

    vectorn*                fw_q =          new vectorn(fw_start_q.length());
    vectorn*                fw_dq =         new vectorn(fw_start_dq.length());

    do {
        fw_integrator->step(&t, fw_q, fw_dq);

        for (int i = 0; i < fw_q->length()  ; i++)
            file << fw_q->get(i)   << "\t";
        for (int i = 0; i < fw_dq->length() ; i++)
            file << fw_dq->get(i)  << "\t";

        file << endl;

    } while (t < 10);

    file.close();

    vectorn bw_start_q(*fw_q);

    vectorn bw_final_q(fw_start_q);

    vectorn bw_start_dq = _first_derivative->get_value(t, bw_start_q);

    integrator_rk4* bw_integrator = new integrator_rk4(_first_derivative, t, bw_start_q, (-1) * h, bw_start_dq);

    cout << "forward integration done" << endl;
    cout << "backward integration started" << endl;

    file.open(".backward.dat");

    for (int i = 0; i < bw_start_q.length()  ; i++)
        file << bw_start_q.get(i)   << "\t";
    for (int i = 0; i < bw_start_dq.length() ; i++)
        file << bw_start_dq.get(i)  << "\t";

    file << endl;

    vectorn*                bw_q =          new vectorn(bw_start_q.length());
    vectorn*                bw_dq =         new vectorn(bw_start_dq.length());


    do {
        bw_integrator->step(&t, bw_q, bw_dq);

        for (int i = 0; i < bw_q->length()  ; i++)
            file << bw_q->get(i)   << "\t";
        for (int i = 0; i < bw_dq->length() ; i++)
            file << bw_dq->get(i)  << "\t";

        file << endl;

    } while (t > 0);

    file.close();

    cout << "backward integration done" << endl;

    cout << endl << "------started from-------" << endl;
    cout << "x\ttheta\tpx\tptheta" << endl;
    for (int i = 0; i < fw_start_q.length()  ; i++)
        cout << fw_start_q.get(i)   << "\t";
    //for (int i = 0; i < fw_start_dq.length() ; i++)
        //cout << fw_start_dq.get(i)  << "\t";

    cout << endl << endl << "-------reached to--------" << endl;
    cout << "x\ttheta\tpx\tptheta" << endl;
    for (int i = 0; i < bw_q->length()  ; i++)
        cout << bw_q->get(i)   << "\t";
    //for (int i = 0; i < bw_dq->length() ; i++)
        //cout << bw_dq->get(i)  << "\t";
    cout << endl;
    
}

int main(int argc, char ** argv){
    double  theta =             90.0 * M_PI / 180.0;
    int     example_string =    0,
            extreme_precision = 0;

    if (argc > 1)
        example_string = stoi(argv[1]);

    if (argc > 2)
        theta = stoi(argv[2]) * M_PI / 180.0;

    if (argc > 3)
        extreme_precision = stoi(argv[3]);
    
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

            shooting_method_example(theta, extreme_precision);
            break;
        case 3 :
            /// shooting method to describe optimal trajectory of a quadrotor in 1d

            quadrotor_1d_example(theta);
            break;

        case 4 :
            // TODO

            forward_bacward_integration();
            break;
    }

    return 0;
}
