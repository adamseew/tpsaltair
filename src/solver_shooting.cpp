#include "../include/solver_shooting.hpp"
#include <stdexcept>
#include <iostream>
#include <limits>
#include <math.h>

using namespace std;
using namespace solver;

solver_shooting::solver_solution::solver_solution(double _delta, vectorn* _solution) {
    delta = _delta;
    solution = _solution;
}

solver_shooting::solver_shooting(second_derivative* __second_derivative, vectorn _start_y, vectorn _start_dy, vectorn _start_d2y, vectorn _final_y, double _t0, double _h) {
    _second_derivative = __second_derivative;
    _first_derivative = nullptr;
    start_y = _start_y;
    start_dy = _start_dy;
    start_d2y = _start_d2y;
    final_y = _final_y;
    t0 = _t0;   
    h = _h;
    set_default_functions();
}

solver_shooting::solver_shooting(first_derivative* __first_derivative, vectorn _start_y, vectorn _start_dy, vectorn _final_y, double _t0, double _h) {
    _first_derivative = __first_derivative;
    _second_derivative = nullptr;
    start_y = _start_y;
    start_dy = _start_dy;
    final_y = _final_y;
    t0 = _t0;   
    h = _h;
    set_default_functions();
}

bool solver_shooting::isbisided(vector<solver_solution*> solutions) {
    double first_solution_delta = solutions.at(0)->delta;

    for(auto const& solution: solutions)
        if (first_solution_delta * solution->delta < 0)
            return true;

    return false;
}

solver_shooting::solver_solution* solver_shooting::latest_other_side(vector<solver_solution*> solutions, double current_delta) { 
    solver_solution* _solver_solution;

    for (auto reverse_iterator = solutions.rbegin(); reverse_iterator != solutions.rend(); ++reverse_iterator) {
        _solver_solution = *reverse_iterator;
        if (current_delta * _solver_solution->delta < 0)
            return _solver_solution;
    }

    return nullptr;
}

void solver_shooting::set_default_functions(void) {
    set_function_adjust([](double target, double x, double dx)  { return (dx * target) / x; }   );
    set_function_cost(  [](double target, double x)             { return target - x; }          );
    set_function_stop(  [](double target, double x)             { return x > target; }          );
}

void solver_shooting::set_function_adjust(std::function<double(double, double, double)> function) { adjust_function = function; }

void solver_shooting::set_function_cost(std::function<double(double, double)> function) { cost_function = function; }

void solver_shooting::set_function_stop(std::function<bool(double, double)> function) { stop_function = function; }

vectorn solver_shooting::shoot(double epsilon) {

    double                  fixed_adjustment,
                            delta, 
                            t =                 0.0;

    int                     shoot_count =       0;
    string                  file_name;

    vectorn*                y =                 new vectorn(start_y.length());
    vectorn*                dy =                new vectorn(start_dy.length());
    vectorn*                d2y;

    if (_second_derivative != nullptr)
                            d2y =               new vectorn(start_d2y.length());

    integrator_rkn*         _integrator_rkn;
    integrator_rk4*         _integrator_rk4;

    y->inherit_flags(start_y);
    dy->inherit_flags(start_dy);
        
    if (_second_derivative != nullptr)
        d2y->inherit_flags(start_d2y);

    vector<solver_solution*> solutions;

    while (true) {

#pragma region ___S_H_O_O_T___

        if (shoot_count >= __SHOOTING_LIMIT) {
            throw std::length_error("shooting limit exceeded, the solution diverges");
        }

#ifdef ___D_E_B_U_G___
        file_name = shoot_count < 10 ? ".tmp_0" : ".tmp_";            
        file_name += to_string(shoot_count++) + ".dat";
#endif

        if (_second_derivative != nullptr)
            _integrator_rkn = new integrator_rkn(_second_derivative, t0, start_y, start_dy, h, start_d2y);
        else if (_first_derivative != nullptr)
            _integrator_rk4 = new integrator_rk4(_first_derivative, t0, start_y, h, start_dy);

#ifdef ___D_E_B_U_G___
        freopen(file_name.c_str(), "w", stdout);

        for (int i = 0; i < start_y.length()  ; i++)
            cout << start_y.get(i)   << "\t";
        for (int i = 0; i < start_dy.length() ; i++)
            cout << start_dy.get(i)  << "\t";

        if (_second_derivative != nullptr)
            for (int i = 0; i < start_d2y.length(); i++)
                cout << start_d2y.get(i) << "\t";

        cout << endl;
#endif

        do {
            if (_second_derivative != nullptr)
                _integrator_rkn->step(&t, y, dy, d2y);
            else if (_first_derivative != nullptr)
                _integrator_rk4->step(&t, y, dy);

#ifdef ___D_E_B_U_G___
            for (int i = 0; i < y->length()  ; i++)
                cout << y->get(i)   << "\t";
            for (int i = 0; i < dy->length() ; i++)
                cout << dy->get(i)  << "\t";

            if (_second_derivative != nullptr)
                for (int i = 0; i < d2y->length(); i++)
                    cout << d2y->get(i) << "\t";

            cout << endl;
#endif
#ifdef __APROXIMATION_TOLERANCE
        } while (!stop_function(final_y.get(vectorn_flags::stop_position), y->get(final_y.get_index(vectorn_flags::stop_position)) + __APROXIMATION_TOLERANCE));
#else
        } while (!stop_function(final_y.get(vectorn_flags::stop_position), y->get(final_y.get_index(vectorn_flags::stop_position))));
#endif

        delta = cost_function(final_y.get(vectorn_flags::cost_position), y->get(final_y.get_index(vectorn_flags::cost_position)));

        if (_second_derivative != nullptr)            
            solutions.push_back(new solver_solution(delta, start_dy.copy()));
        else if (_first_derivative != nullptr)
            solutions.push_back(new solver_solution(delta, start_y.copy()));

        if (fabs(delta) < epsilon)
            break;

        if (!isbisided(solutions)) {
            fixed_adjustment = delta < 0 ? (-1) * pow(epsilon, 2) : pow(epsilon, 2);

            /// adjustment of the parameters proportionaly to the desired goal; in this way, the solution below (or above) the goal is achieved on the next step. If the solution (abowe/below) is not achieved, and thus the solutions set is not besided, another lower (or greater) adjustment is done

            for (int i = 0; i < start_dy.length(); i++) {
                if (_second_derivative != nullptr)
                    start_dy.set(i, adjust_function(final_y.get(vectorn_flags::cost_position), y->get(vectorn_flags::cost_position),  start_dy.get(i)) + fixed_adjustment);
                else if (_first_derivative != nullptr)
                    start_y.set(i, adjust_function(final_y.get(vectorn_flags::cost_position), y->get(vectorn_flags::cost_position),  start_y.get(i)) + fixed_adjustment);
            }

        } else {

            /// modified bisection method implementation to adjust parameters and reach a solution

            for (int i = 0; i < solutions.back()->solution->length(); i++) {
                if (_second_derivative != nullptr)
                    start_dy.set(i, (solutions.back()->solution->get(i) + latest_other_side(solutions, delta)->solution->get(i)) / 2);
                else if (_first_derivative != nullptr)
                    start_y.set(i, (solutions.back()->solution->get(i) + latest_other_side(solutions, delta)->solution->get(i)) / 2);
            }
        }

        if (_second_derivative != nullptr)
            start_d2y = _second_derivative->get_value(t0, start_y, start_dy);
        else if (_first_derivative != nullptr)
            start_dy = _first_derivative->get_value(t0, start_y);

#pragma endregion ___S_H_O_O_T___

    }

    delete y;
    delete dy;
        
    if (_second_derivative != nullptr)
        delete d2y;
    
    if (_second_derivative != nullptr)
        return start_dy; 
    else if (_first_derivative != nullptr)
        return start_y;

    vector<solver_solution*>().swap(solutions);

    throw std::invalid_argument("the derivative object representing the problem to be solved has not been specified");

}
