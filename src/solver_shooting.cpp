#include "../include/solver_shooting.hpp"
#include <iostream>
#include <math.h>

using namespace std;
using namespace solver;

#define SOLVER_SHOOTING_DEBUG 1

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
    set_cost_function([](double target, double x) { return target - x; });
    set_adjust_function([](double target, double x, double dx) { return (target * dx) / x; });
}

// TODO
solver_shooting::solver_shooting(first_derivative* __first_derivative, vectorn _start_y, vectorn _start_dy, vectorn _final_y, double _t0, double _h) {
    _first_derivative = __first_derivative;
    _second_derivative = nullptr;
    start_y = _start_y;
    start_dy = _start_dy;
    final_y = _final_y;
    t0 = _t0;   
    h = _h;
    set_cost_function([](double target, double x) { return target - x; });
    set_adjust_function([](double target, double x, double dx) { return (dx * target) / x; });
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

void solver_shooting::set_cost_function(std::function<double (double, double)> _cost_function) {
    cost_function = _cost_function;
}

void solver_shooting::set_adjust_function(std::function<double (double, double, double)> _adjust_function) {
    adjust_function = _adjust_function;
}

void solver_shooting::shoot(double epsilon) {
    double                  fixed_adjustment,
                            delta, 
                            t;
    int                     shoot_count =       0;
    string                  file_name;

    vectorn*                y;
    vectorn*                dy =                (vectorn*)malloc(sizeof(vectorn));
    vectorn*                d2y =               (vectorn*)malloc(sizeof(vectorn));

    vector<solver_solution*> solutions;

    while (true) {

        y = (vectorn*)malloc(sizeof(vectorn));

#ifdef SOLVER_SHOOTING_DEBUG
        file_name  = ".tmp_";
        file_name += to_string(shoot_count++);
        file_name += ".dat";
#endif

        integrator_rkn* _integrator_rkn = new integrator_rkn(_second_derivative, t0, start_y, start_dy, h, start_d2y);

#ifdef SOLVER_SHOOTING_DEBUG
        freopen(file_name.c_str(), "w", stdout);

        cout << start_y.get(0) << "\t" << start_y.get(1) << "\t"
             << t0 << "\t" << start_dy.get(0) << "\t" << start_dy.get(1) << "\t"
             << start_d2y.get(0) << "\t" << start_d2y.get(1) << endl;
#endif

        do {
            _integrator_rkn->step(&t, y, dy, d2y);
#ifdef SOLVER_SHOOTING_DEBUG
            cout << y->get(0) << "\t" << y->get(1) << "\t"
                 << t << "\t" << dy->get(0) << "\t" << dy->get(1) << "\t"
                 << d2y->get(0) << "\t" << d2y->get(1) << endl;
#endif
        } while (y->get(1) > final_y.get(1));

        delta = cost_function(final_y.get(0), y->get(0));

        solver_solution* _solver_solution = new solver_solution(delta, y);

        solutions.push_back(_solver_solution);

        // TODO il controllo va fatto su delta calcolato o su distance_function_output.push_back()???
        if (fabs(delta) < epsilon)
            break;
                
        if (!isbisided(solutions)) {
            fixed_adjustment = delta < 0 ? (-1) * pow(epsilon, 2) : pow(epsilon, 2);
            
            start_dy.set(0, adjust_function(final_y.get(0), y->get(0),  start_dy.get(0)) + fixed_adjustment);
            start_dy.set(1, adjust_function(final_y.get(0), y->get(0),  start_dy.get(1)) + fixed_adjustment);
        } else {
            start_dy.set(0, (solutions.back()->solution->get(0) + latest_other_side(solutions, delta)->solution->get(1)) / 2);
            start_dy.set(1, (solutions.back()->solution->get(1) + latest_other_side(solutions, delta)->solution->get(1)) / 2);
        } 

        start_d2y = _second_derivative->get_value(t0, start_y, start_dy);
    } 
}
