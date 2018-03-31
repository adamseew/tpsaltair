#include "integrator_rkn.hpp"
#include "integrator_rk4.hpp"
#include <functional>
#include <vector>

#ifndef SOLVER_SOLVER_SHOOTING_H
#define SOLVER_SOLVER_SHOOTING_H

/// set desired tolerance. The parameter may vary on specific needs; if undefined, a goal without any approximation tolerance is reached

//  #define __APROXIMATION_TOLERANCE    1e-04

/// when more that __SHOOTING_LIMIT shootings are performed, probably the solution cannot be reached (i.e. one of the doubles became nan). Thus an exception will be invoked

#define __SHOOTING_LIMIT            100

namespace solver 
{
    class solver_shooting {
    private:
        struct solver_solution {
            double delta;
            vectorn* solution;

            solver_solution(double _delta, vectorn* _solution);
        };

        std::function<double (double, double)>          cost_function; 
        std::function<bool (double, double)>            stop_function; 
        std::function<double (double, double, double)>  adjust_function;
        second_derivative*                              _second_derivative;
        first_derivative*                               _first_derivative;
        integrator_rkn*                                 _integrator_rkn;
        integrator_rk4*                                 _integrator_rk4;
        vectorn                                         start_y;
        vectorn                                         start_dy;
        vectorn                                         start_d2y;
        vectorn                                         final_y;
        double                                          t0;
        double                                          h;

        bool isbisided(std::vector< solver_solution* > solutions);

        solver_solution* latest_other_side(std::vector< solver_solution* > solutions, double current_delta);

        void set_default_functions(void);

    public:
        solver_shooting(second_derivative* __second_derivative, vectorn _start_y, vectorn _start_dy, vectorn _start_d2y, vectorn _final_y, double _t0, double _h);

        solver_shooting(first_derivative* __first_derivative, vectorn _start_y, vectorn _start_dy, vectorn _final_y, double _t0, double _h);

        void set_function_adjust(std::function<double (double, double, double)> function);
        void set_function_cost(std::function<double (double, double)> function);
        void set_function_stop(std::function<bool (double, double)> function);

        vectorn shoot(double epsilon);
    };
}

#endif