#include "integrator_rkn.hpp"
#include "integrator_rk4.hpp"
#include <functional>
#include <vector>

#ifndef SOLVER_SOLVER_SHOOTING_H
#define SOLVER_SOLVER_SHOOTING_H

#define __APROXIMATION_TOLERANCE    1e-04
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

    public:
        solver_shooting(second_derivative* __second_derivative, vectorn _start_y, vectorn _start_dy, vectorn _start_d2y, vectorn _final_y, double _t0, double _h);

        solver_shooting(first_derivative* __first_derivative, vectorn _start_y, vectorn _start_dy, vectorn _final_y, double _t0, double _h);

        void set_adjust_function(std::function<double (double, double, double)> _adjust_function);
        
        void set_cost_function(std::function<double (double, double)> _cost_function);

        void shoot(double epsilon);
    };
}

#endif