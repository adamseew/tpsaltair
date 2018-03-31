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
    /// Solves a boundary value problem given specific intial and desired final conditions. Gives the solution within specific epsilon
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
        /// @brief Initialize the problem using a second order ODE
        /// @param __second_derivative  object that represents the second derivative of a desired function
        /// @param _start_y             initial conditions, value of the dependent variable, i.e. space
        /// @param _start_dy            initial conditions, value of its derivative, i.e. velocity
        /// @param _start_d2y           initial conditions, value of its second derivative (use __second_derivative object to obtain the value, using dependent variable and its derivative), i.e. acceleration
        /// @param _final_y             boundary limit, value of the dependent variable to be reached at the end
        /// @param _t0                  value of the indipedent variable, i.e. time
        /// @param _h                   integration step, i.e. time interval
        solver_shooting(second_derivative* __second_derivative, vectorn _start_y, vectorn _start_dy, vectorn _start_d2y, vectorn _final_y, double _t0, double _h);

        /// @brief Initialize the problem using a second order ODE
        /// @param __second_derivative  object that represents the first derivative of a desired function
        /// @param _start_y             initial conditions, value of the dependent variable, i.e. space
        /// @param _start_dy            initial conditions, value of its derivative (use __first_derivative object to obtain the value, using dependent variable), i.e. velocity
        /// @param _final_y             boundary limit, value of the dependent variable to be reached at the end
        /// @param _t0                  value of the indipedent variable, i.e. time
        /// @param _h                   integration step, i.e. time interval
        solver_shooting(first_derivative* __first_derivative, vectorn _start_y, vectorn _start_dy, vectorn _final_y, double _t0, double _h);

        /// @brief Sets the adjustment function that is used to adjust parameters before the bisection method can be applied. Better the function represents the problem's evolution, faster a solution will be achieved
        /// @param function the function to be used
        void set_function_adjust(std::function<double (double, double, double)> function);

        /// @brief Sets the cost function that is evaluated each time has to be decided if it's the case to make another shoot or the precision achieved is enough
        /// @param function the function to be used
        void set_function_cost(std::function<double (double, double)> function);

        /// @brief Sets the stop function that is evaluated to know whenever another step of runge-kutta integrator has to be done
        ///        frustration free hint:
        ///        /---------------\ 
        ///        | FAQ: QUESTION : ?? difference between cost and stop function ??
        ///        |      ANSWER   : immagine a situation where the path of a missile has to be evaluated through this evaluator
        ///        |               | - cost function: is used to know if another shoot has to be done, thus it uses the x axis; it evaluates the distance between the target to be reached by the missile and the real achieved point
        ///        |               | - stop function: is used to know if another integration step has to be done to obtain the shoot, which solution is used by the cost function later; the function returns true/false and does not return a double! In the example it is represented by the height and we have to evaluate if the y is 0 (i.e. if the ground has been reached)
        ///        :---------------:
        ///        |      QUESTION : ok I now know the difference between cost/stop functions... But
        ///        |               | ?? how to say to the solver which one of the vectorn variables (i.e. which axis) refers to cost and which to stop function ??
        ///        |      ANSWER   : vectorn object is a variable set of doubles (i.e. the axis x,y,z or x,y) and each double can be flagged. To let know to the solver that a specific element of the set has to be evaluated with:
        ///        |               | - cost function: use set_flag(index, vectorn_flags::cost_position)
        ///        |               | - stop function: use set_flag(index, vectorn_flags::stop_position)
        ///        |               | in the above example, we will have start_position.set_flag(0, vectorn_flags::cost_position).
        ///        |               | please take on mind that if no couple of doubles of the vectorns passed to the constructor as dependent variables have both cost/stop flags, an exception will be thrown! So avoid it, you are advised
        ///        :---------------:
        ///        |      QUESTION : ?? have I always to specify those functions ??
        ///        |      ANSWER   : no! If none is specified, default functions are used. Those functions are just fine for the example problem above
        ///        \---------------/
        /// @param function the function to be used
        void set_function_stop(std::function<bool (double, double)> function);

        /// @brief Performs the shooting method on the problem using a modified bisection method to adjust parameters and a runge-kutta integrator of the order that is evaluated depending on the ODE's degree
        /// @param epsilon  specific error uto be used as reference when confronting the expected result with the obtained one
        ///                 /------------\ 
        ///                 |! BE AWARE !| 0.1, is reasonable, either 0.001 is (be patient and stay tuned). But you must either adjust the h in the integrator (the integration step)
        ///                 \------------/
        vectorn shoot(double epsilon);
    };
}

#endif