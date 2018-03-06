#include "vectorn.hpp"

#ifndef SOLVER_SECOND_DERIVATIVE_H
#define SOLVER_SECOND_DERIVATIVE_H

namespace solver 
{

    /// Abstract class that gives the value of second derivative from an ODE
    class second_derivative{

    public:
        virtual ~second_derivative() { };

        /// @brief Used to obtain the value of the second derivative of a desired function
        /// @param x    value of the independent variable, i.e. time
        /// @param y    value of the dependent variable
        /// @param dy   value of its derivative
        /// @return     the second derivative
        virtual vectorn get_value(double x, vectorn y, vectorn dy) = 0;
    };

    /// Gives one step of Nyström modification of the fourth-order Runge-Kutta method to sovle a second order differential equation with initial conditions  
    class integrator_rkn {
        
    private:
        double t0, h;
        second_derivative* _second_derivative;
        vectorn* y0;
        vectorn* dy0;
        vectorn* d2y0;

    public:
        /// @brief Initialize an integrator object with initial conditions
        /// @param __second_derivative  object that represents the second derivative of a desired function
        /// @param _t0                  value of the indipedent variable, i.e. time
        /// @param _y0                  value of the dependent variable
        /// @param _dy0                 value of its derivative, i.e. dy0/dt
        /// @param _h                   integration step, i.e. time interval
        /// @param _d2y0                value of second derivative
        integrator_rkn(second_derivative* __second_derivative, double _t0, vectorn _y0, vectorn _dy0, double _h, vectorn _d2y0);
        
        /// @brief Gives values (params are passed as pointers) after one step of Nyström modification of the fourth-order Runge-Kutta method
        /// @param t1   value of the indipedent variable, i.e. time after the end of the step
        /// @param y1   value of the dependent variable
        /// @param dy1  value of its derivative, i.e. dy1/dt
        /// @param d2y1 value of second derivative
        void step(double* t1, vectorn* y1, vectorn* dy1, vectorn* d2y1);
    };
}

#endif