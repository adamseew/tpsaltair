#include "integrator_rkn.hpp"

#ifndef SOLVER_CANNON_BALL_H
#define SOLVER_CANNON_BALL_H

namespace solver 
{
    /// Gives a model of the acceleration of a thrown ball
    class cannon_ball : public second_derivative {
        
    private:
        double ball_diameter;    // ball diameter       [m]
        double ball_rho;         // ball density        [kg/m^3]
        double drag_coefficient; // drag coefficient C_d
        double g;                // gravitational acceleration
                                 //                     [m/s^2]
        double air_rho;          // air density
        
        double ball_mass;        // ball mass           [kg]
        double ball_area;        // ball cross-section area
                                 //                     [m^2]

    public:
        /// @brief Initialize an object that represents a thrown ball
        /// @param _ball_diameter    value of the ball diameter
        /// @param _ball_rho         value of the ball material density
        /// @param _drag_coefficient value of the drag coefficient
        /// @param _g                value of the acceleration due to gravity
        /// @param _air_rho          value of the air density
        cannon_ball(double _ball_diameter, double _ball_rho, double _drag_coefficient, double _g, double _air_rho);

        ~cannon_ball();
        
        /// @brief Gives the acceleration of the ball
        /// @param t    value of time
        /// @param y    value of the vector of positions
        /// @param v    value of the vector of velocities
        /// @return     the vector of accelerations
        virtual vectorn get_value(double t, vectorn y, vectorn v);
    };
}


#endif