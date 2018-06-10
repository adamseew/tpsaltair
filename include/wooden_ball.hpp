#include "integrator_rkn.hpp"

#ifndef SOLVER_WOODEN_BALL_H
#define SOLVER_WOODEN_BALL_H

namespace solver 
{
    /// Gives a model of vertically thrown wooden ball
    class wooden_ball : public second_derivative {
        
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
        /// @brief Initialize an object that represents a wooden ball thrown vertically
        /// @param _ball_diameter    value of the ball diameter
        /// @param _ball_rho         value of the ball material density
        /// @param _drag_coefficient value of the drag coefficient
        /// @param _g                value of the acceleration due to gravity
        /// @param _air_rho          value of the air density
        wooden_ball(double _ball_diameter, double _ball_rho, double _drag_coefficient, double _g, double _air_rho);

        /// @brief Default destructor
        ~wooden_ball();

        /// @brief Gives the acceleration of the ball
        /// @param t    value of time
        /// @param y    value of ball height above the initial level (as 1x1 vector)
        /// @param v    value of ball velocity (as a 1x1 vector)
        /// @return     the acceleration (as a 1x1 vector)
        virtual vectorn get_value(double t, vectorn y, vectorn v);
    };
}

#endif