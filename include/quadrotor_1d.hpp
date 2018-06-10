#include "integrator_rk4.hpp"
#include <vector>

#ifndef QUADROTOR_1D
#define QUADROTOR_1D

namespace solver 
{
    /// Gives a model of the acceleration of a thrown ball
    class quadrotor_1d : public first_derivative {
        
    private:
        double quadrotor_mass;                      
        double drag_coefficient; 
        double g;
        double moment_inertia;

        std::vector<double> u;

        int _time;
        int __time;
        int time_progress_unit;

    public:
        
        quadrotor_1d(double _quadrotor_mass, double _drag_coefficient, double _g, double _moment_inertia, std::vector<double> _u, int _time_progress_unit);

        quadrotor_1d(double _quadrotor_mass, double _drag_coefficient, double _g, double _moment_inertia, std::vector<double> _u);

        ~quadrotor_1d();
        
        virtual vectorn get_value(double t, vectorn q);
    };
}


#endif