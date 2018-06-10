#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>

#ifndef SOLVER_SPLINER_H
#define SOLVER_SPLINER_H

namespace solver
{
    class band_matrix {
        
        private:
            std::vector< std::vector<double> > m_upper,
                                               m_lower;

        public:
            band_matrix();
            
            band_matrix(int dim, int n_u, int n_l);
            
            ~band_matrix();
            
            void resize(int dim, int n_u, int n_l);
            
            int dim() const;
            
            int num_upper() const;

            int num_lower() const;
            
            double &operator()(int i, int j);
            double  operator()(int i, int j) const;

            double& saved_diag(int i);
            double  saved_diag(int i) const;
            void lu_decompose();
            std::vector<double> r_solve(const std::vector<double>& b) const;
            std::vector<double> l_solve(const std::vector<double>& b) const;
            std::vector<double> lu_solve(const std::vector<double>& b, bool is_lu_decomposed=false);
    };

    // spline interpolation
    class spline {
        public:
            enum bd_type {
                first_deriv = 1,
                second_deriv = 2
            };

        private:

            // x, y coordinates of points
            std::vector<double> m_x,
                                m_y,

            // interpolation parameters
            // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
            // spline coefficients
                                m_a,
                                m_b,
                                m_c;

            // for left extrapol
            double              m_b0,
                                m_c0;

            bd_type             m_left,
                                m_right;

            double              m_left_value,
                                m_right_value;
            
            bool                m_force_linear_extrapolation;

        public:

            // set default boundary condition to be zero curvature at both ends
            spline() : m_left(second_deriv), m_right(second_deriv),
                       m_left_value(0.0),    m_right_value(0.0),
                       m_force_linear_extrapolation(false) { ; }
            
            // optional, but if called it has to come be before set_points()
            void set_boundary(bd_type left, double left_value, bd_type right, double right_value,  bool force_linear_extrapolation = false);
            
            void set_points(const std::vector<double>& x, const std::vector<double>& y, bool cubic_spline = true);

            double operator()(double x) const;
            
            double deriv(int order, double x) const;
    };
}

#endif