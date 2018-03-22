#ifndef SOLVER_VECTORN_H
#define SOLVER_VECTORN_H

namespace solver
{
    /// Gives a point in n-dimensional space. The class stores a sequence of real numbers and defines operators enabling to treat such sequences as real vectors
    class vectorn {

    private:
        int _length;
        double* vector;

    public:
        /// @brief Given a set of doubles and its length n, initializes a nx1 vector
        /// @param __length vector's length
        /// @param values   a set of doubles
        vectorn(int __length, double* values);

        /// @brief Given a double, initializes a 1x1 vector
        /// @param value    value to assign to 1x1 vector
        vectorn(double value);

        /// @brief Given an int n, initializes a nx1 void vector
        /// @param __length number of vector components
        vectorn(int __length);

        vectorn(const vectorn &_vectorn);

        /// @brief Default constructor
        vectorn();
        
        /// @brief Gives vector's size
        /// @return value of vectors size
        const int length() const;

        /// @brief Gives the value stored in a specific row in a nx1 vector
        /// @param index    index of the row
        /// @return         the value stored in the row index
        const double get(int index) const;

        /// @brief Stores a vaue in a specific row in a nx1 vector
        /// @param index    index of the row
        /// @param value    the value to be stored in the row index
        void set(int index, double value);

        /// @brief Gives the Euclidean norm of the vector
        /// @return the value of the Euclidean norm
        double abs();

        /// @brief Gives a copy of itself
        /// @return a pointer to a new object containing the copy of the current one
        vectorn* copy();

        vectorn operator+(const vectorn& _vectorn) const;
        vectorn operator-(const vectorn& _vectorn) const;
        vectorn operator*(const double value) const;
        vectorn operator/(const double value) const;

        friend vectorn operator*(const double lhs, const vectorn& rhs) { return rhs.operator*(lhs); }
        friend vectorn operator/(const double lhs, const vectorn& rhs) { return rhs.operator/(lhs); } 
    };  
}

#endif