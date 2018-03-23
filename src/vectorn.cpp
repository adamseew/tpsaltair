#include "../include/vectorn.hpp"

#include <iostream>
#include <cassert>
#include <math.h>

using namespace solver;

using std::vector;

vectorn::vectorn(int __length, double* values) {
    _length = __length;
    for (int i = 0; i < length(); i++)
        _vector.push_back(values[i]);
}

vectorn::vectorn(double value) : vectorn::vectorn(1, &value) { }

vectorn::vectorn(int __length) : vectorn::vectorn(__length, (double*)malloc(sizeof(double) * __length)) { }

vectorn::vectorn() : vectorn::vectorn(1, new double(0.0)) { }

const int vectorn::length() const { return _length; }

const double vectorn::get(int index) const { 
    assert(index >= 0 && index < length());
    return _vector.at(index); 
}

void vectorn::set(int index, double value) {
    assert(index >= 0 && index < length());
    _vector.at(index) = value; 
}

double vectorn::abs() {
    double sum = 0;
    for (int i = 0; i < length(); i++)
        sum += pow(get(i), 2);
    return sqrt(sum);
}

vectorn* vectorn::copy() { return new vectorn(length(), &_vector[0]); }

vectorn vectorn::operator+(const vectorn& _vectorn) const {
    double* values = (double*)malloc(sizeof(double) * length());
    for (int i = 0; i < length(); i++)
        values[i] = this->get(i) + _vectorn.get(i);
    return *new vectorn(length(), values);
}

vectorn vectorn::operator-(const vectorn& _vectorn) const { return this->operator+(_vectorn.operator*(-1.0)); }

vectorn vectorn::operator*(const double value) const {
    double* values = (double*)malloc(sizeof(double) * length());
    for (int i = 0; i < length(); i++)
        values[i] = value * this->get(i);
    return *new vectorn(length(), values);
}

vectorn vectorn::operator/(const double value) const { return this->operator*(1.0 / value); }

