#include "../include/vectorn.hpp"

#include <iostream>
#include <cassert>
#include <math.h>

using namespace solver;

using std::vector;
using std::swap;

vectorn::vectorn(int __length, double* values) {
    _length = __length;
    for (int i = 0; i < length(); i++) {
        _vector.push_back(values[i]);
        _flags.push_back(vectorn_flags::unflagged);
    }
}

vectorn::vectorn(double value) : vectorn::vectorn(1, &value) { }

vectorn::vectorn(int __length) { 
    _length = __length;
    for (int i = 0; i < length(); i++) {
        _vector.push_back(0.0);
        _flags.push_back(vectorn_flags::unflagged);
    }
}

vectorn::vectorn(const vectorn& _vectorn) {
    _length = _vectorn._length;
    _vector = _vectorn._vector;
    _flags = _vectorn._flags;
}

vectorn::vectorn() : vectorn::vectorn(1) { }

vectorn::~vectorn() {
    vector<double>().swap(_vector);
    vector<vectorn_flags>().swap(_flags);
}

const int vectorn::length() const { return _length; }

const double vectorn::get(int index) const { 
    assert(index >= 0 && index < length());
    return _vector.at(index); 
}

const double vectorn::get(vectorn_flags flag) const { 
    for (int i = 0; i < length(); i++)
        if (_flags.at(i) == flag)
            return _vector.at(i);

    throw std::invalid_argument("no position in vector match the flag");
}

const int vectorn::get_index(vectorn_flags flag) const { 
    for (int i = 0; i < length(); i++)
        if (_flags.at(i) == flag)
            return i;

    throw std::invalid_argument("no position in vector match the flag");
}

const vectorn_flags vectorn::get_flag(int index) const {
    assert(index >= 0 && index < length());
    return _flags.at(index);
}

void vectorn::set(int index, double value) {
    assert(index >= 0 && index < length());
    _vector.at(index) = value; 
}

void vectorn::set(int index, double value, vectorn_flags flag) {
    set(index, value);
    _flags.at(index) = flag; 
}

void vectorn::set_flag(int index, vectorn_flags flag) {
    assert(index >= 0 && index < length());
    _flags.at(index) = flag;
}

vectorn* vectorn::inherit_flags(vectorn _vectorn) {
    for (int i = 0; i < _vectorn.length(); i++) {
        if (i >= length())
            break;

        set_flag(i, _vectorn.get_flag(i));
    }

    return this;
}

double vectorn::abs() {
    double sum = 0;
    for (int i = 0; i < length(); i++)
        sum += pow(get(i), 2);
    return sqrt(sum);
}

vectorn* vectorn::copy() { return new vectorn(*this); }

vectorn& vectorn::operator=(const vectorn& _vectorn) {
    _length = _vectorn._length;
    _vector  = _vectorn._vector;
    _flags = _vectorn._flags;

    return *this;
}

vectorn vectorn::operator+(const vectorn& _vectorn) const {
    double* values = (double*)malloc(sizeof(double) * length());
    for (int i = 0; i < length(); i++)
        values[i] = this->get(i) + _vectorn.get(i);

    vectorn sum(length(), values);
    sum.inherit_flags(_vectorn);

    delete values;

    return sum;
}

vectorn vectorn::operator-(const vectorn& _vectorn) const { return this->operator+(_vectorn.operator*(-1.0)); }

vectorn vectorn::operator*(const double value) const {
    double* values = (double*)malloc(sizeof(double) * length());
    for (int i = 0; i < length(); i++)
        values[i] = value * this->get(i);

    vectorn product(length(), values);
    product.inherit_flags(*this);

    delete values;

    return product;
}

vectorn vectorn::operator/(const double value) const { return this->operator*(1.0 / value); }
