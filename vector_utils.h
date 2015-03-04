/* vector_utils.h
 *
 * Copyright (c)2015 Visesh Chari <visesh [at] research.iiit.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * Changelog:
 * Sun 22 Feb 2015 8:44:00 PM IST Created
 */

#ifndef VECTOR_UTILS_H
#define VECTOR_UTILS_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace std;

/*
* Some basic vector functions defined here. It would be better to use some matrix
* library to do these functions, but for now we will not bother.
* Ideally overhauling the code using a matrix library is very essential
*/
template <class T>
inline void vector_set_equal( T *vec_one, const T *vec_two, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] = vec_two[i];
    return;
}

template <class T>
inline void vector_subtract( T *vec_one, const T *vec_two, const T *vec_three, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] = vec_two[i] - vec_three[i];
    return;
}

template <class T>
inline void vector_add( T *vec_one, const T *vec_two, const T *vec_three, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] = vec_two[i] + vec_three[i];
    return;
}

template <class T>
inline void vector_plusequal( T *vec_one, const T *vec_two, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] += vec_two[i];
    return;
}

template <class T>
inline void vector_minusequal( T *vec_one, const T *vec_two, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] -= vec_two[i];
    return;
}
 
template <class T>
inline void vector_minusequal_scalar( T *vec_one, T scalar, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] -= scalar;
    return;
}

template <class T>
inline void vector_plusequal_scalar( T *vec_one, T scalar, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] += scalar;
    return;
}

template <class T>
inline void vector_mult_scalar( T *vec_one, T scalar, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] *= scalar;
    return;
}

template <class T>
inline void vector_scalar_mult_subtract( T *vec_one, T *vec_two, double *vec_three, T scalar, int dimension = 3 )
{
    for( int i = 0; i < dimension; i++ ) vec_one[i] = ( vec_two[i] - T(vec_three[i]) ) * scalar ;
}

#endif /* ifndef VECTOR_UTILS_H */


