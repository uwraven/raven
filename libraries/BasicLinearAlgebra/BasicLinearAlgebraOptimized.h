
#ifndef BLAD_H
#define BLAD_H

#include <BasicLinearAlgebra.h>

//TODO:: Optimize Jh and Ja

template<int dim, class ElemT> struct Diagonal
{
    mutable ElemT m[dim];

    // The only requirement on this class is that it implement the () operator like so:
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT dummy;

        // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
        if(row == col && row < dim)
            return m[row];
        else
            // Otherwise return a zero
            return (dummy = 0);
    }
};

#endif