#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

#include <cstdlib>

// Matrix common functions

// Matrix addition
void mat_add(float *m1, float *m2, float *sol, int row, int column);

// Matrix subtraction
void mat_sub(float *m1, float *m2, float *sol, int row, int column);

// Matrix multiplication
void mat_mul(float *m1, float *m2, float *sol, int row1, int column1, int row2,
             int column2);

// Matrix transposition
void mat_tran(float *m1, float *sol, int row_original, int column_original);

// Matrix scalar multiplication
void mat_mul_const(float *m1, float c, float *sol, int row, int column);

// Matrix inversion (by Gaussian elimination)
void mat_inv(float *m, float *sol, int column, int row);

#endif // MATRIX_OPERATIONS_H
