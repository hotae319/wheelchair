#ifndef MATFUNC_H_
#define MATFUNC_H_

#include <Arduino.h>

float (*Transpose32(float matrix[][3]))[2];
float (*Transpose33(float matrix[][3]))[3];
float (*InverseMatrix2(float matrix[][2]))[2];
float (*InverseMatrix3(float matrix[][3]))[3];
float (*MatMultiply_AB(float a[][2], float b[]));
float (*MatMultiply_AtA(float a[][3]))[3];
float (*MatMultiply_AB_total(float a[][3], float b[]));
float (*MatMultiply32(float a[][2], float b[][3]))[3];

#endif










































































































































































































































































































































































































































































































































































































































































































