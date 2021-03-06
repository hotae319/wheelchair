#include "matfunc.h"
#include <Arduino.h>

float (*Transpose32(float matrix[][3]))[2]{
  static float TranMat[3][2];
  for(int i = 0; i<3; i++){
    for(int j = 0; j<2; j++){
      TranMat[i][j] = matrix[j][i];
    }
  }
  return TranMat;
}

float (*Transpose33(float matrix[][3]))[3]{
  static float TranMat[3][3];
  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      TranMat[i][j] = matrix[j][i];
    }
  }
  return TranMat;
}

float (*InverseMatrix2(float matrix[][2]))[2]{
  static float inv[2][2];
  float det = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
  inv[0][0] = matrix[1][1]/det;
  inv[0][1] = -matrix[0][1]/det;
  inv[1][0] = -matrix[1][0]/det;
  inv[1][1] = matrix[0][0]/det;
  return inv;
}
float (*InverseMatrix3(float matrix[][3]))[3]{
  static float inv[3][3];
  float (*Tmatrix)[3];
  float det = matrix[0][0]*matrix[1][1]*matrix[2][2] + matrix[1][0]*matrix[2][1]*matrix[0][2] + matrix[2][0]*matrix[0][1]*matrix[1][2]
  -matrix[0][0]*matrix[2][1]*matrix[1][2] - matrix[2][0]*matrix[1][1]*matrix[0][2] - matrix[1][0]*matrix[0][1]*matrix[2][2];
  Tmatrix = Transpose33(matrix);
  // inverse formula
  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){      
      inv[i][j] = 1.0/det*(Tmatrix[(i+2)%3][(j+2)%3]*Tmatrix[(i+1)%3][(j+1)%3] - Tmatrix[(i+1)%3][(j+2)%3]*Tmatrix[(i+2)%3][(j+1)%3]);
     }
  }
  return inv;
}

float (*MatMultiply_AB(float a[][2], float b[])){
  static float ResultMat[3];
  for(int i = 0; i<3; i++){
      ResultMat[i] = a[i][0]*b[1]+a[i][1]*b[0];  // b = [torque[0]; torque[1]], order change
  }
  return ResultMat;
}
float (*MatMultiply_AtA(float a[][3]))[3]{
  static float ResultMat[3][3];

  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){      
      ResultMat[i][j] = a[0][i]*a[0][j]+a[1][i]*a[1][j];      
    }
  }
  return ResultMat;
}
float (*MatMultiply_AB_total(float a[][3], float b[])){
  static float ResultMat[3];
  for(int i = 0; i<3; i++){
      ResultMat[i] = a[i][0]*b[0]+ a[i][1]*b[1]+a[i][2]*b[2];    
  }
  return ResultMat;
}
float (*MatMultiply32(float a[][2], float b[][3]))[3]{
  static float ResultMat[3][3];
  for(int i = 0 ; i<3; i++){
    for(int j =0 ; j<3; j++){
      ResultMat[i][j] = a[i][0]*b[0][j]+a[i][1]*b[1][j];      
    }
  }
  return ResultMat;
}







































































































































































































































































































































































































































