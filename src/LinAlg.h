#ifndef LINALG_H
#define LINALG_H

#include <math.h>

struct Vector {
  double x, y, z;
  Vector() : x(0), y(0), z(0) {}
  Vector(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
  Vector operator+(const Vector& o) const;
  Vector operator-(const Vector& o) const;
  Vector operator*(int a) const;
  Vector operator/(int a) const;
};

struct Matrix {
  Vector row0, row1, row2;
  Matrix(Vector r0, Vector r1, Vector r2) : row0(r0), row1(r1), row2(r2) {}
};

double magnitude(Vector a);
Vector normalize(Vector a);
double dot(Vector a, Vector b);
Vector cross(Vector a, Vector b);
Matrix rotaion_matrix_around_axis(Vector axis, double theta);
Vector matrix_multiplication(Matrix m, Vector a);

#endif
