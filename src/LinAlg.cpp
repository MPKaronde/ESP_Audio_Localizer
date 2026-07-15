#include "LinAlg.h"

Vector Vector::operator+(const Vector& o) const { return {x+o.x, y+o.y, z+o.z}; }
Vector Vector::operator-(const Vector& o) const { return {x-o.x, y-o.y, z-o.z}; }
Vector Vector::operator*(int a) const { return {x*a, y*a, z*a}; }
Vector Vector::operator/(int a) const { return {x/a, y/a, z/a}; }

double magnitude(Vector a) {
  return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

Vector normalize(Vector a) {
  double m = magnitude(a);
  return {a.x/m, a.y/m, a.z/m};
}

double dot(Vector a, Vector b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector cross(Vector a, Vector b) {
  return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}

Matrix rotaion_matrix_around_axis(Vector axis, double theta) {
  double s = sin(theta), c = cos(theta);
  return Matrix(
    {axis.x*axis.x*(1-c)+c,            axis.x*axis.y*(1-c)-axis.z*s, axis.x*axis.z*(1-c)+axis.y*s},
    {axis.x*axis.y*(1-c)+axis.z*s,     axis.y*axis.y*(1-c)+c,        axis.y*axis.z*(1-c)-axis.x*s},
    {axis.x*axis.z*(1-c)-axis.y*s,     axis.y*axis.z*(1-c)+axis.x*s, axis.z*axis.z*(1-c)+c       }
  );
}

Vector matrix_multiplication(Matrix m, Vector a) {
  return {
    m.row0.x*a.x + m.row0.y*a.y + m.row0.z*a.z,
    m.row1.x*a.x + m.row1.y*a.y + m.row1.z*a.z,
    m.row2.x*a.x + m.row2.y*a.y + m.row2.z*a.z,
  };
}
