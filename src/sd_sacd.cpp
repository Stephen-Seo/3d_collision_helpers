#include "sd_sacd.h"

// Standard library includes.
#include <cmath>

int SC_SACD_AABB_Box_Collision(const SC_SACD_AABB_Box *a,
                               const SC_SACD_AABB_Box *b) {
  return (a->x < (b->x + b->width) && (a->x + a->width) > b->x &&
          a->y < (b->y + b->height) && (a->y + a->height) > b->y &&
          a->z < (b->z + b->depth) && (a->z + a->depth) > b->z)
             ? 1
             : 0;
}

int SC_SACD_Generic_Box_Collision(const SC_SACD_Generic_Box *a,
                                  const SC_SACD_Generic_Box *b) {
  // TODO
  return 0;
}

int SC_SACD_AABB_Generic_Box_Collision(const SC_SACD_AABB_Box *a,
                                       const SC_SACD_Generic_Box *b) {
  SC_SACD_Generic_Box a_conv;

  a_conv.x = a->x;
  a_conv.y = a->y;
  a_conv.z = a->z;
  a_conv.width = a->width;
  a_conv.height = a->height;
  a_conv.depth = a->depth;
  a_conv.x_radians = 0.0F;
  a_conv.y_radians = 0.0F;
  a_conv.z_radians = 0.0F;
  return SC_SACD_Generic_Box_Collision(&a_conv, b);
}

float SC_SACD_Dot_Product(const SC_SACD_Vec3 a, const SC_SACD_Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

SC_SACD_Vec3 SC_SACD_Cross_Product(const SC_SACD_Vec3 a, const SC_SACD_Vec3 b) {
  return SC_SACD_Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x};
}

SC_SACD_Vec3 SC_SACD_Mat3_Vec3_Mult(const SC_SACD_Mat3 *mat,
                                    const SC_SACD_Vec3 vec) {
  return SC_SACD_Vec3{
      vec.x * mat->x0 + vec.y * mat->y0 + vec.z * mat->z0,
      vec.x * mat->x1 + vec.y * mat->y1 + vec.z * mat->z1,
      vec.x * mat->x2 + vec.y * mat->y2 + vec.z * mat->z2,
  };
}

SC_SACD_Vec3 SC_SACD_Vec3_Rotate(const SC_SACD_Vec3 vec, float x_axis,
                                 float y_axis, float z_axis) {
  /*
   * z_axis counter-clockwise affects x and y.
   * [  cos, -sin, 0 ]
   * [  sin,  cos, 0 ]
   * [  0,    0,   1 ]
   *
   * x_axis counter-clockwise affects y and z.
   * [ 1,    0,    0 ]
   * [ 0,  cos, -sin ]
   * [ 0,  sin,  cos ]
   *
   * y_axis counter-clockwise affects x and z.
   * [  cos, 0, sin ]
   * [    0, 1,   0 ]
   * [ -sin, 0, cos ]
   */

  SC_SACD_Mat3 mat;
  SC_SACD_Vec3 result;

  // About x_axis.
  mat.x0 = 1.0F;
  mat.y0 = 0.0F;
  mat.z0 = 0.0F;
  mat.x1 = 0.0F;
  mat.y1 = std::cos(x_axis);
  mat.z1 = -std::sin(x_axis);
  mat.x2 = 0.0F;
  mat.y2 = -mat.z1;
  mat.z2 = mat.y1;

  result = SC_SACD_Mat3_Vec3_Mult(&mat, vec);

  // About y_axis.
  mat.x0 = std::cos(y_axis);
  mat.y0 = 0.0F;
  mat.z0 = std::sin(y_axis);
  mat.x1 = 0.0F;
  mat.y1 = 1.0F;
  mat.z1 = 0.0F;
  mat.x2 = -mat.z0;
  mat.y2 = 0.0F;
  mat.z2 = mat.x0;

  result = SC_SACD_Mat3_Vec3_Mult(&mat, result);

  // About z_axis.
  mat.x0 = std::cos(z_axis);
  mat.y0 = -std::sin(z_axis);
  mat.z0 = 0.0F;
  mat.x1 = -mat.y0;
  mat.y1 = mat.x0;
  mat.z1 = 0.0F;
  mat.x2 = 0.0F;
  mat.y2 = 0.0F;
  mat.z2 = 1.0F;

  return SC_SACD_Mat3_Vec3_Mult(&mat, result);
}
