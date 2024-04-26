#include "sd_sacd.h"

// Standard library includes.
#include <cmath>
#include <stdfloat>
#include <vector>

// =============================================================================
// Private Helpers BEGIN
// =============================================================================

SC_SACD_Vec3 operator+(const SC_SACD_Vec3 &a, const SC_SACD_Vec3 &b) {
  return SC_SACD_Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

std::vector<SC_SACD_Vec3> SC_SACD_Get_Box_Normals(
    const SC_SACD_Generic_Box *box) {
  std::vector<SC_SACD_Vec3> normals;

  normals.emplace_back(SC_SACD_Vec3{1.0F, 0.0F, 0.0F});
  normals.back() = SC_SACD_Vec3_Rotate(normals.back(), box->x_radians,
                                       box->y_radians, box->z_radians);

  normals.emplace_back(SC_SACD_Vec3{0.0F, 1.0F, 0.0F});
  normals.back() = SC_SACD_Vec3_Rotate(normals.back(), box->x_radians,
                                       box->y_radians, box->z_radians);

  normals.emplace_back(SC_SACD_Vec3{0.0F, 0.0F, 1.0F});
  normals.back() = SC_SACD_Vec3_Rotate(normals.back(), box->x_radians,
                                       box->y_radians, box->z_radians);

  // Not normalizing the normals on purpose for optimization. (No unit vectors.)
  return normals;
}

std::vector<SC_SACD_Vec3> SC_SACD_Get_Box_Corners(
    const SC_SACD_Generic_Box *box) {
  std::vector<SC_SACD_Vec3> corners;

  SC_SACD_Vec3 pos{box->x, box->y, box->z};

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{-box->width / 2.0F, -box->height / 2.0F,
                                       -box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{box->width / 2.0F, -box->height / 2.0F,
                                       -box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{-box->width / 2.0F, box->height / 2.0F,
                                       -box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{box->width / 2.0F, box->height / 2.0F,
                                       -box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{-box->width / 2.0F, -box->height / 2.0F,
                                       box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{box->width / 2.0F, -box->height / 2.0F,
                                       box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{-box->width / 2.0F, box->height / 2.0F,
                                       box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  corners.push_back(
      SC_SACD_Vec3_Rotate(SC_SACD_Vec3{box->width / 2.0F, box->height / 2.0F,
                                       box->depth / 2.0F},
                          box->x_radians, box->y_radians, box->z_radians) +
      pos);

  return corners;
}

struct SC_SACD_MinMax {
  float min, max;
};

std::vector<SC_SACD_MinMax> SC_SACD_Get_Box_MinMax(
    const SC_SACD_Generic_Box *box, const std::vector<SC_SACD_Vec3> &normals) {
  std::vector<SC_SACD_MinMax> minmaxes;

  std::vector<SC_SACD_Vec3> corners = SC_SACD_Get_Box_Corners(box);

  // Assuming normals are not normalized, and will not normalize anyway.
  // MinMax count should be same as normals count.
  for (const auto &normal : normals) {
    SC_SACD_MinMax minmax{INFINITY, -INFINITY};
    for (const auto &corner : corners) {
      float projected = SC_SACD_Dot_Product(corner, normal);
      if (projected > minmax.max) {
        minmax.max = projected;
      }
      if (projected < minmax.min) {
        minmax.min = projected;
      }
    }
    minmaxes.push_back(minmax);
  }

  return minmaxes;
}

// =============================================================================
// Private Helpers END
// =============================================================================

int SC_SACD_AABB_Box_Collision(const SC_SACD_AABB_Box *a,
                               const SC_SACD_AABB_Box *b) {
  float ax_min = a->x - a->width / 2.0F;
  float ax_max = a->x + a->width / 2.0F;
  float ay_min = a->y - a->height / 2.0F;
  float ay_max = a->y + a->height / 2.0F;
  float az_min = a->z - a->depth / 2.0F;
  float az_max = a->z + a->depth / 2.0F;

  float bx_min = b->x - b->width / 2.0F;
  float bx_max = b->x + b->width / 2.0F;
  float by_min = b->y - b->height / 2.0F;
  float by_max = b->y + b->height / 2.0F;
  float bz_min = b->z - b->depth / 2.0F;
  float bz_max = b->z + b->depth / 2.0F;

  return (ax_min < bx_max && ax_max > bx_min && ay_min < by_max &&
          ay_max > by_min && az_min < bz_max && az_max > bz_min)
             ? 1
             : 0;
}

int SC_SACD_Generic_Box_Collision(const SC_SACD_Generic_Box *a,
                                  const SC_SACD_Generic_Box *b) {
  // Get all normals.
  std::vector<SC_SACD_Vec3> normals = SC_SACD_Get_Box_Normals(a);
  for (const auto &normal : SC_SACD_Get_Box_Normals(b)) {
    normals.push_back(normal);
  }

  // Get all minmaxes.
  std::vector<SC_SACD_MinMax> minmaxes_a = SC_SACD_Get_Box_MinMax(a, normals);
  std::vector<SC_SACD_MinMax> minmaxes_b = SC_SACD_Get_Box_MinMax(b, normals);

  // Check minmaxes.
  for (unsigned int i = 0; i < normals.size(); ++i) {
    if (minmaxes_a[i].max < minmaxes_b[i].min ||
        minmaxes_b[i].max < minmaxes_a[i].min) {
      return 0;
    }
  }

  return 1;
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
