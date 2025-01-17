#include "sc_sacd.h"

// Standard library includes.
#include <array>
#include <cmath>
#include <span>
#include <vector>

// =============================================================================
// Private Helpers BEGIN
// =============================================================================

constexpr float INV_SQRT2 = 0.70710678118654752440F;

SC_SACD_Vec3 operator+(SC_SACD_Vec3 a, SC_SACD_Vec3 b) {
  return SC_SACD_Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

SC_SACD_Vec3 operator-(SC_SACD_Vec3 a, SC_SACD_Vec3 b) {
  return SC_SACD_Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

SC_SACD_Vec3 operator*(SC_SACD_Vec3 a, float scalar) {
  return SC_SACD_Vec3{a.x * scalar, a.y * scalar, a.z * scalar};
}

SC_SACD_Vec3 operator/(SC_SACD_Vec3 a, float scalar) {
  return SC_SACD_Vec3{a.x / scalar, a.y / scalar, a.z / scalar};
}

SC_SACD_Mat4 operator+(SC_SACD_Mat4 a, SC_SACD_Mat4 b) {
  return SC_SACD_Mat4{a.x0 + b.x0, a.x1 + b.x1, a.x2 + b.x2, a.x3 + b.x3,
                      a.y0 + b.y0, a.y1 + b.y1, a.y2 + b.y2, a.y3 + b.y3,
                      a.z0 + b.z0, a.z1 + b.z1, a.z2 + b.z2, a.z3 + b.z3,
                      a.w0 + b.w0, a.w1 + b.w1, a.w2 + b.w2, a.w3 + b.w3};
}

SC_SACD_Mat4 operator*(SC_SACD_Mat4 a, SC_SACD_Mat4 b) {
  SC_SACD_Mat4 mat;

  mat.x0 = b.x0 * a.x0 + b.y0 * a.x1 + b.z0 * a.x2 + b.w0 * a.x3;
  mat.y0 = b.x0 * a.y0 + b.y0 * a.y1 + b.z0 * a.y2 + b.w0 * a.y3;
  mat.z0 = b.x0 * a.z0 + b.y0 * a.z1 + b.z0 * a.z2 + b.w0 * a.z3;
  mat.w0 = b.x0 * a.w0 + b.y0 * a.w1 + b.z0 * a.w2 + b.w0 * a.w3;

  mat.x1 = b.x1 * a.x0 + b.y1 * a.x1 + b.z1 * a.x2 + b.w1 * a.x3;
  mat.y1 = b.x1 * a.y0 + b.y1 * a.y1 + b.z1 * a.y2 + b.w1 * a.y3;
  mat.z1 = b.x1 * a.z0 + b.y1 * a.z1 + b.z1 * a.z2 + b.w1 * a.z3;
  mat.w1 = b.x1 * a.w0 + b.y1 * a.w1 + b.z1 * a.w2 + b.w1 * a.w3;

  mat.x2 = b.x2 * a.x0 + b.y2 * a.x1 + b.z2 * a.x2 + b.w2 * a.x3;
  mat.y2 = b.x2 * a.y0 + b.y2 * a.y1 + b.z2 * a.y2 + b.w2 * a.y3;
  mat.z2 = b.x2 * a.z0 + b.y2 * a.z1 + b.z2 * a.z2 + b.w2 * a.z3;
  mat.w2 = b.x2 * a.w0 + b.y2 * a.w1 + b.z2 * a.w2 + b.w2 * a.w3;

  mat.x3 = b.x3 * a.x0 + b.y3 * a.x1 + b.z3 * a.x2 + b.w3 * a.x3;
  mat.y3 = b.x3 * a.y0 + b.y3 * a.y1 + b.z3 * a.y2 + b.w3 * a.y3;
  mat.z3 = b.x3 * a.z0 + b.y3 * a.z1 + b.z3 * a.z2 + b.w3 * a.z3;
  mat.w3 = b.x3 * a.w0 + b.y3 * a.w1 + b.z3 * a.w2 + b.w3 * a.w3;

  return mat;
}

SC_SACD_Vec3 operator*(SC_SACD_Mat4 mat, SC_SACD_Vec3 vec) {
  return SC_SACD_Vec3{
      vec.x * mat.x0 + vec.y * mat.x1 + vec.z * mat.x2 + mat.x3,
      vec.x * mat.y0 + vec.y * mat.y1 + vec.z * mat.y2 + mat.y3,
      vec.x * mat.z0 + vec.y * mat.z1 + vec.z * mat.z2 + mat.z3};
}

SC_SACD_Mat3 operator+(SC_SACD_Mat3 a, SC_SACD_Mat3 b) {
  return SC_SACD_Mat3{a.x0 + b.x0, a.x1 + b.x1, a.x2 + b.x2,
                      a.y0 + b.y0, a.y1 + b.y1, a.y2 + b.y2,
                      a.z0 + b.z0, a.z1 + b.z1, a.z2 + b.z2};
}

SC_SACD_Mat3 operator*(SC_SACD_Mat3 a, SC_SACD_Mat3 b) {
  return SC_SACD_Mat3{// x0
                      a.x0 * b.x0 + a.x1 * b.y0 + a.x2 * b.z0,
                      // x1
                      a.x0 * b.x1 + a.x1 * b.y1 + a.x2 * b.z1,
                      // x2
                      a.x0 * b.x2 + a.x1 * b.y2 + a.x2 * b.z2,
                      // y0
                      a.y0 * b.x0 + a.y1 * b.y0 + a.y2 * b.z0,
                      // y1
                      a.y0 * b.x1 + a.y1 * b.y1 + a.y2 * b.z1,
                      // y2
                      a.y0 * b.x2 + a.y1 * b.y2 + a.y2 * b.z2,
                      // z0
                      a.z0 * b.x0 + a.z1 * b.y0 + a.z2 * b.z0,
                      // z1
                      a.z0 * b.x1 + a.z1 * b.y1 + a.z2 * b.z1,
                      // z2
                      a.z0 * b.x2 + a.z1 * b.y2 + a.z2 * b.z2};
}

SC_SACD_Vec3 operator*(SC_SACD_Mat3 mat3, SC_SACD_Vec3 vec) {
  return SC_SACD_Vec3{
      mat3.x0 * vec.x + mat3.x1 * vec.y + mat3.x2 * vec.z,
      mat3.y0 * vec.x + mat3.y1 * vec.y + mat3.y2 * vec.z,
      mat3.z0 * vec.x + mat3.z1 * vec.y + mat3.z2 * vec.z,
  };
}

SC_SACD_Mat3 operator*(SC_SACD_Mat3 mat3, float scalar) {
  return SC_SACD_Mat3{mat3.x0 * scalar, mat3.x1 * scalar, mat3.x2 * scalar,
                      mat3.y0 * scalar, mat3.y1 * scalar, mat3.y2 * scalar,
                      mat3.z0 * scalar, mat3.z1 * scalar, mat3.z2 * scalar};
}

SC_SACD_Mat3 UHat_Mat3(float x, float y, float z) {
  return SC_SACD_Mat3{0.0F, -z, y, z, 0.0F, -x, -y, x, 0.0F};
}

std::array<SC_SACD_Vec3, 3> SC_SACD_Get_Box_Normals(SC_SACD_Generic_Box box) {
  SC_SACD_Vec3 a, b, c;

  // Facing positive x-axis.
  a.x = 0.0F;
  a.y = 0.0F;
  a.z = 0.0F;

  b.x = 0.0F;
  b.y = 1.0F;
  b.z = 0.0F;

  c.x = 0.0F;
  c.y = 0.0F;
  c.z = 1.0F;

  a = box.transform * a;
  b = box.transform * b;
  c = box.transform * c;

  b = b - a;
  c = c - a;

  auto normal_x = SC_SACD_Cross_Product(b, c);

  // Facing positive y-axis.
  a.x = 0.0F;
  a.y = 0.0F;
  a.z = 0.0F;

  b.x = 1.0F;
  b.y = 0.0F;
  b.z = 0.0F;

  c.x = 0.0F;
  c.y = 0.0F;
  c.z = -1.0F;

  a = box.transform * a;
  b = box.transform * b;
  c = box.transform * c;

  b = b - a;
  c = c - a;

  auto normal_y = SC_SACD_Cross_Product(b, c);

  // Facing positive z-axis.
  a.x = 0.0F;
  a.y = 0.0F;
  a.z = 0.0F;

  b.x = 0.0F;
  b.y = 1.0F;
  b.z = 0.0F;

  c.x = -1.0F;
  c.y = 0.0F;
  c.z = 0.0F;

  a = box.transform * a;
  b = box.transform * b;
  c = box.transform * c;

  b = b - a;
  c = c - a;

  auto normal_z = SC_SACD_Cross_Product(b, c);

  return {normal_x, normal_y, normal_z};
}

std::array<SC_SACD_Vec3, 3> SC_SACD_Get_Box_Normals_Normalized(
    SC_SACD_Generic_Box box) {
  auto normals = SC_SACD_Get_Box_Normals(box);

  for (auto &normal : normals) {
    normal = normal / std::sqrt(SC_SACD_Dot_Product(normal, normal));
  }

  return normals;
}

std::array<SC_SACD_Vec3, 8> SC_SACD_Get_Box_Corners(SC_SACD_Generic_Box box) {
  SC_SACD_Vec3 corner_0 =
      box.transform *
      SC_SACD_Vec3{-box.width / 2.0F, -box.height / 2.0F, -box.depth / 2.0F};
  corner_0.x += box.x;
  corner_0.y += box.y;
  corner_0.z += box.z;

  SC_SACD_Vec3 corner_1 =
      box.transform *
      SC_SACD_Vec3{box.width / 2.0F, -box.height / 2.0F, -box.depth / 2.0F};
  corner_1.x += box.x;
  corner_1.y += box.y;
  corner_1.z += box.z;

  SC_SACD_Vec3 corner_2 =
      box.transform *
      SC_SACD_Vec3{-box.width / 2.0F, box.height / 2.0F, -box.depth / 2.0F};
  corner_2.x += box.x;
  corner_2.y += box.y;
  corner_2.z += box.z;

  SC_SACD_Vec3 corner_3 =
      box.transform *
      SC_SACD_Vec3{box.width / 2.0F, box.height / 2.0F, -box.depth / 2.0F};
  corner_3.x += box.x;
  corner_3.y += box.y;
  corner_3.z += box.z;

  SC_SACD_Vec3 corner_4 =
      box.transform *
      SC_SACD_Vec3{-box.width / 2.0F, -box.height / 2.0F, box.depth / 2.0F};
  corner_4.x += box.x;
  corner_4.y += box.y;
  corner_4.z += box.z;

  SC_SACD_Vec3 corner_5 =
      box.transform *
      SC_SACD_Vec3{box.width / 2.0F, -box.height / 2.0F, box.depth / 2.0F};
  corner_5.x += box.x;
  corner_5.y += box.y;
  corner_5.z += box.z;

  SC_SACD_Vec3 corner_6 =
      box.transform *
      SC_SACD_Vec3{-box.width / 2.0F, box.height / 2.0F, box.depth / 2.0F};
  corner_6.x += box.x;
  corner_6.y += box.y;
  corner_6.z += box.z;

  SC_SACD_Vec3 corner_7 =
      box.transform *
      SC_SACD_Vec3{box.width / 2.0F, box.height / 2.0F, box.depth / 2.0F};
  corner_7.x += box.x;
  corner_7.y += box.y;
  corner_7.z += box.z;

  return {
      corner_0, corner_1, corner_2, corner_3,
      corner_4, corner_5, corner_6, corner_7,
  };
}

struct SC_SACD_MinMax {
  float min, max;
};

std::vector<SC_SACD_MinMax> SC_SACD_Get_Box_MinMax(
    SC_SACD_Generic_Box box, const std::span<SC_SACD_Vec3> normals) {
  std::vector<SC_SACD_MinMax> minmaxes;

  auto corners = SC_SACD_Get_Box_Corners(box);

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

SC_SACD_Generic_Box SC_SACD_Generic_Box_Default() {
  return {
      0.0F, 0.0F, 0.0F, 2.0F, 2.0F, 2.0F, SC_SACD_Mat4_Identity(),
  };
}

int SC_SACD_AABB_Box_Collision(SC_SACD_AABB_Box a, SC_SACD_AABB_Box b) {
  float ax_min = a.x - a.width / 2.0F;
  float ax_max = a.x + a.width / 2.0F;
  float ay_min = a.y - a.height / 2.0F;
  float ay_max = a.y + a.height / 2.0F;
  float az_min = a.z - a.depth / 2.0F;
  float az_max = a.z + a.depth / 2.0F;

  float bx_min = b.x - b.width / 2.0F;
  float bx_max = b.x + b.width / 2.0F;
  float by_min = b.y - b.height / 2.0F;
  float by_max = b.y + b.height / 2.0F;
  float bz_min = b.z - b.depth / 2.0F;
  float bz_max = b.z + b.depth / 2.0F;

  return (ax_min < bx_max && ax_max > bx_min && ay_min < by_max &&
          ay_max > by_min && az_min < bz_max && az_max > bz_min)
             ? 1
             : 0;
}

int SC_SACD_Generic_Box_Collision(SC_SACD_Generic_Box a,
                                  SC_SACD_Generic_Box b) {
  // Get all normals.
  std::vector<SC_SACD_Vec3> normals;
  {
    for (const auto &normal : SC_SACD_Get_Box_Normals(a)) {
      normals.push_back(normal);
    }
    for (const auto &normal : SC_SACD_Get_Box_Normals(b)) {
      normals.push_back(normal);
    }
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

int SC_SACD_AABB_Generic_Box_Collision(SC_SACD_AABB_Box a,
                                       SC_SACD_Generic_Box b) {
  SC_SACD_Generic_Box a_conv;

  a_conv.x = a.x;
  a_conv.y = a.y;
  a_conv.z = a.z;
  a_conv.width = a.width;
  a_conv.height = a.height;
  a_conv.depth = a.depth;
  a_conv.transform = SC_SACD_Mat4_Identity();
  return SC_SACD_Generic_Box_Collision(a_conv, b);
}

int SC_SACD_Sphere_Collision(SC_SACD_Sphere a, SC_SACD_Sphere b) {
  SC_SACD_Vec3 vec{a.x - b.x, a.y - b.y, a.z - b.z};

  return (a.radius + b.radius) > std::sqrt(SC_SACD_Dot_Product(vec, vec)) ? 1
                                                                          : 0;
}

int SC_SACD_Sphere_AABB_Box_Collision(SC_SACD_Sphere sphere,
                                      SC_SACD_AABB_Box box) {
  SC_SACD_Vec3 box_min{
      box.x - box.width / 2.0F,
      box.y - box.height / 2.0F,
      box.z - box.depth / 2.0F,
  };
  SC_SACD_Vec3 box_max{
      box.x + box.width / 2.0F,
      box.y + box.height / 2.0F,
      box.z + box.depth / 2.0F,
  };

  SC_SACD_Vec3 clamped{std::max(box_min.x, std::min(sphere.x, box_max.x)),
                       std::max(box_min.y, std::min(sphere.y, box_max.y)),
                       std::max(box_min.z, std::min(sphere.z, box_max.z))};

  SC_SACD_Vec3 diff = clamped - SC_SACD_Vec3{sphere.x, sphere.y, sphere.z};

  float dist = std::sqrt(SC_SACD_Dot_Product(diff, diff));

  return dist < sphere.radius;
}

int SC_SACD_Sphere_Box_Collision(SC_SACD_Sphere sphere,
                                 SC_SACD_Generic_Box box) {
  // Adapted from Generic_Box/Generic_Box collision.

  // First check plane where normal = box_pos - sphere_pos.

  SC_SACD_Vec3 sphere_pos{sphere.x, sphere.y, sphere.z};
  std::array<SC_SACD_Vec3, 1> sphere_box_normal = {
      SC_SACD_Vec3{box.x, box.y, box.z} - sphere_pos};
  if (sphere_box_normal[0].x < 0.0001F && sphere_box_normal[0].x > -0.0001F &&
      sphere_box_normal[0].y < 0.0001F && sphere_box_normal[0].y > -0.0001F &&
      sphere_box_normal[0].z < 0.0001F && sphere_box_normal[0].z > -0.0001F) {
    // Sphere center is box center.
    return 1;
  }
  sphere_box_normal[0] =
      sphere_box_normal[0] / std::sqrt(SC_SACD_Dot_Product(
                                 sphere_box_normal[0], sphere_box_normal[0]));

  std::vector<SC_SACD_MinMax> box_minmaxes =
      SC_SACD_Get_Box_MinMax(box, sphere_box_normal);

  float projected_0 = SC_SACD_Dot_Product(
      sphere_box_normal[0], sphere_pos + sphere_box_normal[0] * sphere.radius);
  float projected_1 = SC_SACD_Dot_Product(
      sphere_box_normal[0], sphere_pos - sphere_box_normal[0] * sphere.radius);
  if (projected_0 < projected_1) {
    if (box_minmaxes[0].max < projected_0 ||
        box_minmaxes[0].min > projected_1) {
      return 0;
    }
  } else if (box_minmaxes[0].max < projected_1 ||
             box_minmaxes[0].min > projected_0) {
    return 0;
  }

  // Next check the planes for the 3 normals of the box.

  auto box_normals = SC_SACD_Get_Box_Normals(box);
  box_minmaxes = SC_SACD_Get_Box_MinMax(box, box_normals);
  for (unsigned int i = 0; i < box_normals.size(); ++i) {
    projected_0 = SC_SACD_Dot_Product(
        box_normals[i], sphere_pos + box_normals[i] * sphere.radius);
    projected_1 = SC_SACD_Dot_Product(
        box_normals[i], sphere_pos - box_normals[i] * sphere.radius);
    if (projected_0 < projected_1) {
      if (box_minmaxes[i].max < projected_0 ||
          box_minmaxes[i].min > projected_1) {
        return 0;
      }
    } else if (box_minmaxes[i].max < projected_1 ||
               box_minmaxes[i].min > projected_0) {
      return 0;
    }
  }

  return 1;
}

float SC_SACD_Dot_Product(SC_SACD_Vec3 a, SC_SACD_Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

SC_SACD_Vec3 SC_SACD_Cross_Product(SC_SACD_Vec3 a, SC_SACD_Vec3 b) {
  return SC_SACD_Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x};
}

SC_SACD_Mat4 SC_SACD_Mat4_Identity(void) {
  return SC_SACD_Mat4{1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                      0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};
}

SC_SACD_Mat4 SC_SACD_Mat4_Sum(SC_SACD_Mat4 a, SC_SACD_Mat4 b) { return a + b; }

SC_SACD_Mat4 SC_SACD_Mat4_Mult(SC_SACD_Mat4 a, SC_SACD_Mat4 b) { return a * b; }

SC_SACD_Vec3 SC_SACD_Mat4_Vec3_Mult(SC_SACD_Mat4 mat, SC_SACD_Vec3 vec) {
  return mat * vec;
}

SC_SACD_Vec3 SC_SACD_Vec3_Rotate(SC_SACD_Vec3 vec, float x_axis, float y_axis,
                                 float z_axis) {
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

  SC_SACD_Mat4 mat;
  SC_SACD_Vec3 result;

  // About x_axis.
  mat = SC_SACD_Rotation_Mat4_XAxis(x_axis);

  result = SC_SACD_Mat4_Vec3_Mult(mat, vec);

  // About y_axis.
  mat = SC_SACD_Rotation_Mat4_YAxis(y_axis);

  result = SC_SACD_Mat4_Vec3_Mult(mat, result);

  // About z_axis.
  mat = SC_SACD_Rotation_Mat4_ZAxis(z_axis);

  return SC_SACD_Mat4_Vec3_Mult(mat, result);
}

SC_SACD_Mat4 SC_SACD_Rotation_Mat4_XAxis(float x_radians) {
  SC_SACD_Mat4 mat;

  mat.x0 = 1.0F;
  mat.x1 = 0.0F;
  mat.x2 = 0.0F;
  mat.x3 = 0.0F;

  mat.y0 = 0.0F;
  mat.y1 = std::cos(x_radians);
  mat.y2 = -std::sin(x_radians);
  mat.y3 = 0.0F;

  mat.z0 = 0.0F;
  mat.z1 = -mat.y2;
  mat.z2 = mat.y1;
  mat.z3 = 0.0F;

  mat.w0 = 0.0F;
  mat.w1 = 0.0F;
  mat.w2 = 0.0F;
  mat.w3 = 1.0F;

  return mat;
}

SC_SACD_Mat4 SC_SACD_Rotation_Mat4_YAxis(float y_radians) {
  SC_SACD_Mat4 mat;

  mat.x0 = std::cos(y_radians);
  mat.x1 = 0.0F;
  mat.x2 = std::sin(y_radians);
  mat.x3 = 0.0F;

  mat.y0 = 0.0F;
  mat.y1 = 1.0F;
  mat.y2 = 0.0F;
  mat.y3 = 0.0F;

  mat.z0 = -mat.x2;
  mat.z1 = 0.0F;
  mat.z2 = mat.x0;
  mat.z3 = 0.0F;

  mat.w0 = 0.0F;
  mat.w1 = 0.0F;
  mat.w2 = 0.0F;
  mat.w3 = 1.0F;

  return mat;
}

SC_SACD_Mat4 SC_SACD_Rotation_Mat4_ZAxis(float z_radians) {
  SC_SACD_Mat4 mat;

  mat.x0 = std::cos(z_radians);
  mat.x1 = -std::sin(z_radians);
  mat.x2 = 0.0F;
  mat.x3 = 0.0F;

  mat.y0 = -mat.x1;
  mat.y1 = mat.x0;
  mat.y2 = 0.0F;
  mat.y3 = 0.0F;

  mat.z0 = 0.0F;
  mat.z1 = 0.0F;
  mat.z2 = 1.0F;
  mat.z3 = 0.0F;

  mat.w0 = 0.0F;
  mat.w1 = 0.0F;
  mat.w2 = 0.0F;
  mat.w3 = 1.0F;

  return mat;
}

SC_SACD_Mat4 SC_SACD_Translate_Mat4(float x, float y, float z) {
  return SC_SACD_Mat4{1.0F, 0.0F, 0.0F, x, 0.0F, 1.0F, 0.0F, y,
                      0.0F, 0.0F, 1.0F, z, 0.0F, 0.0F, 0.0F, 1.0F};
}

SC_SACD_Mat4 SC_SACD_Scale_Mat4(float x, float y, float z) {
  return SC_SACD_Mat4{x,    0.0F, 0.0F, 0.0F, 0.0F, y,    0.0F, 0.0F,
                      0.0F, 0.0F, z,    0.0F, 0.0F, 0.0F, 0.0F, 1.0F};
}

SC_SACD_Vec3 SC_SACD_Closest_Point_Dir_Normalized(SC_SACD_Vec3 pos,
                                                  SC_SACD_Vec3 dir,
                                                  SC_SACD_Vec3 point) {
  float alpha = SC_SACD_Dot_Product(dir, point) - SC_SACD_Dot_Product(dir, pos);
  return pos + dir * alpha;
}

SC_SACD_Vec3 SC_SACD_Closest_Point(SC_SACD_Vec3 pos, SC_SACD_Vec3 dir,
                                   SC_SACD_Vec3 point) {
  float alpha =
      (SC_SACD_Dot_Product(dir, point) - SC_SACD_Dot_Product(dir, pos)) /
      SC_SACD_Dot_Product(dir, dir);
  return pos + dir * alpha;
}

float SC_SACD_Vec3_Length(SC_SACD_Vec3 vec) {
  return std::sqrt(SC_SACD_Dot_Product(vec, vec));
}

SC_SACD_AABB_Box SC_SACD_Sphere_To_AABB(SC_SACD_Sphere s) {
  SC_SACD_AABB_Box aabb{};

  aabb.x = s.x;
  aabb.y = s.y;
  aabb.z = s.z;

  aabb.width = s.radius * 2.0F;
  aabb.height = s.radius * 2.0F;
  aabb.depth = s.radius * 2.0F;

  return aabb;
}

SC_SACD_AABB_Box SC_SACD_Generic_Box_To_AABB(SC_SACD_Generic_Box s) {
  SC_SACD_AABB_Box aabb{};

  auto corners = SC_SACD_Get_Box_Corners(s);

  SC_SACD_Vec3 min{INFINITY, INFINITY, INFINITY};
  SC_SACD_Vec3 max{-INFINITY, -INFINITY, -INFINITY};

  for (const auto &corner : corners) {
    if (corner.x < min.x) {
      min.x = corner.x;
    }
    if (corner.y < min.y) {
      min.y = corner.y;
    }
    if (corner.z < min.z) {
      min.z = corner.z;
    }

    if (corner.x > max.x) {
      max.x = corner.x;
    }
    if (corner.y > max.y) {
      max.y = corner.y;
    }
    if (corner.z > max.z) {
      max.z = corner.z;
    }
  }

  aabb.width = max.x - min.x;
  aabb.height = max.y - min.y;
  aabb.depth = max.z - min.z;

  aabb.x = min.x + aabb.width / 2.0F;
  aabb.y = min.y + aabb.height / 2.0F;
  aabb.z = min.z + aabb.depth / 2.0F;

  return aabb;
}

SC_SACD_AABB_Box SC_SACD_AABB_Combine(SC_SACD_AABB_Box a, SC_SACD_AABB_Box b) {
  SC_SACD_Vec3 min, max;

  // Populate min values.

  float temp_a = a.x - a.width / 2.0F;
  float temp_b = b.x - b.width / 2.0F;

  min.x = temp_a < temp_b ? temp_a : temp_b;

  temp_a = a.y - a.height / 2.0F;
  temp_b = b.y - b.height / 2.0F;

  min.y = temp_a < temp_b ? temp_a : temp_b;

  temp_a = a.z - a.depth / 2.0F;
  temp_b = b.z - b.depth / 2.0F;

  min.z = temp_a < temp_b ? temp_a : temp_b;

  // Populate max values.

  temp_a = a.x + a.width / 2.0F;
  temp_b = b.x + b.width / 2.0F;

  max.x = temp_a > temp_b ? temp_a : temp_b;

  temp_a = a.y + a.height / 2.0F;
  temp_b = b.y + b.height / 2.0F;

  max.y = temp_a > temp_b ? temp_a : temp_b;

  temp_a = a.z + a.depth / 2.0F;
  temp_b = b.z + b.depth / 2.0F;

  max.z = temp_a > temp_b ? temp_a : temp_b;

  // Populate the result.

  temp_a = max.x - min.x;
  temp_b = max.y - min.y;
  float temp_c = max.z - min.z;

  return SC_SACD_AABB_Box{min.x + temp_a / 2.0F,
                          min.y + temp_b / 2.0F,
                          min.z + temp_c / 2.0F,
                          temp_a,
                          temp_b,
                          temp_c};
}

SC_SACD_Vec3 SC_SACD_Vec3_Sum(SC_SACD_Vec3 a, SC_SACD_Vec3 b) { return a + b; }

SC_SACD_Vec3 SC_SACD_Vec3_Difference(SC_SACD_Vec3 a, SC_SACD_Vec3 b) {
  return a - b;
}

SC_SACD_Vec3 SC_SACD_Vec3_Mult(SC_SACD_Vec3 vec, float scalar) {
  return vec * scalar;
}

SC_SACD_Vec3 SC_SACD_Vec3_Div(SC_SACD_Vec3 vec, float scalar) {
  return vec / scalar;
}

SC_SACD_Vec3 SC_SACD_Vec3_Negate(SC_SACD_Vec3 vec) {
  return SC_SACD_Vec3{-vec.x, -vec.y, -vec.z};
}

SC_SACD_Vec3 SC_SACD_Vec3_Normalize(SC_SACD_Vec3 vec) {
  return vec / SC_SACD_Vec3_Length(vec);
}

SC_SACD_Vec3 SC_SACD_Vec3_Project(SC_SACD_Vec3 vec, SC_SACD_Vec3 target) {
  float upper_dot_product = SC_SACD_Dot_Product(vec, target);
  float lower_dot_product = SC_SACD_Dot_Product(target, target);

  return target * (upper_dot_product / lower_dot_product);
}

SC_SACD_Vec3 SC_SACD_Vec3_Reflect(SC_SACD_Vec3 vec, SC_SACD_Vec3 target) {
  SC_SACD_Vec3 proj = SC_SACD_Vec3_Project(vec, target);
  return proj * 2.0F - vec;
}

SC_SACD_Mat4 SC_SACD_Mat3_Promote(SC_SACD_Mat3 mat3) {
  return SC_SACD_Mat4{mat3.x0, mat3.x1, mat3.x2, 0.0F,    mat3.y0, mat3.y1,
                      mat3.y2, 0.0F,    mat3.z0, mat3.z1, mat3.z2, 0.0F,
                      0.0F,    0.0F,    0.0F,    1.0F};
}

SC_SACD_Mat3 SC_SACD_Mat4_Demote(SC_SACD_Mat4 mat4) {
  return SC_SACD_Mat3{mat4.x0, mat4.x1, mat4.x2, mat4.y0, mat4.y1,
                      mat4.y2, mat4.z0, mat4.z1, mat4.z2};
}

SC_SACD_Mat3 SC_SACD_Mat3_Identity() {
  return SC_SACD_Mat3{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
}

SC_SACD_Mat3 SC_SACD_Mat3_Sum(SC_SACD_Mat3 a, SC_SACD_Mat3 b) { return a + b; }

SC_SACD_Mat3 SC_SACD_Mat3_Mult(SC_SACD_Mat3 a, SC_SACD_Mat3 b) { return a * b; }

SC_SACD_Vec3 SC_SACD_Mat3_Vec3_Mult(SC_SACD_Mat3 mat3, SC_SACD_Vec3 vec) {
  return mat3 * vec;
}

SC_SACD_Mat3 SC_SACD_Rotation_Mat3_XAxis(float x_radians) {
  SC_SACD_Mat3 mat;

  mat.x0 = 1.0F;
  mat.x1 = 0.0F;
  mat.x2 = 0.0F;

  mat.y0 = 0.0F;
  mat.y1 = std::cos(x_radians);
  mat.y2 = -std::sin(x_radians);

  mat.z0 = 0.0F;
  mat.z1 = -mat.y2;
  mat.z2 = mat.y1;

  return mat;
}

SC_SACD_Mat3 SC_SACD_Rotation_Mat3_YAxis(float y_radians) {
  SC_SACD_Mat3 mat;

  mat.x0 = std::cos(y_radians);
  mat.x1 = 0.0F;
  mat.x2 = std::sin(y_radians);

  mat.y0 = 0.0F;
  mat.y1 = 1.0F;
  mat.y2 = 0.0F;

  mat.z0 = -mat.x2;
  mat.z1 = 0.0F;
  mat.z2 = mat.x0;

  return mat;
}

SC_SACD_Mat3 SC_SACD_Rotation_Mat3_ZAxis(float z_radians) {
  SC_SACD_Mat3 mat;

  mat.x0 = std::cos(z_radians);
  mat.x1 = -std::sin(z_radians);
  mat.x2 = 0.0F;

  mat.y0 = -mat.x1;
  mat.y1 = mat.x0;
  mat.y2 = 0.0F;

  mat.z0 = 0.0F;
  mat.z1 = 0.0F;
  mat.z2 = 1.0F;

  return mat;
}

SC_SACD_Mat3 SC_SACD_Scale_Mat3(float x, float y, float z) {
  return SC_SACD_Mat3{x, 0.0f, 0.0f, 0.0f, y, 0.0f, 0.0f, 0.0f, z};
}

SC_SACD_Mat3 SC_SACD_ExpMap(SC_SACD_Vec3 axis, float angle) {
  axis = SC_SACD_Vec3_Normalize(axis);
  auto uhat = UHat_Mat3(axis.x, axis.y, axis.z);
  auto uhat_squared = uhat * uhat;
  return SC_SACD_Mat3_Identity() + uhat * std::sin(angle) +
         uhat_squared * (1 - std::cos(angle));
}

float SC_SACD_LogMap_Angle(SC_SACD_Mat3 mat3) {
  float trace = mat3.x0 + mat3.y1 + mat3.z2;
  return std::acos((trace - 1.0F) / 2.0F);
}

SC_SACD_Vec3 SC_SACD_LogMap_Axis(SC_SACD_Mat3 mat3, float angle) {
  return SC_SACD_Vec3{mat3.z1 - mat3.y2, mat3.x2 - mat3.z0, mat3.y0 - mat3.x1} *
         (1.0F / (2.0F * std::sin(angle)));
}
