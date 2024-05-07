#include <cmath>
#include <iostream>
#include <numbers>

static int checks_checked = 0;
static int checks_passed = 0;

// Macros for unit testing.

#define CHECK_TRUE(x)                                                 \
  do {                                                                \
    ++checks_checked;                                                 \
    if (!(x)) {                                                       \
      std::cout << "CHECK_TRUE at line " << __LINE__ << " failed!\n"; \
    } else {                                                          \
      ++checks_passed;                                                \
    }                                                                 \
  } while (false);
#define CHECK_FALSE(x)                                                 \
  do {                                                                 \
    ++checks_checked;                                                  \
    if (x) {                                                           \
      std::cout << "CHECK_FALSE at line " << __LINE__ << " failed!\n"; \
    } else {                                                           \
      ++checks_passed;                                                 \
    }                                                                  \
  } while (false);

#define CHECK_FLOAT(var, value)                                        \
  do {                                                                 \
    ++checks_checked;                                                  \
    if ((var) > (value)-0.0001F && (var) < (value) + 0.0001F) {        \
      ++checks_passed;                                                 \
    } else {                                                           \
      std::cout << "CHECK_FLOAT at line " << __LINE__ << " failed!\n"; \
    }                                                                  \
  } while (false);

#include "sc_sacd.h"

int main() {
  // Test 2D AABB.
  {
    SC_SACD_AABB_Box a{0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};
    SC_SACD_AABB_Box b{2.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};

    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.x = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.x = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));
    b.x = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));

    b.x = 0.0F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));
    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.y = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));
    b.y = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));

    b.y = 0.0F;
    b.z = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.z = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.z = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));

    b.z = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(a, b));

    b.z = 0.0F;
    b.x = 0.5F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.x = -0.5F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));

    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(a, b));
  }

  // Test Separating_Axis_Collision check.
  {
    SC_SACD_Generic_Box a{
        0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, SC_SACD_Mat4_Identity()};
    SC_SACD_Generic_Box b{
        0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, SC_SACD_Mat4_Identity()};

    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    b.x = 1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));
    b.x = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);
    b.transform = a.transform;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));
    b.x = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    b.x = 0.0F;
    b.y = 1.1F;
    a.transform = SC_SACD_Mat4_Identity();
    b.transform = a.transform;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));
    b.y = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);
    b.transform = a.transform;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));
    b.y = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    b.y = 0.0F;
    a.transform = SC_SACD_Mat4_Identity();
    b.transform = a.transform;

    b.z = 1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));
    b.z = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Rotation_Mat4_YAxis(std::numbers::pi_v<float> / 4.0F);
    b.transform = a.transform;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));
    b.z = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));
  }

  // Test Sphere/Sphere collision check.
  {
    SC_SACD_Sphere a{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_Sphere b{0.0F, 0.0F, 0.0F, 1.0F};
    CHECK_TRUE(SC_SACD_Sphere_Collision(a, b));

    a.x = 10.0F;
    a.y = 10.0F;
    a.z = 10.0F;
    b.x = 10.0F;
    b.y = 10.5F;
    b.z = 10.0F;
    CHECK_TRUE(SC_SACD_Sphere_Collision(a, b));
    b.y = 12.0F;
    b.z = 12.0F;
    CHECK_FALSE(SC_SACD_Sphere_Collision(a, b));
    b.y = 11.0F;
    b.z = 11.0F;
    CHECK_TRUE(SC_SACD_Sphere_Collision(a, b));
  }

  // Test Sphere/AABB collision check.
  {
    SC_SACD_Sphere sphere{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_AABB_Box box{0.0F, 0.0F, 0.0F, 2.0F, 2.0F, 2.0F};

    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 0.0F;
    sphere.y = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.y = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.y = 0.0F;
    sphere.z = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.z = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.z = 0.0F;
    sphere.x = 1.0F;
    sphere.y = 1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = -1.0F;
    sphere.y = -1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 2.0F;
    sphere.y = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = -2.0F;
    sphere.y = -2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 1.0F;
    sphere.y = 0.0F;
    sphere.z = 1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = -1.0F;
    sphere.z = -1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 2.0F;
    sphere.z = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = -2.0F;
    sphere.z = -2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 1.0F;
    sphere.z = 1.5F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));

    sphere.x = 1.0F;
    sphere.z = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(sphere, box));
  }

  // Test Sphere/Generic_Box collision check.
  {
    SC_SACD_Sphere sphere{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_Generic_Box box{
        0.0F,
        0.0F,
        0.0F,
        2.0F,
        2.0F,
        2.0F,
        SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F)};

    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 0.0F;
    sphere.y = 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.y = 0.0F;
    sphere.z = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = 1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = -1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 1.5F;
    sphere.y = 1.5F;
    sphere.z = 0.0F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 1.4F;
    sphere.y = 1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 2.2F;
    sphere.y = 0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 2.1F;
    sphere.y = 0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = -1.5F;
    sphere.y = -1.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = -1.4F;
    sphere.y = -1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = -2.2F;
    sphere.y = -0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = -2.1F;
    sphere.y = -0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    box.x = 10.0F;
    box.y = -10.0F;
    box.z = 13.0F;

    sphere.y = -10.0F;
    sphere.z = 13.0F;
    sphere.x = 10.0F + 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 10.0F + -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 10.0F + 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 10.0F + -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + 0.0F;
    sphere.y = -10.0F + 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = -10.0F + -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = -10.0F + 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.y = -10.0F + -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.y = -10.0F + 0.0F;
    sphere.z = 13.0F + 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = 13.0F + -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = 13.0F + 1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.z = 13.0F + -1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + 1.5F;
    sphere.y = -10.0F + 1.5F;
    sphere.z = 13.0F + 0.0F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 10.0F + 1.4F;
    sphere.y = -10.0F + 1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + 2.2F;
    sphere.y = -10.0F + 0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + 2.1F;
    sphere.y = -10.0F + 0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + -1.5F;
    sphere.y = -10.0F + -1.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));
    sphere.x = 10.0F + -1.4F;
    sphere.y = -10.0F + -1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + -2.2F;
    sphere.y = -10.0F + -0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(sphere, box));

    sphere.x = 10.0F + -2.1F;
    sphere.y = -10.0F + -0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(sphere, box));
  }

  // Test matrix/vector multiplication.
  {
    SC_SACD_Mat4 mat_a{1.0F, 2.0F,  3.0F,  4.0F,  5.0F,  6.0F,  7.0F,  8.0F,
                       9.0F, 10.0F, 11.0F, 12.0F, 13.0F, 14.0F, 15.0F, 16.0F};

    SC_SACD_Mat4 mat_b = SC_SACD_Mat4_Identity();

    {
      auto result = SC_SACD_Mat4_Mult(mat_a, mat_b);
      CHECK_TRUE(mat_a.x0 == result.x0);
      CHECK_TRUE(mat_a.x1 == result.x1);
      CHECK_TRUE(mat_a.x2 == result.x2);
      CHECK_TRUE(mat_a.x3 == result.x3);
      CHECK_TRUE(mat_a.y0 == result.y0);
      CHECK_TRUE(mat_a.y1 == result.y1);
      CHECK_TRUE(mat_a.y2 == result.y2);
      CHECK_TRUE(mat_a.y3 == result.y3);
      CHECK_TRUE(mat_a.z0 == result.z0);
      CHECK_TRUE(mat_a.z1 == result.z1);
      CHECK_TRUE(mat_a.z2 == result.z2);
      CHECK_TRUE(mat_a.z3 == result.z3);
      CHECK_TRUE(mat_a.w0 == result.w0);
      CHECK_TRUE(mat_a.w1 == result.w1);
      CHECK_TRUE(mat_a.w2 == result.w2);
      CHECK_TRUE(mat_a.w3 == result.w3);
    }

    mat_b.x0 = 2.0F;
    mat_b.y1 = 0.0F;
    mat_b.z2 = 0.0F;
    {
      auto result = SC_SACD_Mat4_Mult(mat_a, mat_b);
      CHECK_FLOAT(result.x0, 2.0F);
      CHECK_FLOAT(result.y0, 10.0F);
      CHECK_FLOAT(result.z0, 18.0F);
      CHECK_FLOAT(result.w0, 26.0F);
      CHECK_FLOAT(result.x1, 0.0F);
      CHECK_FLOAT(result.y1, 0.0F);
      CHECK_FLOAT(result.z1, 0.0F);
      CHECK_FLOAT(result.w1, 0.0F);
      CHECK_FLOAT(result.x2, 0.0F);
      CHECK_FLOAT(result.y2, 0.0F);
      CHECK_FLOAT(result.z2, 0.0F);
      CHECK_FLOAT(result.w2, 0.0F);
      CHECK_FLOAT(result.x3, 4.0F);
      CHECK_FLOAT(result.y3, 8.0F);
      CHECK_FLOAT(result.z3, 12.0F);
      CHECK_FLOAT(result.w3, 16.0F);
    }

    mat_b = SC_SACD_Mat4_Identity();
    SC_SACD_Vec3 vec_a{1.0F, 0.0F, 0.0F};
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_b, vec_a);
      CHECK_TRUE(result.x == vec_a.x);
      CHECK_TRUE(result.y == vec_a.y);
      CHECK_TRUE(result.z == vec_a.z);
    }

    // Rotations about each axis.
    mat_a = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 2.0F);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < 1.0001F && result.y > 0.9999F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    mat_a = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float>);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < -0.9999F && result.x > -1.0001F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    mat_a =
        SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> * 3.0F / 2.0F);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < -0.9999F && result.y > -1.0001F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    mat_a = SC_SACD_Rotation_Mat4_XAxis(std::numbers::pi_v<float> / 2.0F);
    vec_a.x = 0.0F;
    vec_a.y = 1.0F;
    vec_a.z = 0.0F;
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < 1.0001F && result.z > 0.9999F);
    }

    mat_a = SC_SACD_Rotation_Mat4_XAxis(std::numbers::pi_v<float>);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < -0.9999F && result.y > -1.0001F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    mat_a =
        SC_SACD_Rotation_Mat4_XAxis(std::numbers::pi_v<float> * 3.0F / 2.0F);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < -0.9999F && result.z > -1.0001F);
    }

    mat_a = SC_SACD_Rotation_Mat4_YAxis(std::numbers::pi_v<float> / 2.0F);
    vec_a.x = 0.0F;
    vec_a.y = 0.0F;
    vec_a.z = 1.0F;
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 1.0001F && result.x > 0.9999F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    mat_a = SC_SACD_Rotation_Mat4_YAxis(std::numbers::pi_v<float>);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < 0.0001F && result.x > -0.0001F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < -0.9999F && result.z > -1.0001F);
    }

    mat_a =
        SC_SACD_Rotation_Mat4_YAxis(std::numbers::pi_v<float> * 3.0F / 2.0F);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_TRUE(result.x < -0.9999F && result.x > -1.0001F);
      CHECK_TRUE(result.y < 0.0001F && result.y > -0.0001F);
      CHECK_TRUE(result.z < 0.0001F && result.z > -0.0001F);
    }

    // Combined axis rotation.
    vec_a.x = 1.0F;
    vec_a.y = 0.0F;
    vec_a.z = 0.0F;
    mat_a = SC_SACD_Rotation_Mat4_YAxis(std::numbers::pi_v<float> / 4.0F);
    mat_b = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);
    // Apply mat_a, then mat_b.
    mat_a = SC_SACD_Mat4_Mult(mat_b, mat_a);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_FLOAT(result.x, 0.5F);
      CHECK_FLOAT(result.y, 0.5F);
      CHECK_FLOAT(result.z, -std::sqrt(2.0F) / 2.0F);
    }
    // Apply another rotation on combined mat_a.
    mat_b = SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);
    mat_a = SC_SACD_Mat4_Mult(mat_b, mat_a);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_FLOAT(result.x, 0.0F);
      CHECK_FLOAT(result.y, std::sqrt(2.0F) / 2.0F);
      CHECK_FLOAT(result.z, -std::sqrt(2.0F) / 2.0F);
    }
    // Apply another rotation on combined mat_a.
    mat_b = SC_SACD_Rotation_Mat4_XAxis(std::numbers::pi_v<float> / 2.0F);
    mat_a = SC_SACD_Mat4_Mult(mat_b, mat_a);
    {
      auto result = SC_SACD_Mat4_Vec3_Mult(mat_a, vec_a);
      CHECK_FLOAT(result.x, 0.0F);
      CHECK_FLOAT(result.y, std::sqrt(2.0F) / 2.0F);
      CHECK_FLOAT(result.z, std::sqrt(2.0F) / 2.0F);
    }
  }

  // Mat4 rotation and translation.
  {
    SC_SACD_Mat4 mat_a = SC_SACD_Translate_Mat4(1.0F, 1.0F, 1.0F);
    SC_SACD_Mat4 mat_b =
        SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);
    mat_a = SC_SACD_Mat4_Mult(mat_b, mat_a);
    mat_b = SC_SACD_Translate_Mat4(0.0F, 0.0F, -1.0F);
    mat_a = SC_SACD_Mat4_Mult(mat_b, mat_a);

    {
      auto result =
          SC_SACD_Mat4_Vec3_Mult(mat_a, SC_SACD_Vec3{0.0F, 0.0F, 0.0F});
      CHECK_FLOAT(result.x, 0.0F);
      CHECK_FLOAT(result.z, 0.0F);
      CHECK_FLOAT(result.y, std::sqrt(2.0F));
    }
  }

  // Box collision with transform that has translation.
  {
    SC_SACD_Generic_Box a = SC_SACD_Generic_Box_Default();
    SC_SACD_Generic_Box b = SC_SACD_Generic_Box_Default();

    a.width = 1.1F;
    a.height = 1.1F;
    b.width = 1.1F;
    b.height = 1.1F;
    a.transform = SC_SACD_Translate_Mat4(-1.0F, 0.0F, 0.0F);
    b.transform = SC_SACD_Translate_Mat4(0.0F, -1.0F, 0.0F);

    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.width = 0.9F;
    a.height = 0.9F;
    b.width = 0.9F;
    b.height = 0.9F;

    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));
  }

  // Box with Scale Mat4.
  {
    SC_SACD_Generic_Box a = SC_SACD_Generic_Box_Default();
    SC_SACD_Generic_Box b = SC_SACD_Generic_Box_Default();

    a.x = 1.1F;
    b.x = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(2.0F, 1.0F, 1.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(-2.0F, 1.0F, 1.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.x = 0.0F;
    b.x = 0.0F;
    a.y = 1.1F;
    b.y = -1.1F;
    a.transform = SC_SACD_Mat4_Identity();
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(1.0F, 2.0F, 1.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(1.0F, -2.0F, 1.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.y = 0.0F;
    b.y = 0.0F;
    a.z = 1.1F;
    b.z = -1.1F;
    a.transform = SC_SACD_Mat4_Identity();
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(1.0F, 1.0F, 2.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));

    a.transform = SC_SACD_Scale_Mat4(1.0F, 1.0F, -2.0F);
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(a, b));
  }

  // Test Sphere/GenericBox to AABB.
  {
    SC_SACD_Sphere s{10.0F, 10.0F, 10.0F, 5.0F};

    SC_SACD_AABB_Box aabb = SC_SACD_Sphere_To_AABB(s);
    CHECK_FLOAT(aabb.x, s.x);
    CHECK_FLOAT(aabb.y, s.y);
    CHECK_FLOAT(aabb.z, s.z);
    CHECK_FLOAT(aabb.width, s.radius * 2.0F);
    CHECK_FLOAT(aabb.height, s.radius * 2.0F);
    CHECK_FLOAT(aabb.depth, s.radius * 2.0F);

    SC_SACD_Generic_Box box = SC_SACD_Generic_Box_Default();
    box.x = 20.0F;
    box.y = 20.0F;
    box.z = 20.0F;

    aabb = SC_SACD_Generic_Box_To_AABB(box);
    CHECK_FLOAT(aabb.x, box.x);
    CHECK_FLOAT(aabb.y, box.y);
    CHECK_FLOAT(aabb.z, box.z);
    CHECK_FLOAT(aabb.width, box.width);
    CHECK_FLOAT(aabb.height, box.height);
    CHECK_FLOAT(aabb.depth, box.depth);

    box.transform =
        SC_SACD_Rotation_Mat4_ZAxis(std::numbers::pi_v<float> / 4.0F);

    aabb = SC_SACD_Generic_Box_To_AABB(box);
    CHECK_FLOAT(aabb.x, box.x);
    CHECK_FLOAT(aabb.y, box.y);
    CHECK_FLOAT(aabb.z, box.z);
    CHECK_FLOAT(aabb.width, std::sqrt(2.0F) * 2.0F);
    CHECK_FLOAT(aabb.height, std::sqrt(2.0F) * 2.0F);
    CHECK_FLOAT(aabb.depth, box.depth);

    auto translate = SC_SACD_Translate_Mat4(-5.0F, -4.0F, 1.0F);
    box.transform = SC_SACD_Mat4_Mult(translate, box.transform);

    aabb = SC_SACD_Generic_Box_To_AABB(box);
    CHECK_FLOAT(aabb.x, box.x - 5.0F);
    CHECK_FLOAT(aabb.y, box.y - 4.0F);
    CHECK_FLOAT(aabb.z, box.z + 1.0F);
    CHECK_FLOAT(aabb.width, std::sqrt(2.0F) * 2.0F);
    CHECK_FLOAT(aabb.height, std::sqrt(2.0F) * 2.0F);
    CHECK_FLOAT(aabb.depth, box.depth);
  }

  // Test combining AABB.
  {
    SC_SACD_AABB_Box a{5.0F, 5.0F, 5.0F, 2.0F, 2.0F, 2.0F};

    SC_SACD_AABB_Box b{-3.0F, -3.0F, -3.0F, 2.0F, 2.0F, 2.0F};

    auto combined = SC_SACD_AABB_Combine(a, b);

    auto *box_min = a.x - a.width / 2.0F < b.x - b.width / 2.0F ? &a : &b;
    auto *box_max = a.x + a.width / 2.0F > b.x + b.width / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.width, box_max->x + box_max->width / 2.0F -
                                    (box_min->x - box_min->width / 2.0F));
    CHECK_FLOAT(combined.x, box_min->x - box_min->width / 2.0F +
                                (box_max->x + box_max->width / 2.0F -
                                 (box_min->x - box_min->width / 2.0F)) /
                                    2.0F);
    box_min = a.y - a.height / 2.0F < b.y - b.height / 2.0F ? &a : &b;
    box_max = a.y + a.height / 2.0F > b.y + b.height / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.height, box_max->y + box_max->height / 2.0F -
                                     (box_min->y - box_min->height / 2.0F));
    CHECK_FLOAT(combined.y, box_min->y - box_min->height / 2.0F +
                                (box_max->y + box_max->height / 2.0F -
                                 (box_min->y - box_min->height / 2.0F)) /
                                    2.0F);
    box_min = a.z - a.depth / 2.0F < b.z - b.depth / 2.0F ? &a : &b;
    box_max = a.z + a.depth / 2.0F > b.z + b.depth / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.depth, box_max->z + box_max->depth / 2.0F -
                                    (box_min->z - box_min->depth / 2.0F));
    CHECK_FLOAT(combined.z, box_min->z - box_min->depth / 2.0F +
                                (box_max->z + box_max->depth / 2.0F -
                                 (box_min->z - box_min->depth / 2.0F)) /
                                    2.0F);

    a.x = -12.0F;
    a.z = -1.0F;
    a.width = 1.0F;
    a.height = 5.0F;
    a.depth = 0.5F;

    b.x = 1.0F;
    b.y = 7.0F;
    b.width = 3.0F;
    b.height = 4.0F;
    b.depth = 0.7F;

    combined = SC_SACD_AABB_Combine(a, b);

    box_min = a.x - a.width / 2.0F < b.x - b.width / 2.0F ? &a : &b;
    box_max = a.x + a.width / 2.0F > b.x + b.width / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.width, box_max->x + box_max->width / 2.0F -
                                    (box_min->x - box_min->width / 2.0F));
    CHECK_FLOAT(combined.x, box_min->x - box_min->width / 2.0F +
                                (box_max->x + box_max->width / 2.0F -
                                 (box_min->x - box_min->width / 2.0F)) /
                                    2.0F);
    box_min = a.y - a.height / 2.0F < b.y - b.height / 2.0F ? &a : &b;
    box_max = a.y + a.height / 2.0F > b.y + b.height / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.height, box_max->y + box_max->height / 2.0F -
                                     (box_min->y - box_min->height / 2.0F));
    CHECK_FLOAT(combined.y, box_min->y - box_min->height / 2.0F +
                                (box_max->y + box_max->height / 2.0F -
                                 (box_min->y - box_min->height / 2.0F)) /
                                    2.0F);
    box_min = a.z - a.depth / 2.0F < b.z - b.depth / 2.0F ? &a : &b;
    box_max = a.z + a.depth / 2.0F > b.z + b.depth / 2.0F ? &a : &b;
    CHECK_FLOAT(combined.depth, box_max->z + box_max->depth / 2.0F -
                                    (box_min->z - box_min->depth / 2.0F));
    CHECK_FLOAT(combined.z, box_min->z - box_min->depth / 2.0F +
                                (box_max->z + box_max->depth / 2.0F -
                                 (box_min->z - box_min->depth / 2.0F)) /
                                    2.0F);
  }

  std::cout << "Checks checked: " << checks_checked << '\n'
            << "Checks passed:  " << checks_passed << '\n';

  return checks_checked == checks_passed ? 0 : 1;
}
