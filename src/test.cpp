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

#include "sc_sacd.h"

int main() {
  // Test 2D AABB.
  {
    SC_SACD_AABB_Box a{0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};
    SC_SACD_AABB_Box b{2.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};

    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.x = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.x = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));
    b.x = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.x = 0.0F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));
    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.y = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));
    b.y = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.y = 0.0F;
    b.z = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.z = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.z = 0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.z = -0.5F;
    CHECK_TRUE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.z = 0.0F;
    b.x = 0.5F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.x = -0.5F;
    b.y = 2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));

    b.y = -2.0F;
    CHECK_FALSE(SC_SACD_AABB_Box_Collision(&a, &b));
  }

  // Test Separating_Axis_Collision check.
  {
    SC_SACD_Generic_Box a{0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F};
    SC_SACD_Generic_Box b{0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F};

    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));

    b.x = 1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.x = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));

    a.z_radians = std::numbers::pi_v<float> / 4.0F;
    b.z_radians = a.z_radians;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.x = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));

    b.x = 0.0F;
    b.y = 1.1F;
    a.z_radians = 0.0F;
    b.z_radians = 0.0F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.y = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));

    a.z_radians = std::numbers::pi_v<float> / 4.0F;
    b.z_radians = a.z_radians;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.y = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));

    b.y = 0.0F;
    a.z_radians = 0.0F;
    b.z_radians = 0.0F;

    b.z = 1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.z = -1.1F;
    CHECK_FALSE(SC_SACD_Generic_Box_Collision(&a, &b));

    a.y_radians = std::numbers::pi_v<float> / 4.0F;
    b.y_radians = a.y_radians;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));
    b.z = 1.1F;
    CHECK_TRUE(SC_SACD_Generic_Box_Collision(&a, &b));
  }

  // Test Sphere/Sphere collision check.
  {
    SC_SACD_Sphere a{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_Sphere b{0.0F, 0.0F, 0.0F, 1.0F};
    CHECK_TRUE(SC_SACD_Sphere_Collision(&a, &b));

    a.x = 10.0F;
    a.y = 10.0F;
    a.z = 10.0F;
    b.x = 10.0F;
    b.y = 10.5F;
    b.z = 10.0F;
    CHECK_TRUE(SC_SACD_Sphere_Collision(&a, &b));
    b.y = 12.0F;
    b.z = 12.0F;
    CHECK_FALSE(SC_SACD_Sphere_Collision(&a, &b));
    b.y = 11.0F;
    b.z = 11.0F;
    CHECK_TRUE(SC_SACD_Sphere_Collision(&a, &b));
  }

  // Test Sphere/AABB collision check.
  {
    SC_SACD_Sphere sphere{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_AABB_Box box{0.0F, 0.0F, 0.0F, 2.0F, 2.0F, 2.0F};

    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 0.0F;
    sphere.y = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.y = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.y = 0.0F;
    sphere.z = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.z = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.z = 0.0F;
    sphere.x = 1.0F;
    sphere.y = 1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = -1.0F;
    sphere.y = -1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 2.0F;
    sphere.y = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = -2.0F;
    sphere.y = -2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 1.0F;
    sphere.y = 0.0F;
    sphere.z = 1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = -1.0F;
    sphere.z = -1.0F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 2.0F;
    sphere.z = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = -2.0F;
    sphere.z = -2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 1.0F;
    sphere.z = 1.5F;
    CHECK_TRUE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));

    sphere.x = 1.0F;
    sphere.z = 2.0F;
    CHECK_FALSE(SC_SACD_Sphere_AABB_Box_Collision(&sphere, &box));
  }

  // Test Sphere/Generic_Box collision check.
  {
    SC_SACD_Sphere sphere{0.0F, 0.0F, 0.0F, 1.0F};
    SC_SACD_Generic_Box box{0.0F, 0.0F, 0.0F,
                            2.0F, 2.0F, 2.0F,
                            0.0F, 0.0F, std::numbers::pi_v<float> / 4.0F};

    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.x = -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.x = 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.x = -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = 0.0F;
    sphere.y = 2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.y = -2.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.y = 2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.y = -2.3F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.y = 0.0F;
    sphere.z = 2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.z = -2.1F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.z = 1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.z = -1.9F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = 1.5F;
    sphere.y = 1.5F;
    sphere.z = 0.0F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.x = 1.4F;
    sphere.y = 1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = 2.2F;
    sphere.y = 0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = 2.1F;
    sphere.y = 0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = -1.5F;
    sphere.y = -1.5F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
    sphere.x = -1.4F;
    sphere.y = -1.4F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = -2.2F;
    sphere.y = -0.7929F;
    CHECK_FALSE(SC_SACD_Sphere_Box_Collision(&sphere, &box));

    sphere.x = -2.1F;
    sphere.y = -0.6929F;
    CHECK_TRUE(SC_SACD_Sphere_Box_Collision(&sphere, &box));
  }

  std::cout << "Checks checked: " << checks_checked << '\n'
            << "Checks passed:  " << checks_passed << '\n';

  return 0;
}
