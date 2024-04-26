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

#include "sd_sacd.h"

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

  std::cout << "Checks checked: " << checks_checked << '\n'
            << "Checks passed:  " << checks_passed << '\n';

  return 0;
}
