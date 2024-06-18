# Changelog

## Upcoming Changes

## Version 3.3.0

Refactor some internal operators for Vec3/Mat4 operations (like mult., sum,
etc.)

Added Mat3 and helper functions related to Mat3.

Implemented conversion from axis/angle to rotation matrix (Mat3).

Implemented conversion from rotation matrix (mat3) to axis/angle (untested).

## Version 3.2.1

Add workflow to build shared/static libs available
[here](https://git.seodisparate.com/stephenseo/3d_collision_helpers/releases).

## Version 3.2.0

Add Vec3 negate and normalize functions.

## Version 3.1.0

Add Vec3 helper functions.

Add Vec3 project and reflect functions.

## Version 3.0.1

Remove unnecessary header include in sc_sacd.cpp. This should fix builds that
do not target C++23 as the header is from C++23.

## Version 3.0.0

### Breaking Changes

Change pointer-parameters in API to non-pointer parameters.

## Version 2.2.1

Fix UnitTest for checking AABB.

## Version 2.2.0

Refactoring of internally used function.

Add functions to convert Sphere and GenericBox to AABB.

Add function to combine two AABBs.

## Version 2.1.0

Refactoring of internally used function(s).

This library now requires a compiler that supports C++20.

Add SC_SACD_Scale_Mat4(...) fn.

## Version 2.0.2

Fix SC_SACD_Translate_Mat4(...). It was missing a "1" in the first element of
the Mat4.

Made internal function getting normals of a box more robust.

## Version 2.0.1

Added check in SC_SACD_Sphere_Box_Collision(...) to see if sphere and box has
same center.

## Version 2.0.0

### Breaking Changes

Replace Mat3 with Mat4.

Replace xyz-axis rotation floats in GenericBox with Mat4.

### Non-Breaking Changes

Add collision detection with Spheres.

Add some more vector/matrix math stuff.

## Version 1.0.0

First version of this library.

Implements collision detection between AABB (Axis-Aligned-Bounding-Box) and
AABB, AABB and Generic-Boxes, and Generic-Boxes and Generic-Boxes.

Some vector/matrix math also added.
