# Changelog

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
