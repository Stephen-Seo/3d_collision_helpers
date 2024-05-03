# Changelog

## Upcoming Changes

Refactoring of internally used function(s).

This library now requires a compiler that supports C++20.

## Version 2.0.2

Fix SC_SACD_Translate_Mat4(...). It was missing a "1" in the first element of
the Mat4.

Made internal function getting normals of a box more robust.

## Version 2.0.1

Added check in SC_SACD_Sphere_Box_Collision(...) to see if sphere and box has
same center.

## Version 2.0.0

Add collision detection with Spheres.

Add some more vector/matrix math stuff.

Replace Mat3 with Mat4.

Replace xyz-axis rotation floats in GenericBox with Mat4.

## Version 1.0.0

First version of this library.

Implements collision detection between AABB (Axis-Aligned-Bounding-Box) and
AABB, AABB and Generic-Boxes, and Generic-Boxes and Generic-Boxes.

Some vector/matrix math also added.
