#ifndef SEODISPARATE_COM_SEPARATING_AXIS_COLLISION_DETECTION_H_
#define SEODISPARATE_COM_SEPARATING_AXIS_COLLISION_DETECTION_H_

#define SC_SACD_PLATFORM_WINDOWS 1
#define SC_SACD_PLATFORM_OTHER 2

#if defined _WIN32
#define SC_SACD_PLATFORM SC_SACD_PLATFORM_WINDOWS
#else
#define SC_SACD_PLATFORM SC_SACD_PLATFORM_OTHER
#endif

#if SC_SACD_PLATFORM == SC_SACD_PLATFORM_WINDOWS
#define SC_SACD_EXPORT __declspec(dllexport)
#else
#define SC_SACD_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SC_SACD_EXPORT SC_SACD_Vec3 {
  float x, y, z;
} SC_SACD_Vec3;

typedef struct SC_SACD_EXPORT SC_SACD_Mat3 {
  float x0, x1, x2;
  float y0, y1, y2;
  float z0, z1, z2;
} SC_SACD_Mat3;

typedef struct SC_SACD_EXPORT SC_SACD_Mat4 {
  float x0, x1, x2, x3;
  float y0, y1, y2, y3;
  float z0, z1, z2, z3;
  float w0, w1, w2, w3;
} SC_SACD_Mat4;

typedef struct SC_SACD_EXPORT SC_SACD_AABB_Box {
  /// Coordinates are to center of box.
  float x;
  float y;
  float z;
  /// x-axis width.
  float width;
  /// y-axis height.
  float height;
  /// z-axis depth.
  float depth;
} SC_SACD_AABB_Box;

/// Must not be zero initialized. Use SC_SACD_Generix_Box_Default() instead.
typedef struct SC_SACD_EXPORT SC_SACD_Generic_Box {
  /// Coordinates are to center of box.
  float x;
  float y;
  float z;
  /// x-axis width.
  float width;
  /// y-axis height.
  float height;
  /// z-axis depth.
  float depth;
  /// Local transform; expects center of box as origin.
  SC_SACD_Mat4 transform;
} SC_SACD_Generic_Box;

typedef struct SC_SACD_EXPORT SC_SACD_Sphere {
  float x;
  float y;
  float z;
  float radius;
} SC_SACD_Sphere;

/// Returns a box at origin with width/height/depth of 2 each.
SC_SACD_EXPORT SC_SACD_Generic_Box SC_SACD_Generic_Box_Default(void);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Box_Collision(SC_SACD_AABB_Box a,
                                              SC_SACD_AABB_Box b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Generic_Box_Collision(SC_SACD_Generic_Box a,
                                                 SC_SACD_Generic_Box b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Generic_Box_Collision(SC_SACD_AABB_Box a,
                                                      SC_SACD_Generic_Box b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_Collision(SC_SACD_Sphere a, SC_SACD_Sphere b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_AABB_Box_Collision(SC_SACD_Sphere sphere,
                                                     SC_SACD_AABB_Box box);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_Box_Collision(SC_SACD_Sphere sphere,
                                                SC_SACD_Generic_Box box);

SC_SACD_EXPORT float SC_SACD_Dot_Product(SC_SACD_Vec3 a, SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Cross_Product(SC_SACD_Vec3 a,
                                                  SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat4_Identity(void);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat4_Sum(SC_SACD_Mat4 a, SC_SACD_Mat4 b);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat4_Mult(SC_SACD_Mat4 a, SC_SACD_Mat4 b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Mat4_Vec3_Mult(SC_SACD_Mat4 mat,
                                                   SC_SACD_Vec3 vec);

/// Rotates by x-axis first, then y-axis, then finally z-axis.
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Rotate(SC_SACD_Vec3 vec, float x_axis,
                                                float y_axis, float z_axis);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_XAxis(float x_radians);
SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_YAxis(float y_radians);
SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_ZAxis(float z_radians);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Translate_Mat4(float x, float y, float z);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Scale_Mat4(float x, float y, float z);

/// This variant of Closest_Point expects "dir" to be a unit vector.
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Closest_Point_Dir_Normalized(
    SC_SACD_Vec3 pos, SC_SACD_Vec3 dir, SC_SACD_Vec3 point);

/// This variant of Closest_Point expects "dir" to NOT be a unit vector.
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Closest_Point(SC_SACD_Vec3 pos,
                                                  SC_SACD_Vec3 dir,
                                                  SC_SACD_Vec3 point);

SC_SACD_EXPORT float SC_SACD_Vec3_Length(SC_SACD_Vec3 vec);

SC_SACD_EXPORT SC_SACD_AABB_Box SC_SACD_Sphere_To_AABB(SC_SACD_Sphere s);
SC_SACD_EXPORT SC_SACD_AABB_Box
SC_SACD_Generic_Box_To_AABB(SC_SACD_Generic_Box s);

/// Combines AABB's such that the new AABB encompasses the two AABB's.
SC_SACD_EXPORT SC_SACD_AABB_Box SC_SACD_AABB_Combine(SC_SACD_AABB_Box a,
                                                     SC_SACD_AABB_Box b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Sum(SC_SACD_Vec3 a, SC_SACD_Vec3 b);
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Difference(SC_SACD_Vec3 a,
                                                    SC_SACD_Vec3 b);
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Mult(SC_SACD_Vec3 vec, float scalar);
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Div(SC_SACD_Vec3 vec, float scalar);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Negate(SC_SACD_Vec3 vec);

/// Turns vec into a unit vector and returns the unit vector.
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Normalize(SC_SACD_Vec3 vec);

/// Projects "vec" onto "target".
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Project(SC_SACD_Vec3 vec,
                                                 SC_SACD_Vec3 target);

/// Reflects "vec" about "target".
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Reflect(SC_SACD_Vec3 vec,
                                                 SC_SACD_Vec3 target);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat3_Promote(SC_SACD_Mat3 mat3);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Mat4_Demote(SC_SACD_Mat4 mat4);

SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Mat3_Identity(void);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Mat3_Sum(SC_SACD_Mat3 a, SC_SACD_Mat3 b);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Mat3_Mult(SC_SACD_Mat3 a, SC_SACD_Mat3 b);
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Mat3_Vec3_Mult(SC_SACD_Mat3 mat3,
                                                   SC_SACD_Vec3 vec);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Rotation_Mat3_XAxis(float x_radians);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Rotation_Mat3_YAxis(float y_radians);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Rotation_Mat3_ZAxis(float z_radians);
SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_Scale_Mat3(float x, float y, float z);

SC_SACD_EXPORT SC_SACD_Mat3 SC_SACD_ExpMap(SC_SACD_Vec3 axis, float angle);
SC_SACD_EXPORT float SC_SACD_LogMap_Angle(SC_SACD_Mat3 mat3);
SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_LogMap_Axis(SC_SACD_Mat3 mat3, float angle);

#ifdef __cplusplus
}
#endif

#endif
