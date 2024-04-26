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
  float x0, y0, z0;
  float x1, y1, z1;
  float x2, y2, z2;
} SC_SACD_Mat3;

typedef struct SC_SACD_EXPORT SC_SACD_AABB_Box {
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

typedef struct SC_SACD_EXPORT SC_SACD_Generic_Box {
  float x;
  float y;
  float z;
  /// x-axis width.
  float width;
  /// y-axis height.
  float height;
  /// z-axis depth.
  float depth;
  /// Rotation about center of box about axis.
  float x_radians;
  float y_radians;
  float z_radians;
} SC_SACD_Generic_Box;

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Box_Collision(const SC_SACD_AABB_Box *a,
                                              const SC_SACD_AABB_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Generic_Box_Collision(const SC_SACD_Generic_Box *a,
                                                 const SC_SACD_Generic_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Generic_Box_Collision(
    const SC_SACD_AABB_Box *a, const SC_SACD_Generic_Box *b);

SC_SACD_EXPORT float SC_SACD_Dot_Product(const SC_SACD_Vec3 a,
                                         const SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Cross_Product(const SC_SACD_Vec3 a,
                                                  const SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Mat3_Vec3_Mult(const SC_SACD_Mat3 *mat,
                                                   const SC_SACD_Vec3 vec);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Rotate(const SC_SACD_Vec3 vec,
                                                float x_axis, float y_axis,
                                                float z_axis);

#ifdef __cplusplus
}
#endif

#endif
