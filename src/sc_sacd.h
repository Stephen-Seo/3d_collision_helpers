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
  /// Rotation about center of box about axis.
  float x_radians;
  float y_radians;
  float z_radians;
} SC_SACD_Generic_Box;

typedef struct SC_SACD_EXPORT SC_SACD_Sphere {
  float x;
  float y;
  float z;
  float radius;
} SC_SACD_Sphere;

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Box_Collision(const SC_SACD_AABB_Box *a,
                                              const SC_SACD_AABB_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Generic_Box_Collision(const SC_SACD_Generic_Box *a,
                                                 const SC_SACD_Generic_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Generic_Box_Collision(
    const SC_SACD_AABB_Box *a, const SC_SACD_Generic_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_Collision(const SC_SACD_Sphere *a,
                                            const SC_SACD_Sphere *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_AABB_Box_Collision(
    const SC_SACD_Sphere *sphere, const SC_SACD_AABB_Box *box);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Sphere_Box_Collision(const SC_SACD_Sphere *sphere,
                                                const SC_SACD_Generic_Box *box);

SC_SACD_EXPORT float SC_SACD_Dot_Product(const SC_SACD_Vec3 a,
                                         const SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Cross_Product(const SC_SACD_Vec3 a,
                                                  const SC_SACD_Vec3 b);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat4_Identity(void);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Mat4_Mult(const SC_SACD_Mat4 *a,
                                              const SC_SACD_Mat4 *b);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Mat4_Vec3_Mult(const SC_SACD_Mat4 *mat,
                                                   const SC_SACD_Vec3 vec);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Vec3_Rotate(const SC_SACD_Vec3 vec,
                                                float x_axis, float y_axis,
                                                float z_axis);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_XAxis(float x_radians);
SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_YAxis(float y_radians);
SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Rotation_Mat4_ZAxis(float z_radians);

SC_SACD_EXPORT SC_SACD_Mat4 SC_SACD_Translate_Mat4(float x, float y, float z);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Closest_Point_Dir_Normalized(
    const SC_SACD_Vec3 *pos, const SC_SACD_Vec3 *dir,
    const SC_SACD_Vec3 *point);

SC_SACD_EXPORT SC_SACD_Vec3 SC_SACD_Closest_Point(const SC_SACD_Vec3 *pos,
                                                  const SC_SACD_Vec3 *dir,
                                                  const SC_SACD_Vec3 *point);

#ifdef __cplusplus
}
#endif

#endif
