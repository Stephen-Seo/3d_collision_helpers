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

typedef struct SC_SACD_EXPORT SC_SACD_AABB_Box {
  float x;
  float y;
  float width;
  float height;
} SC_SACD_AABB_Box;

typedef struct SC_SACD_EXPORT SC_SACD_Generic_Box {
  float x;
  float y;
  float width;
  float height;
  float radians;
} SC_SACD_Generic_Box;

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Box_Collision(const SC_SACD_AABB_Box *a,
                                              const SC_SACD_AABB_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_Generic_Box_Collision(const SC_SACD_Generic_Box *a,
                                                 const SC_SACD_Generic_Box *b);

/// Returns non-zero if there is collision.
SC_SACD_EXPORT int SC_SACD_AABB_Generic_Box_collision(
    const SC_SACD_AABB_Box *a, const SC_SACD_Generic_Box *b);

#ifdef __cplusplus
}
#endif

#endif
