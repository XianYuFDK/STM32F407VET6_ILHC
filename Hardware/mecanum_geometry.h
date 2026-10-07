#ifndef MECANUM_GEOMETRY_H
#define MECANUM_GEOMETRY_H
/* Wheel-center distances, not the outer collision rectangle. */
/* User measurements: axle centers 206; outer wheel span 250; wheel width 32 mm. */
#define MECANUM_WHEELBASE_MM 206.0f
#define MECANUM_TRACK_MM 218.0f
#define MECANUM_ROTATION_LEVER_MM ((MECANUM_WHEELBASE_MM+MECANUM_TRACK_MM)*0.5f)
#endif
