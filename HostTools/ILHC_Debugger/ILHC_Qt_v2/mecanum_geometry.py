"""Wheel-center geometry; kept equal to Hardware/mecanum_geometry.h."""
WHEELBASE_MM=206.0
TRACK_MM=218.0
ROTATION_LEVER_MM=(WHEELBASE_MM+TRACK_MM)/2
WHEEL_GEOMETRY=dict(wheelbase_mm=WHEELBASE_MM,track_mm=TRACK_MM,verified=True)
