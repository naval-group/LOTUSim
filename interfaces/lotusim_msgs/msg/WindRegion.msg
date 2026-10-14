uint8 BOX=0
uint8 CONE_SEGMENT=1

string id
uint8 shape_type

WindRegionBox box
WindRegionConeSegment cone

geometry_msgs/Vector3 linear_velocity
bool enable_wind

# Standard deviation [m/s] of the turbulent fluctuation about linear_velocity
# inside this region. 0.0 (the default, and what every pre-existing publisher
# produces) means a steady region, so this field is additive: nothing changes
# for a producer that does not set it.
#
# Sigma in m/s rather than a dimensionless turbulence intensity, deliberately:
# an intensity is a ratio to *some* reference speed, and in a wake the local
# (deficit) speed and the freestream differ substantially. Crespo-type
# wake-added turbulence intensity is defined against the freestream, but
# linear_velocity above carries the deficit speed — so a consumer handed a
# ratio and this message would have no way to recover the right reference and
# would silently normalise by the wrong one. Putting sigma on the wire keeps
# the convention where the knowledge is, in the producer.
float64 turbulence_sigma 0.0

# Speed [m/s] sampled across the region's radius, index 0 on the axis and the
# last index at the boundary, evenly spaced in r/r_boundary. Empty (the
# default, and what every pre-existing publisher produces) means the region is
# radially uniform at linear_velocity, so this field is additive.
#
# Why it exists: a wake model computes a graded cross-wake profile, but a
# region carries one linear_velocity. Publishing only that value forced the
# producer to pick a single radius to represent the whole cross-section, and
# it picked the axis -- so a vehicle anywhere in the cone was handed the
# CENTRELINE deficit, out to the radius where the true deficit had already
# decayed to the 5% cut-off. Integrated over the cross-section that is about
# 2.1x the deficit the model actually predicts, and it turns a graded profile
# into a top-hat with no interior gradient at all.
#
# Direction comes from linear_velocity as before; only the magnitude varies
# with radius, which is what the wake models vary.
float64[] radial_speed
