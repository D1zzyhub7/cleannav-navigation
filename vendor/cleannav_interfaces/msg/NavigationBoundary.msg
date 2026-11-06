# Immutable constraint generation. Empty/inactive is only published by the Facade
# after the previous controller execution has reached a verified terminal state.
string<=64 token
bool active
geometry_msgs/PolygonStamped boundary
float64 clearance_m
