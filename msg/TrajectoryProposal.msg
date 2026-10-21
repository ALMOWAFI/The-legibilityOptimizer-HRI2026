# One downstream input for BOTH conditions. This is never execution permission.
BaselinePlan source
moveit_msgs/RobotTrajectory trajectory
bool optimization_applied
bool retiming_required
bool physical_validation_required
bool execution_ready
# Optimized trials include the solver diagnostics; baseline trials identify bypass.
string diagnostics
