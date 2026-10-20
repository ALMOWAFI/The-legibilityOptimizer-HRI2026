# Includes the original trajectory, all goals and the exact trial/layout snapshot.
BaselinePlan source
moveit_msgs/RobotTrajectory trajectory
bool solver_success
bool numerical_accepted
# Both true; execution_ready is always false. Downstream must retime and validate.
bool retiming_required
bool physical_validation_required
bool execution_ready
float64 baseline_legibility
float64 optimized_legibility
float64 baseline_joint_cost
float64 optimized_joint_cost
string diagnostics
