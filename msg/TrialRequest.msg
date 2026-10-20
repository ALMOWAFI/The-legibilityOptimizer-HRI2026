std_msgs/Header header
# Unique within a running session. Duplicate IDs are rejected.
string trial_id
int32 target_id
# Exactly baseline or legible. Baseline bypasses optimization.
string condition
# Empty means bind to the latest received layout when this trial is accepted.
string layout_id
