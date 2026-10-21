# Published ONLY by the external validator/executor, never by the optimizer.
# stamp = measured event time in the same ROS clock used for participant choices.
std_msgs/Header header
string command_id
string trial_id
string validation_id
# executing (measured arm onset), done (verified reach completion), home, failed
string state
string detail
