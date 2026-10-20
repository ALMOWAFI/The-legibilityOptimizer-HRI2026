std_msgs/Header header
# SHA256 of canonical frame, blocks and goal_tool_z; see legibility_ros/layout.py.
string layout_id
Block[] blocks
# Tool-link hover height in header.frame_id, not cube-center height.
float64 goal_tool_z
