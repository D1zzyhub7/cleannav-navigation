# Structured Safety Supervisor state for programmatic consumers.

std_msgs/Header header
string<=16 interface_version
bool emergency_stop_active
bool autonomous_enabled
bool block_all
string<=64 lease_owner_execution_id
string<=256 message
