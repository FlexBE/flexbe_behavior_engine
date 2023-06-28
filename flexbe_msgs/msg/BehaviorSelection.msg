int32 behavior_key  # This is id key in behavior library
int32 behavior_id   # checksum id of particular behavior encoding

uint8 autonomy_level
string[] arg_keys
string[] arg_values

string[] input_keys
string[] input_values

flexbe_msgs/BehaviorModification[] modifications