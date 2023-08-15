int32  state_id     # Unique 23-bit identifer for state
string path         # String path from top-level state machine
string[] children
string[] outcomes
string[] transitions
int8 type           # 0=State, StateMachine, PriorityContainer,ConcurrencyContainer
int8[] autonomy
