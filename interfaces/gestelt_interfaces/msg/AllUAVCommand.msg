# Command sent to all UAVs
std_msgs/Header header  

# Command sent to server 
uint8 COMMAND_TAKEOFF = 0
uint8 COMMAND_LAND = 1
uint8 COMMAND_START_MISSION = 2
uint8 COMMAND_STOP_MISSION = 3
uint8 COMMAND_EMERGENCY_STOP = 4

uint8 command

# Value: Used for states [TakeOff]. Value which takes the context of the event. For Taking off, it is the height to take off to.
float64 value

# Mode: Used for states [Mission]. Used to determine the type of command input provided i.e. Position/Velocity/Acceleration, Thrust/Torque, Direct actuator control etc. 
uint8 MODE_TRAJECTORY = 0
uint8 MODE_ATTITUDE = 1
uint8 MODE_RATES = 2
uint8 MODE_THRUST_TORQUE = 3
uint8 MODE_MOTORS = 4

uint8 mode