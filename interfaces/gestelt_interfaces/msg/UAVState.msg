# UAV State 
std_msgs/Header header  

int32 agent_id          # ID of agent 

uint8 UNCONNECTED = 0
uint8 IDLE = 1
uint8 LANDING = 2
uint8 TAKINGOFF = 3
uint8 HOVERING = 4
uint8 MISSION = 5
uint8 EMERGENCYSTOP = 6
uint8 UNDEFINED = 7

uint8 state # UAV State