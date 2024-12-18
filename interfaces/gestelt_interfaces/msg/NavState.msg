# Navigator State 
std_msgs/Header header  

int32 agent_id          # ID of agent 

uint8 IDLE = 0
uint8 PLANNING = 1
uint8 PLANNING_TIMEOUT = 2
uint8 UNDEFINED = 7

uint8 state # Navigator State