import time
import os
import json

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from geometry_msgs.msg import Pose
from gestelt_interfaces.msg import PlanRequest, Goals, UAVState
from gestelt_interfaces.srv import UAVCommand

class Scenario:
    """Scenario class that contains all the attributes of a scenario, used to start the fake_map
    and define the number of drones and their spawn positions
    """
    def __init__(self, filepath, scenario_name):
        with open(filepath) as f:
            json_dict = json.loads(f.read())

        scenario_dict = json_dict.get(scenario_name, None)

        if scenario_dict == None:
            raise Exception("Specified scenario does not exist!")

        self.name = scenario_name
        self.map = scenario_dict.get("map", None)
        self.spawns_pos = scenario_dict.get("spawns_pos", None )
        self.goals_pos = scenario_dict.get("goals_pos", None )
        self.num_agents = scenario_dict.get("num_agents", None )

        self.checks()

    def checks(self):
        if (len(self.spawns_pos) != self.num_agents):
            raise Exception("Number of spawn positions does not match number of agents!")

        if (len(self.goals_pos) != self.num_agents):
            raise Exception("Number of goal positions does not match number of agents!")

        if self.map == None or self.spawns_pos == None or self.goals_pos == None or self.num_agents == None:
            raise Exception("map_name and/or spawns_pos field does not exist!")

class Mission(Node):
    def __init__(self):
        super().__init__('mission_node')

        """ Parameter Server """
        self.declare_parameter('scenario', '')
        self.declare_parameter('init_delay', 1)

        self.scenario_name = self.get_parameter('scenario').get_parameter_value().string_value
        self.init_delay = self.get_parameter('init_delay').get_parameter_value().integer_value

        self.max_retries = 20

        # Sleep for init_delay
        time.sleep(self.init_delay)

        """ Read scenario JSON to get goal positions """
        self.scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
            self.scenario_name
        )

        '''Initialize data structures'''
        self.uav_states = []
        for id in range(self.scenario.num_agents):
            self.uav_states.append(UAVState.UNDEFINED)

        """Services"""
        self.uav_cmd_srv_clients_ = []
        for id in range(self.scenario.num_agents):
            self.uav_cmd_srv_clients_.append(self.create_client(
                                                UAVCommand, 
                                                'd' + str(id) + '/uav_command'))
            while not self.uav_cmd_srv_clients_[id].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Drone{id} UAV Command service not available, waiting again...')

        """Publishers"""
        self.goals_pubs_ = []
        for id in range(self.scenario.num_agents):
            self.goals_pubs_.append(self.create_publisher(
                                        Goals, 
                                        'd' + str(id) + '/goals', 10))

        """Subscribers"""
        # self.uav_state_subs_ = []
        # for id in range(self.scenario.num_agents):
        #     self.uav_state_subs_.append(self.create_subscription(
        #                                 UAVState, 
        #                                 'd' + str(id) + '/uav_state', 
        #                                 self.UAVStateSub, 10))

        """Set goals_pos"""
        self.plan_req_msgs = []
        for id in range(self.scenario.num_agents):
            self.plan_req_msgs.append(self.createGoalsMsg(self.scenario.goals_pos[id]))

        """Timers"""
        # self.plan_req_timer_ = self.create_timer(1.0, self.planReqTimerCB, autostart=True)

        for id in range(self.scenario.num_agents):
            while not self.isInReqState(id, UAVState.IDLE):
                time.sleep(0.5)

        print(f"ALL {self.scenario.num_agents} DRONES INITIALIZED!")

    # def UAVStateSub(self, msg):
    #     self.uav_states[msg.agent_id] = msg.state

    def sendUAVCommandReq(self, id, command, value, mode):
        srv_req = UAVCommand.Request()
        srv_req.command = command
        srv_req.value = value
        srv_req.mode = mode

        return self.uav_cmd_srv_clients_[id].call_async(srv_req)
    
    def isInReqState(self, id, req_state):
        '''Check if agent is in required state'''

        ret, msg = wait_for_message(
                    UAVState,
                    self,
                    'd' + str(id) + '/uav_state',
                    time_to_wait=1.0)
        
        if not ret:
            return False
        
        self.uav_states[id] = msg.state
        in_req_state = (self.uav_states[id] == req_state)

        # if (not in_req_state):
        #     self.get_logger().info(f'Drone{id} is in state {msg.state}. Not in required state {req_state}')

        return in_req_state

    def cmdAllDrones(self, req_state, command, value=0.0, mode=0):
        """_summary_

        Args:
            req_state (_type_): Required state before command can be sent
            command (_type_): Command to be sent
            value (float, optional): Value used for commands in takeoff mode
            mode (int, optional): Control mode used for mission 

        Returns:
            _type_: _description_
        """
        retry_num = 0
        
        self.get_logger().info(f"Commanding all drones: (cmd:{command}, value:{value}, mode:{mode})")
        
        for id in range(self.scenario.num_agents):
            while not self.isInReqState(id, req_state):
                time.sleep(0.5)
                retry_num += 1
                if (retry_num > self.max_retries):
                    return False

            self.get_logger().info(f"Sending UAV Command...")
            future = self.sendUAVCommandReq(id, command, value, mode)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if not response.success:
                self.get_logger().info(f'Failed to send command {command} to drone {id}')
                return False
            
            self.get_logger().info(f'Drone {id} transited to state {response.state_name}')

        return True

    # def createPlanReqMsg(self, id, start, goal):
    #     """Set goals_pos

    #     Args:
    #         start (list): [x,y,z]
    #         goal (list): [x,y,z]
    #     """

    #     def createPose(x, y, z):
    #         msg = Pose()
    #         msg.position.x = x
    #         msg.position.y = y
    #         msg.position.z = z

    #         return msg

    #     plan_req_msg = PlanRequest()
    #     plan_req_msg.header.stamp = self.get_clock().now().to_msg()
    #     plan_req_msg.header.frame_id = 'world'
    #     plan_req_msg.agent_id = id

    #     plan_req_msg.start = createPose(start[0], start[1], start[2])
    #     plan_req_msg.goal = createPose(goal[0], goal[1], goal[2])

    #     return plan_req_msg

    def createGoalsMsg(self, goal):
        """Set goals_pos

        Args:
            start (list): [x,y,z]
            goal (list): [x,y,z]
        """
        self.get_logger().info(f'Publishing mission goals to all drones')

        def createPose(x, y, z):
            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z

            return msg

        goal_msg = Goals()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'

        goal_msg.waypoints.append(createPose(goal[0], goal[1], goal[2]))

        return goal_msg

    
    # def planReqTimerCB(self):

    #     self.get_logger().info(f"Publishing plan request to all drones")

    #     # Publish to all drones
    #     for id in range(self.scenario.num_agents):
    #         self.goals_pubs_[id].publish(self.plan_req_msgs[id])

    #     # Timer only runs once
    #     if not self.plan_req_timer_.is_canceled(): 
    #         self.plan_req_timer_.cancel()

    def pubGoals(self):
        # Publish goals to all drones
        for id in range(self.scenario.num_agents):
            self.goals_pubs_[id].publish(self.plan_req_msgs[id])


def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    # Take off 
    mission.cmdAllDrones(UAVState.IDLE ,UAVCommand.Request.COMMAND_TAKEOFF, value=2.5)
    print("All drones switched to HOVER state")
    # Mission mode
    mission.cmdAllDrones(UAVState.HOVERING ,UAVCommand.Request.COMMAND_START_MISSION, mode=0)
    print("All drones switched to MISSION state")

    mission.pubGoals()

    # # Land mode
    # mission.cmdAllDrones(UAVState.MISSION ,UAVCommand.Request.COMMAND_LAND)

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()