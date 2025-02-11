import time
import os
import json

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from geometry_msgs.msg import Pose
from gestelt_interfaces.msg import PlanRequest, Goals, UAVState, AllUAVCommand
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
        self.take_off_height = scenario_dict.get("take_off_height", None )

        self.checks()

    def checks(self):
        if (len(self.spawns_pos) != self.num_agents):
            raise Exception("Number of spawn positions does not match number of agents!")

        if (len(self.goals_pos) != self.num_agents):
            raise Exception("Number of goal positions does not match number of agents!")

        if self.map == None or self.spawns_pos == None or self.goals_pos == None or self.num_agents == None:
            raise Exception("map_name and/or spawns_pos field does not exist!")

class Mission(Node):
    def __init__(self, no_scenario=False):
        super().__init__('mission_node')

        """ Parameter Server """
        self.declare_parameter('scenario', '')
        self.declare_parameter('init_delay', 2)

        self.scenario_name = self.get_parameter('scenario').get_parameter_value().string_value
        self.init_delay = self.get_parameter('init_delay').get_parameter_value().integer_value

        self.max_retries = 20

        self.get_logger().info(f"Initializing mission with scenario {self.scenario_name}...")

        # Sleep for init_delay
        time.sleep(self.init_delay)

        """Publisher to all UAVs"""
        self.all_uav_cmd_pub = self.create_publisher(
            AllUAVCommand, '/all_uav_command', 
            rclpy.qos.qos_profile_services_default)

        if (no_scenario):
            self.get_logger().info("No scenario required! Only initializing publisher to '/all_uav_command'")
            return

        """ Read scenario JSON to get goal positions """
        self.scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
            self.scenario_name
        )

        '''Initialize data structures'''
        self.uav_states = []
        for id in range(self.scenario.num_agents):
            self.uav_states.append(UAVState.UNDEFINED)

        """Services"""
        # self.uav_cmd_srv_clients_ = []
        # for id in range(self.scenario.num_agents):
        #     self.uav_cmd_srv_clients_.append(self.create_client(
        #                                         UAVCommand, 
        #                                         'd' + str(id) + '/uav_command'))
        #     while not self.uav_cmd_srv_clients_[id].wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info(f'Drone{id} UAV Command service not available, waiting again...')

        """Publishers"""

        self.goals_pubs_ = []
        for id in range(self.scenario.num_agents):
            self.goals_pubs_.append(self.create_publisher(
                                    Goals, 'd' + str(id) + '/goals', 
                                    rclpy.qos.qos_profile_services_default))

        """Set goals_pos"""
        self.plan_req_msgs = []
        for id in range(self.scenario.num_agents):
            self.plan_req_msgs.append(self.createGoalsMsg(self.scenario.goals_pos[id]))

        """Timers"""
        # self.plan_req_timer_ = self.create_timer(1.0, self.planReqTimerCB, autostart=True)

        self.get_logger().info(f"Waiting for /dX/uav_state topics...")
        for id in range(self.scenario.num_agents):
            while not self.isInReqState(id, UAVState.IDLE):
                time.sleep(0.5)

        self.get_logger().info(f"ALL {self.scenario.num_agents} DRONES INITIALIZED!")

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

        # if (not in_req_state):
        #     self.get_logger().info(f'Drone{id} is in state {msg.state}. Not in required state {req_state}')

        return (self.uav_states[id] == req_state)

    def waitForReqState(self, req_state, max_retries=20):
        retry_num = 0
        self.get_logger().info(f"Waiting for required state: {req_state} ...")

        if req_state != None:
            for id in range(self.scenario.num_agents):
                while not self.isInReqState(id, req_state):
                    time.sleep(1.0)
                    retry_num += 1
                    if (retry_num > max_retries):
                        self.get_logger().info(f"Failed to wait for required state: {req_state} ...")
                        return False
                    
        self.get_logger().info(f"Waiting for required state: {req_state} ...")
                    
        return True

    def cmdAllDronesPub(self, command, req_state=None, value=0.0, mode=0):
        """Command all drones using publisher

        Args:
            req_state (_type_): Required state before command can be sent
            command (_type_): Command to be sent
            value (float, optional): Value used for commands in takeoff mode
            mode (int, optional): Control mode used for mission 

        Returns:
            _type_: _description_
        """
        retry_num = 0
        
        self.get_logger().info(f"Commanding all drones via topic '/all_uav_command': (cmd:{command}, value:{value}, mode:{mode})")
        
        # Publish to all drones
        # Wait for drone states to be req_state before triggering state transition
        if req_state != None:
            for id in range(self.scenario.num_agents):
                while not self.isInReqState(id, req_state):
                    time.sleep(1.0)
                    retry_num += 1
                    if (retry_num > self.max_retries):
                        return False

        msg = AllUAVCommand()
        msg.command = command
        msg.value = value
        msg.mode = mode

        self.all_uav_cmd_pub.publish(msg)
        self.get_logger().info(f"Commanded all drones via '/all_uav_command': (cmd:{command}, value:{value}, mode:{mode})")

        return True


    def cmdAllDronesSrv(self, req_state, command, value=0.0, mode=0):
        """Command all drones using publisher

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

        def createPose(x, y, z):
            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z

            return msg

        goal_msg = Goals()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.waypoints.append(createPose(goal[0], goal[1], goal[2]))

        return goal_msg


    def pubGoals(self):
        # Publish goals to all drones
        for id in range(self.scenario.num_agents):
            self.goals_pubs_[id].publish(self.plan_req_msgs[id])