import time
import os
import json

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Empty

from gestelt_interfaces.msg import PlanRequest, Goals, UAVState, AllUAVCommand
from gestelt_interfaces.srv import UAVCommand

from gestelt_commander.robot_navigator import BasicNavigator, TaskResult

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
        self.send_goals = scenario_dict.get("send_goals", None )

        self.checks()

    def checks(self):
        if (len(self.spawns_pos) != self.num_agents):
            raise Exception("Number of spawn positions does not match number of agents!")

        if (len(self.goals_pos) != self.num_agents):
            raise Exception("Number of goal positions does not match number of agents!")

        if self.map == None or self.spawns_pos == None or self.goals_pos == None or self.num_agents == None:
            raise Exception("map_name and/or spawns_pos field does not exist!")

class MissionManager(Node):
    def __init__(self, no_scenario=False):
        super().__init__('mission_node')

        self.navigator = BasicNavigator(node_name='basic_navigator')

        """ Parameter Server """
        self.declare_parameter('scenario', 'single_drone_test')
        self.declare_parameter('init_delay', 2)
        self.declare_parameter('point_goal_height', 1.0)
        self.declare_parameter('plan_task_timeout', 60.0)
        self.declare_parameter('global_replanning_freq', 5.0)

        self.scenario_name = self.get_parameter('scenario').get_parameter_value().string_value
        self.init_delay = self.get_parameter('init_delay').get_parameter_value().integer_value
        self.point_goal_height = self.get_parameter('point_goal_height').get_parameter_value().double_value
        self.plan_task_timeout = self.get_parameter('plan_task_timeout').get_parameter_value().double_value
        self.global_replanning_freq = self.get_parameter('global_replanning_freq').get_parameter_value().double_value

        self.max_retries = 20

        self.get_logger().info(f"Initializing mission with scenario {self.scenario_name}...")

        # Sleep for init_delay
        time.sleep(self.init_delay)

        if (no_scenario):
            self.get_logger().info("No scenario required! Only initializing publisher to '/global_uav_command'")
            return

        """ 
        Read scenario JSON to get goal positions 
        """
        self.scenario = Scenario(
            os.path.join(get_package_share_directory('gestelt_commander'), 'scenarios.json'),
            self.scenario_name
        )
        """
        Subscribers
        """
        sub_point_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.pointGoalCallback, 10)
        sub_point_goal  # prevent unused variable warning

        """
        Publishers
        """
        # Publish to all UAVs
        self.uav_cmd_global_pub = self.create_publisher(
            AllUAVCommand, '/global_uav_command', rclpy.qos.qos_profile_services_default)

        # Publish to individual UAV
        self.uav_cmd_pubs = []
        for id in range(self.scenario.num_agents):
            self.uav_cmd_pubs.append(self.create_publisher(
                                    AllUAVCommand, '/d' + str(id) + '/uav_command', 
                                    rclpy.qos.qos_profile_services_default))

        # Publisher for resetting occupancy map and uav goals
        self.occ_map_pubs_ = []
        self.goals_pubs_ = []
        for id in range(self.scenario.num_agents):
            self.goals_pubs_.append(self.create_publisher(
                                    Goals, 'd' + str(id) + '/goals', 
                                    rclpy.qos.qos_profile_services_default))
            self.occ_map_pubs_.append(self.create_publisher(
                                    Empty, '/occ_map/reset_map', 
                                    rclpy.qos.qos_profile_services_default))

        """
        Services
        """
        # self.uav_cmd_srv_clients_ = []
        # for id in range(self.scenario.num_agents):
        #     self.uav_cmd_srv_clients_.append(self.create_client(
        #                                         UAVCommand, 
        #                                         'd' + str(id) + '/uav_command'))
        #     while not self.uav_cmd_srv_clients_[id].wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info(f'Drone{id} UAV Command service not available, waiting again...')

        """
        Set goals_pos
        """
        self.plan_req_msgs = []

        def createPose(x, y, z):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            return pose

        for id in range(self.scenario.num_agents):
            goal_msg = Goals()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal = self.scenario.goals_pos[id]
            goal_msg.waypoints.append(createPose(goal[0], goal[1], goal[2]))

            self.plan_req_msgs.append(self.createGoalsMsg(goal_msg))

        # Initialize data structures
        self.uav_states = []
        for id in range(self.scenario.num_agents):
            self.uav_states.append(UAVState.UNDEFINED)

        """
        Timers
        """
        # self.plan_req_timer_ = self.create_timer(1.0, self.planReqTimerCB, autostart=True)

        # self.get_logger().info(f"Waiting for /dX/uav_state topics...")
        # for id in range(self.scenario.num_agents):
        #     while not self.isInReqState(id, UAVState.IDLE):
        #         time.sleep(0.5)

        # self.get_logger().info(f"ALL {self.scenario.num_agents} DRONES INITIALIZED!")

    def sendUAVCommandReq(self, id, command, value, mode):
        srv_req = UAVCommand.Request()
        srv_req.command = command
        srv_req.value = value
        srv_req.mode = mode

        return self.uav_cmd_srv_clients_[id].call_async(srv_req)
    
    def isInReqState(self, id, req_state):
        '''Check if agent is in required state'''

        state_topic = '/d' + str(id) + '/uav_state'

        ret, msg = wait_for_message(
                    UAVState,
                    self,
                    state_topic,
                    time_to_wait=1.0)
        
        if ret:
            pass
            # self.get_logger().info(f"Received message {msg} from {state_topic}")
        else:
            self.get_logger().error(f"Did not receive message from {state_topic}")
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
                    
        self.get_logger().info(f"All drones in required state: {req_state} ...")
                    
        return True
    
    def cmdAllDronesPubGlobal(self, command, req_state=None, value=0.0, mode=0):
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
        
        self.get_logger().info(f"Commanding all drones via topic '/global_uav_command': (cmd:{command}, value:{value}, mode:{mode})")
        
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

        self.uav_cmd_global_pub.publish(msg)
        self.get_logger().info(f"Commanded all drones via '/global_uav_command': (cmd:{command}, value:{value}, mode:{mode})")

        return True

    def cmdAllDronesPubNamespaced(self, command, req_state=None, value=0.0, mode=0):
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
        
        self.get_logger().info(f"Commanding all drones via topic '/dx/uav_command': (cmd:{command}, value:{value}, mode:{mode})")
        
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

        for i in range(self.scenario.num_agents):
            for uav_cmd_pub in self.uav_cmd_pubs:
                uav_cmd_pub.publish(msg)

        self.get_logger().info(f"Commanded all drones via '/dx/uav_command': (cmd:{command}, value:{value}, mode:{mode})")

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
                time.sleep(1.0)
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

    def pubGoals(self):
        # Publish goals to all drones
        for id in range(self.scenario.num_agents):
            self.goals_pubs_[id].publish(self.plan_req_msgs[id])

    def resetOccMap(self):
        """Reset occupancy map
        """

        for id in range(self.scenario.num_agents):
            self.occ_map_pubs_[id].publish(Empty())

    def pointGoalCallback(self, msg):
        self.get_logger().info("Got point goal callback")
        ns = ''

        initial_pose = PoseStamped()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'world'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y
        goal_pose.pose.position.z = self.point_goal_height
        goal_pose.pose.orientation.w = 1.0

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active(navigator=ns+'/planner_server', localizer='robot_localization')
        # sanity check a valid path exists
        gbl_path = self.navigator.getPath(initial_pose, goal_pose, planner_id='GridBased', use_start=False)
        # Send global path to controller
        self.navigator.followPath(gbl_path)

        gbl_replan_rate = self.create_rate(self.global_replanning_freq)
        i = 0
        while not self.navigator.isTaskComplete():
            # TODO: Add replanning here

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds= self.plan_task_timeout):
                    self.navigator.cancelTask()
            
            gbl_replan_rate.sleep()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
