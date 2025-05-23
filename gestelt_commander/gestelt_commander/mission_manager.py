#! /usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 John Tan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import time
import os

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Empty

from gestelt_interfaces.msg import Goals, UAVState, AllUAVCommand

from gestelt_commander.robot_navigator import BasicNavigator, TaskResult

from gestelt_commander.scenario import Scenario

class MissionManager(Node):
    def __init__(self, no_scenario=False):
        super().__init__('mission_node')

        """ Parameter Server """
        self.declare_parameter('scenario', 'single_drone_test')
        self.declare_parameter('init_delay', 2.0)
        self.declare_parameter('point_goal_height', 1.0)
        self.declare_parameter('global_replanning_freq', 5.0)

        self.scenario_name = self.get_parameter('scenario').get_parameter_value().string_value
        self.init_delay = self.get_parameter('init_delay').get_parameter_value().double_value
        self.point_goal_height = self.get_parameter('point_goal_height').get_parameter_value().double_value
        self.global_replanning_freq = self.get_parameter('global_replanning_freq').get_parameter_value().double_value

        self.max_retries = 20

        self.get_logger().info(f"Initializing mission manager with scenario {self.scenario_name}...")

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
            PoseStamped, '/point_goal', self.pointGoalCallback, 10)
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

        # Initialize data structures
        self.uav_states = []
        for id in range(self.scenario.num_agents):
            self.uav_states.append(UAVState.UNDEFINED)


        """
        Initialize navigator 
        """
        self.navigators = []
        for id in range(self.scenario.num_agents):
            self.navigators.append(BasicNavigator(node_name='basic_navigator', 
                                                  namespace='/d' + str(id)))

        self.navigator = BasicNavigator(node_name='basic_navigator', namespace='/d' + str(id))

        # Check if all drones are in IDLE state
        # self.get_logger().info(f"Waiting for /dX/uav_state topics...")
        # for id in range(self.scenario.num_agents):
        #     while not self.isInReqState(id, UAVState.IDLE):
        #         time.sleep(0.5)

        # self.get_logger().info(f"ALL {self.scenario.num_agents} DRONES INITIALIZED!")

        self.get_logger().info(f"Initialized mission manager")
    
    
    def cmdAllDronesPubGlobal(self, command, req_state=None, value=0.0, mode=0):
        """Command all drones without namespace

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
        """Command all drones with namespace

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


    def pubScenarioGoals(self):
        """Publish goals to all drones from scenario.json configuration
        """
        for id in range(self.scenario.num_agents):
            ns = '/d' + str(id)
            navigator = self.navigators[id]

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'world'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.scenario.goals_pos[id][0]
            goal_pose.pose.position.y = self.scenario.goals_pos[id][1]
            goal_pose.pose.position.z = self.scenario.goals_pos[id][2]
            goal_pose.pose.orientation.w = 1.0

            if not navigator.waitUntilNav2Active(navigator=ns+'/planner_server', localizer='robot_localization'):
                self.get_logger().error(f"Failed to activate {ns+'/planner_server'}, planning request aborted!")
                return

            # sanity check a valid path exists
            gbl_path = navigator.getPath(PoseStamped(), goal_pose, 
                                            planner_id='GridBased', use_start=False)
            # Request for controller to follow global path
            navigator.followPath(gbl_path)

    def pointGoalCallback(self, msg):
        """Callback on point goal topic from RVIZ
        """
        self.get_logger().info(f"Callback on point goal {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {self.point_goal_height:.2f}")
        
        ns = '/d0'
        navigator = self.navigators[0]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'world'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y
        goal_pose.pose.position.z = self.point_goal_height
        goal_pose.pose.orientation.w = 1.0

        # Wait for navigation to fully activate, since autostarting nav2
        # if not navigator.waitUntilNav2Active(navigator=ns+'/planner_server', localizer='robot_localization'):
        #     self.get_logger().error(f"Failed to activate {ns+'/planner_server'}, planning request aborted!")
        #     return

        # sanity check a valid path exists
        gbl_path = navigator.getPath(PoseStamped(), goal_pose, planner_id='GridBased', use_start=False)
        # Request for controller to follow global path
        navigator.followPath(gbl_path)

        # gbl_replan_rate = self.create_rate(self.global_replanning_freq)
        # i = 0
        # while not navigator.isTaskComplete():
        #     # TODO: Add replanning here

        #     # Do something with the feedback
        #     i = i + 1
        #     feedback = navigator.getFeedback()
        #     # navigator.cancelTask()
            
        #     gbl_replan_rate.sleep()

        # # Do something depending on the return code
        # result = navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        # elif result == TaskResult.FAILED:
        #     print('Goal failed!')
        # else:
        #     print('Goal has an invalid return status!')

    def resetOccMap(self):
        """Reset occupancy map
        """

        for id in range(self.scenario.num_agents):
            self.occ_map_pubs_[id].publish(Empty())

    def isInReqState(self, id, req_state):
        """Check if UAV with specified id is in required state

        Args:
            id (_type_): id of drone to be checked
            req_state (_type_): Required state before command can be sent
        """

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
        """Wait until the UAV state matches the required state

        Args:
            req_state (_type_): Required state before command can be sent
            max_retries (_type_): Maximum retries
        """
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