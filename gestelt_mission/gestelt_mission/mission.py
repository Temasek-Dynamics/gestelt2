import time
import os
import json

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from gestelt_interfaces.msg import PlanRequest


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

        self.scenario_name = self.get_parameter('scenario').get_parameter_value().string_value

        """ Read scenario JSON to get goal positions """
        self.scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
            self.scenario_name
        )

        """Publishers and Subscribers"""
        self.plan_req_pub_ = []
        for id in range(self.scenario.num_agents):
            self.plan_req_pub_.append(self.create_publisher(PlanRequest, 
                                        'd'+ str(id) + '/plan_request_dbg', 10))

        """Set goals_pos"""
        self.plan_req_msgs = []
        for id in range(self.scenario.num_agents):
            self.plan_req_msgs.append(self.createPlanReqMsg(id, self.scenario.spawns_pos[id], self.scenario.goals_pos[id]))

        """Timers"""
        self.plan_req_timer_ = self.create_timer(1.0, self.planReqTimerCB, autostart=True)


    def createPlanReqMsg(self, id, start, goal):
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

        plan_req_msg = PlanRequest()
        plan_req_msg.header.stamp = self.get_clock().now().to_msg()
        plan_req_msg.header.frame_id = 'world'
        plan_req_msg.agent_id = id

        plan_req_msg.start = createPose(start[0], start[1], start[2])
        plan_req_msg.goal = createPose(goal[0], goal[1], goal[2])

        return plan_req_msg
    
    def planReqTimerCB(self):
        time.sleep(1)

        self.get_logger().info(f"Publishing plan request to all drones")

        # Publish to all drones
        for id in range(self.scenario.num_agents):
            self.plan_req_pub_[id].publish(self.plan_req_msgs[id])

        # Timer only runs once
        if not self.plan_req_timer_.is_canceled(): 
            self.plan_req_timer_.cancel()

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()