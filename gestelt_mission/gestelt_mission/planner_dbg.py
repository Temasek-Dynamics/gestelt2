import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from gestelt_interfaces.msg import PlanRequest

class Mission(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.plan_req_pub_ = self.create_publisher(PlanRequest, 
                                                    '/plan_request_dbg', 10)

        self.plan_req_timer_ = self.create_timer(1.0, self.planReqTimerCB, autostart=False)
        self.plan_req_msg = None



    def setPlanReq(self, start, goal):
        """Set goals

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

        self.plan_req_msg = PlanRequest()
        self.plan_req_msg.header.stamp = self.get_clock().now().to_msg()
        self.plan_req_msg.header.frame_id = 'world'
        self.plan_req_msg.agent_id = 0

        self.plan_req_msg.start = createPose(start[0], start[1], start[2])
        self.plan_req_msg.goal = createPose(goal[0], goal[1], goal[2])
    
    def startTimer(self):
        self.plan_req_timer_.reset()

    def planReqTimerCB(self):

        time.sleep(1)

        self.get_logger().info(f"Publishing plan request from \
                               ({self.plan_req_msg.start.position.x}, {self.plan_req_msg.start.position.y}, {self.plan_req_msg.start.position.z}) \
                               to ({self.plan_req_msg.goal.position.x}, {self.plan_req_msg.goal.position.y}, {self.plan_req_msg.goal.position.z})")

        self.plan_req_pub_.publish(self.plan_req_msg)

        # Timer only runs once
        if not self.plan_req_timer_.is_canceled(): 
            self.plan_req_timer_.cancel()

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    mission.setPlanReq([-6.0, -4.5, 1.0], 
                     [6.0, 4.5, 2.0])
    
    mission.startTimer()

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()