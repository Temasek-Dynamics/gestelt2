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

import rclpy
from rclpy.executors import ExternalShutdownException

from gestelt_commander.mission_manager import MissionManager
from gestelt_interfaces.msg import UAVState, AllUAVCommand

def main(args=None):
    rclpy.init(args=args)

    mission_mngr = MissionManager()

    try:
        #########
        # Take off 
        #########
        mission_mngr.cmdAllDronesPubNamespaced(
            AllUAVCommand.COMMAND_TAKEOFF, 
            UAVState.IDLE,
            value=mission_mngr.scenario.take_off_height)
        mission_mngr.get_logger().info("Sending commands to TAKE OFF")
        
        #########
        # Wait for Hover
        #########
        if not mission_mngr.waitForReqState(UAVState.HOVERING, max_retries=40):
            raise Exception("Failed to transition to hover mode")
        mission_mngr.get_logger().info("All drones are in HOVER MODE.")
        # Reset occupancy map
        mission_mngr.resetOccMap()

        #########
        # MissionManager mode
        #########
        mission_mngr.cmdAllDronesPubNamespaced(
            AllUAVCommand.COMMAND_START_MISSION, 
            UAVState.HOVERING,
            mode=0)
        mission_mngr.get_logger().info("All drones swtching switching to MISSION MODE")

        #########
        # Wait for mission_mngr
        #########
        if not mission_mngr.waitForReqState(UAVState.MISSION, max_retries=40):
            raise Exception("Failed to transition to mission mode")
        
        mission_mngr.get_logger().info("All drones in MISSION MODE. Ready to execute goals.")

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(mission_mngr)

        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        mission_mngr.destroy_node()
        rclpy.shutdown()
    
    exit(0)

if __name__ == '__main__':
    main()
