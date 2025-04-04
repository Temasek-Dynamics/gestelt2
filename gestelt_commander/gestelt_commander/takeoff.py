from gestelt_commander.scenario import *

def main(args=None):
    rclpy.init(args=args)

    mission_mngr = MissionManager()

    try: 
        #########
        # Take off 
        #########
        mission_mngr.cmdAllDronesPubNamespaced(
            UAVCommand.Request.COMMAND_TAKEOFF, 
            UAVState.IDLE,
            value=mission_mngr.scenario.take_off_height)
        mission_mngr.get_logger().info("All drones TAKING OFF")
        
        # #########
        # # Wait for Hover
        # #########
        # if not mission_mngr.waitForReqState(UAVState.HOVERING, max_retries=20):
        #     raise Exception("Failed to transition to hover mode")
        # mission_mngr.get_logger().info("All drones are in HOVER MODE.")
        # # Reset occupancy map
        # mission_mngr.resetOccMap()

        # time.sleep(2)

        # # Send a goal
        # mission_mngr.get_logger().info("Planning path.")
        # planPath(navigator)

        # #########
        # # MissionManager mode
        # #########
        # mission_mngr.cmdAllDronesPubNamespaced(
        #     UAVCommand.Request.COMMAND_START_MISSION, 
        #     UAVState.HOVERING,
        #     mode=0)
        # mission_mngr.get_logger().info("All drones swtching switching to MISSION MODE")

        # #########
        # # Wait for mission_mngr
        # #########
        # if not mission_mngr.waitForReqState(UAVState.MISSION, max_retries=20):
        #     raise Exception("Failed to transition to mission_mngr mode")
        
        # mission_mngr.get_logger().info("All drones in MISSION MODE")

        rclpy.spin(mission_mngr)

    except Exception as e:
        print(e)

    mission_mngr.destroy_node()
    # navigator.lifecycleShutdown()
    rclpy.shutdown()

    exit(0)
if __name__ == '__main__':
    main()