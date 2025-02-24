from  gestelt_mission.scenario import *

def main(args=None):
    rclpy.init(args=args)

    # user_input = input("Awaiting user input to start MISSION. y/n?")

    # if user_input.lower() == "y":
    #     print("User has given the go!")
    # else:
    #     print("User has aborted mission! Exiting...")
    #     return
    mission = Mission()

    try: 
        #########
        # Take off 
        #########
        mission.cmdAllDronesPub(
            UAVCommand.Request.COMMAND_TAKEOFF, 
            UAVState.IDLE,
            value=mission.scenario.take_off_height)
        mission.get_logger().info("All drones TAKING OFF")
        
        #########
        # Wait for Hover
        #########
        if not mission.waitForReqState(UAVState.HOVERING, max_retries=20):
            raise Exception("Failed to transition to hover mode")

        mission.get_logger().info("All drones are in HOVER MODE")

        if mission.scenario.send_goals:

            for i in range(0, 2):
                mission.pubGoals()
                mission.get_logger().info("Published goals to navigator")

            #########
            # Mission mode
            #########
            mission.cmdAllDronesPub(
                UAVCommand.Request.COMMAND_START_MISSION, 
                UAVState.HOVERING,
                mode=0)
            mission.get_logger().info("All drones swtching switching to MISSION MODE")

            #########
            # Wait for mission
            #########
            if not mission.waitForReqState(UAVState.MISSION, max_retries=20):
                raise Exception("Failed to transition to mission mode")

        else:
            mission.get_logger().info("Not transitioning to MISSION mode as defined by user config.")

        rclpy.spin(mission)

    except Exception as e:
        print(e)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()