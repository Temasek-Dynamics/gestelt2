from  gestelt_commander.scenario import *

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
        for i in range(0, 2):
            mission.pubGoals()
            mission.get_logger().info("Published goals to navigator")

        #########
        # Mission mode
        #########
        mission.cmdAllDronesPub(
            UAVCommand.Request.COMMAND_START_MISSION, 
            None,
            mode=0)
        mission.get_logger().info("All drones switching to MISSION MODE")

        #########
        # Wait for mission
        #########
        if not mission.waitForReqState(UAVState.MISSION, max_retries=20):
            raise Exception("Failed to transition to mission mode")

        rclpy.spin(mission)

    except Exception as e:
        print(e)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()