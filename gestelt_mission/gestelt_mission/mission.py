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
        # Take off 
        mission.cmdAllDronesPub(
            UAVCommand.Request.COMMAND_TAKEOFF, 
            UAVState.IDLE,
            value=mission.scenario.take_off_height)
        mission.get_logger().info("All drones TAKING OFF")
        
        # Mission mode
        mission.cmdAllDronesPub(
            UAVCommand.Request.COMMAND_START_MISSION, 
            UAVState.HOVERING,
            mode=0)
        mission.get_logger().info("All drones swtching switching to MISSION MODE")

        mission.pubGoals()
        mission.get_logger().info("Published goals to navigator")

        # # Land mode
        # mission.cmdAllDronesPub(UAVState.MISSION ,UAVCommand.Request.COMMAND_LAND)

        rclpy.spin(mission)

    except Exception as e:
        print(e)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()