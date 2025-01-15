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
        print("All drones switched to HOVER state")
        
        # Mission mode
        mission.cmdAllDronesPub(
            UAVCommand.Request.COMMAND_START_MISSION, 
            UAVState.HOVERING,
            mode=0)
        print("All drones switched to MISSION state")

        mission.pubGoals()
        print("Published goals to navigator")

        # # Land mode
        # mission.cmdAllDronesPub(UAVState.MISSION ,UAVCommand.Request.COMMAND_LAND)

        rclpy.spin(mission)

    except Exception as e:
        print(e)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()