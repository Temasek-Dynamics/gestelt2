from  gestelt_mission.scenario import *

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    # Take off 
    mission.cmdAllDronesPub(
        UAVState.IDLE,
        UAVCommand.Request.COMMAND_TAKEOFF, 
        value=mission.scenario.take_off_height)
    print("All drones switched to HOVER state")
    # Mission mode
    mission.cmdAllDronesPub(
        UAVState.HOVERING,
        UAVCommand.Request.COMMAND_START_MISSION, 
        mode=0)
    print("All drones switched to MISSION state")

    mission.pubGoals()

    # # Land mode
    # mission.cmdAllDronesPub(UAVState.MISSION ,UAVCommand.Request.COMMAND_LAND)

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()