from  gestelt_mission.scenario import *

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    # Landing
    mission.cmdAllDronesPub(
        UAVCommand.Request.COMMAND_LAND)
    print("All drones LANDING")

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()