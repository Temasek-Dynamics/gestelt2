from gestelt_mission.scenario import *

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    # Landing
    mission.cmdAllDronesPub(
        UAVCommand.Request.COMMAND_TAKEOFF)
    print("All drones TAKING OFF")

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()