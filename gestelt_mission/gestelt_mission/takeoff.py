from gestelt_mission.scenario import *

def main(args=None):
    rclpy.init(args=args)

    take_off_height = 1.5

    mission = Mission(no_scenario=True)

    # Landing
    mission.cmdAllDronesPub(
        UAVCommand.Request.COMMAND_TAKEOFF,
        value=take_off_height)
    print("All drones TAKING OFF")

    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()