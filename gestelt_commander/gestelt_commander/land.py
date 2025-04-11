from gestelt_commander.scenario import *

def main(args=None):
    rclpy.init(args=args)

    mission_mngr = MissionManager()

    try: 
        # Landing
        mission_mngr.cmdAllDronesPubGlobal(
            UAVCommand.Request.COMMAND_LAND)
        mission_mngr.get_logger().info("All drones LANDING")

        rclpy.spin(mission_mngr)

    except Exception as e:
        mission_mngr.get_logger().info(e)

    mission_mngr.destroy_node()
    rclpy.shutdown()

    exit(0)

if __name__ == '__main__':
    main()