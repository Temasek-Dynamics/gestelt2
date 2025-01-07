import rclpy
from gestelt_interfaces.msg import AllUAVCommand

def main(args=None):
    rclpy.init(args=args)

    all_uav_cmd_pub = rclpy.create_publisher(
        AllUAVCommand, '/all_uav_command', rclpy.qos.qos_profile_services_default)

    msg = AllUAVCommand()
    msg.command = command
    msg.value = value
    msg.mode = mode

    all_uav_cmd_pub.publish(msg)


    rclpy.spin(mission)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()