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
        # Wait for mission
        #########
        mission.get_logger().info("Checking if all drones are in MISSION MODE")
        
        if not mission.waitForReqState(UAVState.MISSION, max_retries=20):
            raise Exception("Failed to transition to mission mode")

        if mission.scenario.send_goals:
            mission.pubGoals()
            mission.get_logger().info("Published goals to navigator")
        else:
            mission.get_logger().info("Not publishing goals to navigator")

        rclpy.spin(mission)

    except Exception as e:
        print(e)

    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()