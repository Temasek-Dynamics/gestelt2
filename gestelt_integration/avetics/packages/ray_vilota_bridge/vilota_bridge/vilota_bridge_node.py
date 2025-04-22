import multiprocessing, ctypes

# External libraries
import ecal.core.core as ecal_core

from vilota_bridge.vilota_bridge import VilotaBridge

import rclpy
from rclpy.executors import MultiThreadedExecutor

def main():
    running = multiprocessing.Value(ctypes.c_bool, True)

    # Init eCAL
    print(f"eCAL {ecal_core.getversion()} ({ecal_core.getdate()})\n")

    rclpy.init()

    mtExecutor = MultiThreadedExecutor()

    depthBackRightBridge = VilotaBridge(running, "back_right", "S1/stereo1_l/disparity", "S1/stereo1_l", "S1/vio_odom")
    depthBackLeftBridge = VilotaBridge(running, "back_left", "S1/stereo2_r/disparity", "S1/stereo2_r", "S1/vio_odom")

    mtExecutor.add_node(depthBackRightBridge)
    mtExecutor.add_node(depthBackLeftBridge)

    while running.value and rclpy.ok() and ecal_core.ok():
        try:
            mtExecutor.spin_once()
        except IndexError:
            pass

    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
