#!/usr/bin/env python3

"""
This script initializes the Main Node.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from final_group1.competition_interface import CompetitionInterface

def main(args=None):
    """
    Main function to initialize the Main Node and add to MultiThreaded Executor.
    Args:
        args (list, optional): Command-line arguments. Default: None.

    This function performs the following steps:
    1. Initializes ROS2 client library with the provided arguments.
    2. Creates an instance of Main Node.
    3. Adds this instance to Multi-Threaded Executor.
    4. Start the competition.
    5. Keeps it spinning till a shutdown signal is received.
    6. Destroys the node and cleanly shuts down ROS2 system.
    """
    rclpy.init(args=args)
    control_center = CompetitionInterface()
    control_center.start_competition_cb()

    executor = MultiThreadedExecutor()
    executor.add_node(control_center)

    try:
        executor.spin()
    except KeyboardInterrupt:
        control_center._logger("Keyboard Interrupt (Ctrl+C) received. Shutting down...")
    finally:
        executor.shutdown()
        control_center.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
