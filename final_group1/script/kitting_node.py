#!/usr/bin/env python3

"""
This script initializes the Kitting Node.
"""

import rclpy
from final_group1.kitting_interface import KittingNode
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    """
    Main function to initialize the Kitting Node and add to MultiThreaded Executor.
    Args:
        args (list, optional): Command-line arguments. Default: None.

    This function performs the following steps:
    1. Initializes ROS2 client library with the provided arguments.
    2. Creates an instance of Kitting Node.
    3. Adds this instance to Multi-Threaded Executor.
    4. Keeps it spinning till a shutdown signal is received.
    5. Destroys the node and cleanly shuts down ROS2 system.
    """
    rclpy.init(args=args)
    kitting_node = KittingNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(kitting_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        kitting_node._logger("Keyboard Interrupt (Ctrl+C) received. Shutting down...")
    finally:
        executor.shutdown()
        kitting_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    