#!/usr/bin/python3

"""
This script defines a ROS2 node for Kitting Process.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ariac_msgs.srv import MoveAGV
from std_srvs.srv import Trigger
from ariac_msgs.srv import SubmitOrder, ChangeGripper, VacuumGripperControl, PerformQualityCheck
from ariac_msgs.msg import AGVStatus, AdvancedLogicalCameraImage, Part, VacuumGripperState
from ariac_msgs.msg import Order, KittingPart
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import PyKDL as kdl
from collections import defaultdict
import threading
import traceback


# Import custom ROS services
from robot_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    MoveRobotToPart,
    MovePartToAGV,
    DropTray,
    DropPart

)


class CustomOrder:
    """
    Class to represent the order msg used in the node.
    Attributes:
        order_id (str): Unique identifier for the order.
        status (str): Current status of the order.
        priority (int): Priority level of the order.
        time_elapsed (int): Time elapsed since receiving the order [seconds].
        agv_num (int): Number of the AGV assigned to the order.
        tray_id (int): ID of the tray associated with the order.
        tray_locked (bool): Flag indicating whether the tray is locked.
        shipped (bool): Flag indicating whether the order has been shipped.
        submitted (bool): Flag indicating whether the order has been submitted for processing.
        processed (bool): Flag indicating whether the order has been processed.
        type (str): Type of the order (e.g., "pick", "place").
        destination (str): Destination location for the order.
    """

    def __init__(self, order_id, priority):
        """
        Initialize the CustomOrder class.
        Args:
            order_id (str): Unique identifier for the order.
            priority (int): Priority level of the order.
        """
        self.order_id = order_id
        self.status = "pending"  # Initial status
        self.priority = priority
        self.time_elapsed = 0
        self.agv_num = 0
        self.tray_id = 0
        self.quadrant = 0
        self.tray_locked = False
        self.shipped = False
        self.submitted = False
        self.processed = False
        self.type = None
        self.parts = []
        self.parts_placed = []
        self.faulty_parts = []
        self.part_picked = False
        self.current_part = None
        self.missed_part = None
        self.color = None
        self.destination = None
        self.tray_lock_action = False
        self.agv_move_action = False
        self.submit_action = False
        self.working = False
        self.current_tray = None
        self.discard_part = False
        self.discarded_part = False
        # order status flags
        self.grip_tray = False
        self.move_tray = False
        self.grip_part = False
        self.move_part = False

        self._moving_robot_to_part = False
        self._moving_part_to_agv = False
        self._dropping_part = False
        self.faulty_gripper_detected = False
        self.solve_faulty_gripper = False

        # The following flags are used to ensure an action is not triggered multiple times
        self._moving_robot_home = False
        self._moving_robot_to_table = False
        self._entering_tool_changer = False
        self._changing_gripper = False
        self._exiting_tool_changer = False
        self._activating_gripper = False
        self._deactivating_gripper = False
        self._moving_robot_to_tray = False
        self._moving_tray_to_agv = False

        # The following flags are used to trigger the next action
        self._moved_robot_home = False
        self._moved_robot_to_table = False
        self._entered_tool_changer = False
        self._changed_gripper = False
        self._exited_tool_changer = False
        self._activated_gripper = False
        self._deactivated_gripper = False
        self._moved_robot_to_tray = False
        self._moved_tray_to_agv = False
        self._moved_robot_to_part = False


class KittingNode(Node):
    """
    Class to represent a kitting node that interfaces with the warehouse and AGV.
    """

    _part_properties = {
        'color': {
            Part.RED: 'Red',
            Part.GREEN: 'Green',
            Part.BLUE: 'Blue',
            Part.ORANGE: 'Orange',
            Part.PURPLE: 'Purple'
        },
        'type': {
            Part.BATTERY: 'Battery',
            Part.PUMP: 'Pump',
            Part.SENSOR: 'Sensor',
            Part.REGULATOR: 'Regulator'
        }
    }

    def __init__(self):
        super().__init__('kitting_node')
        self._logger = self.get_logger()

        try:
            self._logger.info('Kitting node started!')

            # Create multithreaded executor callback groups for managing subscriptions, timers, and services
            self._group = ReentrantCallbackGroup()
            self._group2 = None
            self._group3 = ReentrantCallbackGroup()

            # Create a publisher to indicate if orders are submitted
            self._order_submitted_publisher = self.create_publisher(
                Bool,
                '/submitted_orders',
                10,
                callback_group=self._group
            )

            '''
            ------------- Subscriber -------------------
            '''
            # Create a subscriber to receive orders
            self._order_subscriber = self.create_subscription(
                Order,
                '/ariac/orders',
                self.ariac_order_cb,
                QoSProfile(depth=10),
                callback_group=self._group
            )

            # Create a subscriber to receive the camera images
            self._left_tray_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/tray_camera_2/image',
                self._tray_camera_2_cb,
                qos_profile=rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                callback_group=self._group
            )

            self._right_tray_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/tray_camera_1/image',
                self._tray_camera_1_cb,
                qos_profile=rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                callback_group=self._group
            )

            self._left_bin_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/bin_camera_2/image',
                self._bin_camera_2_cb,
                qos_profile=rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                callback_group=self._group
            )
            self._right_bin_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/bin_camera_1/image',
                self._bin_camera_1_cb,
                qos_profile=rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                callback_group=self._group
            )
            # Create a subscriber to check the state of the vacuum gripper
            self.check_faulty_gripper_subscriber = self.create_subscription(
                VacuumGripperState,
                '/ariac/floor_robot_gripper_state',
                self.check_faulty_gripper_cb,
                qos_profile=rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                callback_group=self._group
            )

            '''
            ------------- Clients -------------------
            '''

            # client to check faulty part
            self._check_faulty_part_cli = self.create_client(
                PerformQualityCheck,
                '/ariac/perform_quality_check')

            # client to move the floor robot to the home position
            self._move_robot_home_cli = self.create_client(
                Trigger,
                "/commander/move_robot_home")

            # client to move a robot to a table
            self._move_robot_to_table_cli = self.create_client(
                MoveRobotToTable,
                '/commander/move_robot_to_table')

            # client to move a robot to a table
            self._move_robot_to_tray_cli = self.create_client(
                MoveRobotToTray,
                '/commander/move_robot_to_tray')

            # client to move a tray to an agv
            self._move_tray_to_agv_cli = self.create_client(
                MoveTrayToAGV,
                '/commander/move_tray_to_agv')

            # client to move a robot to a part
            self._move_robot_to_part_cli = self.create_client(
                MoveRobotToPart,
                '/commander/move_robot_to_part')

            # client to move a part to a AGV
            self._move_part_to_agv_cli = self.create_client(
                MovePartToAGV,
                '/commander/move_part_to_agv')

            # client to move the end effector inside a tool changer
            self._enter_tool_changer_cli = self.create_client(
                EnterToolChanger,
                '/commander/enter_tool_changer')

            # client to move the end effector outside a tool changer
            self._exit_tool_changer_cli = self.create_client(
                ExitToolChanger,
                '/commander/exit_tool_changer')

            # client to activate/deactivate the vacuum gripper
            self._set_gripper_state_cli = self.create_client(
                VacuumGripperControl,
                '/ariac/floor_robot_enable_gripper')

            # client to change the gripper type
            # the end effector must be inside the tool changer before calling this service
            self._change_gripper_cli = self.create_client(
                ChangeGripper,
                '/ariac/floor_robot_change_gripper')

            self._drop_part_cli = self.create_client(
                DropPart,
                '/commander/drop_part')

            self._drop_tray_cli = self.create_client(
                DropTray,
                '/commander/drop_tray')

            '''
            ------------- Timer -------------------
            '''
            # Timer for periodic tasks
            self._submit_order_status = 0
            self._timer = self.create_timer(
                2, self.process_orders, callback_group=self._group3)

            # Initialize variables
            self._orders_queue = []
            self._paused_orders = []
            self._trays = defaultdict(list)
            self._bins = defaultdict(list)
            self._current_order = None
            self._order_lock = threading.Lock()

            self._bin_camera_2_flag = False
            self._bin_camera_1_flag = False
            self._tray_camera_2_flag = False
            self._tray_camera_1_flag = False
            self.post_discard_flag = False

            self.current_gripper = None
            self.current_task = None
            self._part = None

            self.get_logger().info("Kitting Inteface Node has been initialised.")

        except Exception as e:
            self._logger.error(
                f"Failed while initializing Class KittingNode. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def find_transform(self, part_pose_in_camera_frame, camera_pose_in_world_frame):
        """
        Function to find the transform of the part pose in the world frame.
        Args:
            part_pose_in_camera_frame (geometry_msgs/msg/Pose): Pose of the part in camera frame.
            camera_pose_in_world_frame (geometry_msgs/msg/Pose): Pose of the camera in world frame.
        Returns:
            _part_pose (geometry_msgs/msg/Pose): Pose of the part in world frame.
        """
        try:
            # First frame: Camera in World frame
            _camera_orientation = camera_pose_in_world_frame.orientation
            _camera_x = camera_pose_in_world_frame.position.x
            _camera_y = camera_pose_in_world_frame.position.y
            _camera_z = camera_pose_in_world_frame.position.z

            _frame_camera_world = kdl.Frame(
                kdl.Rotation.Quaternion(
                    _camera_orientation.x,
                    _camera_orientation.y,
                    _camera_orientation.z,
                    _camera_orientation.w,
                ),
                kdl.Vector(
                    _camera_x,
                    _camera_y,
                    _camera_z
                ),
            )

            # Second frame: Part in Camera Frame
            _part_orientation = part_pose_in_camera_frame.orientation
            _part_x = part_pose_in_camera_frame.position.x
            _part_y = part_pose_in_camera_frame.position.y
            _part_z = part_pose_in_camera_frame.position.z

            _frame_part_camera = kdl.Frame(
                kdl.Rotation.Quaternion(
                    _part_orientation.x,
                    _part_orientation.y,
                    _part_orientation.z,
                    _part_orientation.w,
                ),
                kdl.Vector(
                    _part_x,
                    _part_y,
                    _part_z
                ),
            )

            # Multiply the two frames
            _frame_part_world = _frame_camera_world * _frame_part_camera

            # Return the resulting pose - Part in World Frame
            _part_pose = Pose()
            _part_quaternion = _frame_part_world.M.GetQuaternion()

            _part_pose.position.x = round(_frame_part_world.p.x(), 6)
            _part_pose.position.y = round(_frame_part_world.p.y(), 6)
            _part_pose.position.z = round(_frame_part_world.p.z(), 6)
            _part_pose.orientation.x = round(_part_quaternion[0], 3)
            _part_pose.orientation.y = round(_part_quaternion[1], 3)
            _part_pose.orientation.z = round(_part_quaternion[2], 3)
            _part_pose.orientation.w = round(_part_quaternion[3], 3)

            return _part_pose

        except Exception as e:
            self._logger.error(
                f"Error occurred while finding transforms. Error code: {e}")
            self._logger.error(traceback.format_exc())
            return None

    def quaternion_to_euler(self, quaternion):
        """
        Function to convert quaternion to euler angles.
        Args:
            quaternion (geometry_msgs/msg/Quaternion): Quaternion to be converted.

        Returns:
            rpy (list): List of euler angles in radians.
        """
        try:
            rpy = kdl.Rotation.Quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w).GetRPY()
            rpy = [
                round(rpy[0], 1),
                round(rpy[1], 1),
                round(rpy[2], 2)
            ]
            return rpy
        except Exception as e:
            self._logger.error(
                f"Error occurred while converting quaternion to euler angles. Error code: {e}")
            self._logger.error(traceback.format_exc())
            return None

    def _tray_camera_1_cb(self, msg):
        """
        Callback function to handle the incoming  tray camera 1 images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming  tray camera 1 image.
        """
        try:
            _tray_poses = msg.tray_poses
            _camera_pose = msg.sensor_pose
            if not self._tray_camera_1_flag:
                if len(_tray_poses) > 0:
                    for _tray_pose in _tray_poses:
                        _tray_id = _tray_pose.id
                        _world_tray_pose = self.find_transform(
                            _tray_pose.pose, _camera_pose)
                        _euler = self.quaternion_to_euler(
                            _world_tray_pose.orientation)

                        self._trays[_tray_id].append({
                            'position': [
                                _world_tray_pose.position.x,
                                _world_tray_pose.position.y,
                                _world_tray_pose.position.z
                            ],
                            'orientation': [
                                _euler[0],
                                _euler[1],
                                _euler[2]
                            ],
                            'pose': _world_tray_pose,
                            'camera': 1
                        })
                self._tray_camera_1_flag = True
        except Exception as e:
            self._logger.error(
                f"Error occurred while handling incoming  tray camera 2 image data. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _tray_camera_2_cb(self, msg):
        """
        Callback function to handle the incoming  tray camera 2 images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming  tray camera 2 image.
        """
        try:
            _tray_poses = msg.tray_poses
            _camera_pose = msg.sensor_pose

            if not self._tray_camera_2_flag:
                if len(_tray_poses) > 0:
                    for _tray_pose in _tray_poses:
                        _tray_id = _tray_pose.id
                        _world_tray_pose = self.find_transform(
                            _tray_pose.pose, _camera_pose)
                        _euler = self.quaternion_to_euler(
                            _world_tray_pose.orientation)

                        self._trays[_tray_id].append({
                            'position': [
                                _world_tray_pose.position.x,
                                _world_tray_pose.position.y,
                                _world_tray_pose.position.z
                            ],
                            'orientation': [
                                _euler[0],
                                _euler[1],
                                _euler[2]
                            ],
                            'pose': _world_tray_pose,
                            'camera': 2
                        })
                self._tray_camera_2_flag = True
        except Exception as e:
            self._logger.error(
                f"Error occurred while handling incoming  tray camera 2 image data. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _bin_camera_1_cb(self, msg):
        """
        Callback function to handle the incoming  bin camera 1 images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming  bin camera 1 image.
        """
        try:
            if not self._bin_camera_1_flag:
                _bin_poses = msg.part_poses
                for _bin_pose in _bin_poses:
                    _bin_part = _bin_pose.part
                    _bin_part_pose = Pose()
                    _bin_part_pose = _bin_pose.pose

                    _world_bin_pose = self.find_transform(
                        _bin_part_pose, msg.sensor_pose)

                    _euler = self.quaternion_to_euler(
                        _world_bin_pose.orientation)
                    self._bins[(_bin_part.type, _bin_part.color)].append(
                        {
                            'position': [
                                _world_bin_pose.position.x,
                                _world_bin_pose.position.y,
                                _world_bin_pose.position.z
                            ],
                            'orientation': [
                                _euler[0],
                                _euler[1],
                                _euler[2]
                            ],
                            'pose': _world_bin_pose,
                            'bin': "left_bin",
                            'picked': False
                        }
                    )

                self._bin_camera_1_flag = True

        except Exception as e:
            self._logger.error(
                f"Error occured while handling incoming  bin camera 1 image data. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _bin_camera_2_cb(self, msg):
        """
        Callback function to handle the incoming  bin camera 2 images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming  bin camera 2 image.
        """
        try:
            if not self._bin_camera_2_flag:
                _bin_poses = msg.part_poses
                for _bin_pose in _bin_poses:
                    _bin_part = _bin_pose.part
                    _bin_part_pose = Pose()
                    _bin_part_pose = _bin_pose.pose

                    _world_bin_pose = self.find_transform(
                        _bin_part_pose, msg.sensor_pose)

                    _euler = self.quaternion_to_euler(
                        _world_bin_pose.orientation)
                    self._bins[(_bin_part.type, _bin_part.color)].append(
                        {
                            'position': [
                                _world_bin_pose.position.x,
                                _world_bin_pose.position.y,
                                _world_bin_pose.position.z
                            ],
                            'orientation': [
                                _euler[0],
                                _euler[1],
                                _euler[2]
                            ],
                            'pose': _world_bin_pose,
                            'bin': "right_bin",
                            'picked': False

                        }
                    )

                self._bin_camera_2_flag = True
        except Exception as e:
            self._logger.error(
                f"Error occured while handling incoming  bin camera 2 image data. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def ship_and_submit_order(self):
        """
        Function to start the process of shipping and submitting the order.
        """
        try:

            if self._current_order.tray_locked and not self._current_order.shipped and not self._current_order.agv_move_action:
                self.move_agv(self._current_order)
                self._current_order.agv_move_action = True

        except Exception as e:
            self._logger.error(
                f"Error occured while shipping and submitting the order. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def publish_order_submission_status(self, status):
        """
        Function to publish order submission status to topic '/submitted_orders'.
        Args:
            status: 'True' if all orders have been processed, False otherwise
        """
        try:
            _msg = Bool()
            _msg.data = status
            self._order_submitted_publisher.publish(_msg)
        except Exception as e:
            self._logger.error(
                f"Error occurred while publishing message to topic '/submitted_orders'. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def update_order_queue(self):
        """
        Function to update the order queue.
        """
        try:
            # If there are no orders in queue, return
            if not self._orders_queue:
                return

            # If there is no order in queue or a high priority order is already present in the queue
            if not self._current_order or self._orders_queue[0].priority > self._current_order.priority:
                if self._current_order:

                    while self._current_order.working:
                        pass

                    # Move the current order to paused queue
                    if self._current_order.grip_tray and not self._current_order.move_tray:
                        self._current_order.grip_tray = False
                        self._current_order._moving_robot_home = False
                        self._current_order._moving_robot_to_table = False
                        self._current_order._entering_tool_changer = False
                        self._current_order._changing_gripper = False
                        self._current_order._exiting_tool_changer = False
                        self._current_order._activating_gripper = False
                        self._current_order._activated_gripper = False
                        self._current_order._deactivating_gripper = False
                        self._current_order._moving_robot_to_tray = False
                        self._current_order._moving_tray_to_agv = False
                        self._current_order._moving_robot_to_part = False
                    self._paused_orders.append(self._current_order)
                    self._logger.info(
                        f"Order #{self._current_order.order_id} is paused.")
                    self._current_order = self._orders_queue.pop(0)
                    self._logger.info(
                        f"Processing Order #{self._current_order.order_id} with priority '{self._current_order.priority}'.")
                else:
                    self._logger.debug("No current order")

                # Set current order as the highest priority order from queue
                    self._current_order = self._orders_queue.pop(0)
                    self._logger.info(
                        f"Processing Order #{self._current_order.order_id} with priority '{self._current_order.priority}'.")
        except Exception as e:
            self._logger.error(
                f"Error occurred while updating the order queue. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def remove_part_from_bin(self):
        """
        Function to remove the part from the bin.
        """
        try:
            _order = self._current_order
            self._logger.info(f"Order: {_order.order_id}")
            self._logger.info(" - Kitting Tray:")
            if len(self._trays) > 0:
                _final_trays = self._trays[_order.tray_id]
                for _tray in _final_trays:
                    if _order.tray_id in self._trays.keys():
                        self._logger.info(f"  - ID: {_order.tray_id}")
                        self._logger.info(
                            f"  - Position (xyz): {_tray['position']}")
                        self._logger.info(
                            f"  - Orientation (rpy): {_tray['orientation']}")
                        self._trays[_order.tray_id].remove(_tray)
            else:
                self._logger.warn("Tray not found.")
                return

            _parts_in_order_list = _order.parts

            for _order_part in _parts_in_order_list:
                _part_type = self._part_properties['type'][_order_part.part.type]
                _part_color = self._part_properties['color'][_order_part.part.color]

                self._logger.info(f" - {_part_color} {_part_type}")
                _final_part = None

                if len(self._bins) > 0:
                    _final_part = self._bins[(
                        _order_part.part.type, _order_part.part.color)]
                    for _part in _final_part:
                        if (_order_part.part.type, _order_part.part.color) in self._bins.keys():
                            self._logger.info(
                                f"   - Position (xyz): {_part['position']}")
                            self._logger.info(
                                f"   - Orientation (rpy): {_part['orientation']}")

                            # Pop the part from the bin
                            self._bins[(_order_part.part.type,
                                        _order_part.part.color)].remove(_part)
                else:
                    self._logger.warn("Part not found in bins.")
                    return

        except Exception as e:
            self._logger.error(
                f"Error occurred while removing part from the bin. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_to_table(self, task):
        """
        Function to move the robot to the table.
        """
        try:
            self._current_order.working = True
            if not self._current_order._moving_robot_home:
                self._current_order._moving_robot_home = True
                self._current_order._moved_robot_home = True

            self.current_task = task
            self._to_table()
        except Exception as e:
            self._logger.error(
                f"Error occurred while moving the robot to the table. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_tray(self):
        """
        Function to move the tray to the AGV.
        """
        try:
            _order = self._current_order
            self._current_order.working = True
            
            # move to tray
            if self._current_order._activated_gripper:
                if not self._current_order._moving_robot_to_tray:
                    self._move_robot_to_tray(
                        _order.tray_id, self._current_order.current_tray['pose'])

        except Exception as e:
            self._logger.error(
                f"Error occurred while moving the tray to the AGV. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_part(self):
        """
        Function to move the part to the AGV.
        """
        try:
            self._current_order.working = True
            if self._current_order._activated_gripper:
                for i, _order_part in enumerate(self._current_order.parts):
                    if len(self._bins) > 0:

                        if (_order_part.part.type, _order_part.part.color) in self._bins.keys():
                            _final_part = self._bins[(
                                _order_part.part.type, _order_part.part.color)]
                            if len(_final_part) > 0:
                                for j, p in enumerate(_final_part):
                                    if not p['picked']:
                                        self._part = p
                                        break

                                if self._part == None:
                                    self._current_order.parts_placed[i] = True
                                    break
                                self._current_order.quadrant = _order_part.quadrant
                            
                                self._current_order.current_part = i
                                self.picking_part(
                                    _order_part.part.color, _order_part.part.type, self._part['pose'], self._part['bin'])
                                self._bins[(
                                            _order_part.part.type, _order_part.part.color)][j]['picked'] = True
                                self._part['picked'] = True

                                while not self._current_order.parts_placed[i]:
                                    if self._current_order.discarded_part:
                                        if not self.discard_parts(_order_part,i):
                                            break
                                    elif (self._current_order.solve_faulty_gripper and self._current_order.current_part == self._current_order.missed_part and not self._current_order.parts_placed[self._current_order.missed_part]):
                                        if not self.grip_fail(_order_part,i):
                                            break

                                self.reset_flags()                                  

                            else:
                                self._current_order.parts_placed[i] = True

                        else:
                            self._current_order.parts_placed[i] = True

                    self._part = None
                   
                if all(self._current_order.parts_placed):
                    self._current_order.processed = True
                    self._current_order.working = False
                    self._current_order.move_part = True
                    self.post_discard_flag = False
                

                self._logger.info(f"All orders of {self._current_order.order_id} are placed")

        except Exception as e:
            self._logger.error(
                f"Error occurred while moving the part to the AGV. Error code: {e}")
            self._logger.error(traceback.format_exc())
            
    def grip_fail(self,_order_part,i ):
        """
        create a function to simulate the faulty gripper
        """  
        if (_order_part.part.type, _order_part.part.color) in self._bins.keys():
            _final_part = self._bins[(
                _order_part.part.type, _order_part.part.color)]
            if len(_final_part) > 0:
                for k, p in enumerate(_final_part):
                    if not p['picked']:
                        self._part = p
                        break
                    else:
                        self._part = None                        
                
                if self._part == None:
                    self._current_order.parts_placed[i] = True
                    return False    
                if (_order_part.part.type, _order_part.part.color) in self._bins.keys() and not self._part['picked']:
                    
    
                    self.reset_flags()                                  

                    self.picking_part(
                        _order_part.part.color, _order_part.part.type, self._part['pose'], self._part['bin'])
                    
                    self.reset_flags()                                  

                    while self._current_order.solve_faulty_gripper:
                        pass
                    self._bins[(
                            _order_part.part.type, _order_part.part.color)][k]['picked'] = True
                    
                else:
                    self._current_order.parts_placed[i] = True
                    return False
            else:
                self._current_order.parts_placed[i] = True
                return False
        else:
            self._current_order.parts_placed[i] = True         
    
        
    def discard_parts(self,_order_part,i ):
        """ 
        create a function to discard the faulty parts
        """
        if (_order_part.part.type, _order_part.part.color) in self._bins.keys():
            _final_part = self._bins[(
                _order_part.part.type, _order_part.part.color)]
            if len(_final_part) > 0:
                for k, p in enumerate(_final_part):
                    if not p['picked']:
                        self._part = p
                        break
                    else:
                        self._part = None
                       
                if self._part == None:
                    self._current_order.parts_placed[i] = True
                    return False    
                if (_order_part.part.type, _order_part.part.color) in self._bins.keys() and not self._part['picked']:
                    
                    self.reset_flags()                                  

                    self.picking_part(
                        _order_part.part.color, _order_part.part.type, self._part['pose'], self._part['bin'])
                    self.reset_flags()                                  
                    while self._current_order.discarded_part:
                        pass
                    self._bins[(
                            _order_part.part.type, _order_part.part.color)][k]['picked'] = True
                else:
                    self._current_order.parts_placed[i] = True
                    return False
            else:
                self._current_order.parts_placed[i] = True
                return False
        else:
            self._current_order.parts_placed[i] = True    
                 
    def reset_flags(self):
        """ 
        create a function to reset the flags
        """
        self._current_order._moving_robot_to_part = False
        self._current_order._moved_robot_to_part = False
        self._current_order._moving_part_to_agv = False
        self._current_order._moved_part_to_agv = False
        self._current_order._dropping_part = False
        self._current_order._dropped_part = False
        self._current_order._deactivating_gripper = False
        self._current_order._deactivated_gripper = False    
        
        
    def process_orders(self):
        """
        Function to simulate the orders for the AGV. This function is called through a timer at a given frequency.
        It iterates through the orders and sets the priority for each order.
        """
        try:
            # Processing current order
            if self._current_order and not self._current_order.processed and not self._current_order.working and not self._current_order.grip_tray:
                self.move_to_table("trays")
                
            if self._current_order and self._current_order._moved_robot_to_table and self._current_order.grip_tray and not self._current_order.move_tray and not self._current_order.working:
                self.move_tray()

            if self._current_order and self._current_order.grip_tray and self._current_order.move_tray and not self._current_order.working and not self._current_order.grip_part:
                self._current_order._moving_robot_home = False
                self._current_order._moving_robot_to_table = False
                self._current_order._entering_tool_changer = False
                self._current_order._changing_gripper = False
                self._current_order._exiting_tool_changer = False
                self._current_order._activating_gripper = False
                self._current_order._deactivating_gripper = False
                self.move_to_table("parts")

            if self._current_order and self._current_order.grip_part and self._current_order.move_tray and not self._current_order.working and not self._current_order.move_part:
                self.move_part()

            if self._current_order and self._current_order.processed:
                self.ship_and_submit_order()
                try:
                    while not self._current_order.submitted:
                        pass
                    if self._current_order.submitted:
                        self._current_order = None
                        self._orders_queue = self._paused_orders + self._orders_queue
                        self._logger.info(f"Orders queue {self._orders_queue}")
                        # add the trays of paused orders back to the trays
                        for _order in self._paused_orders:

                            if _order.tray_id in self._trays.keys():
                                self._logger.info(
                                    f"Tray Keys paused: {self._trays.keys()} {_order.current_tray}")
                                self._trays[_order.tray_id].append(
                                    _order.current_tray)

                        self._paused_orders = []
                except Exception as e:
                    pass

            # Publish status based as 'True' if all orders have been processed
            self.publish_order_submission_status(
                not self._orders_queue and not self._current_order)

            # Processing the pending orders
            with self._order_lock:
                self.update_order_queue()

        except Exception as e:
            self._logger.error(
                f"Error occurred while processing orders. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def ariac_order_cb(self, msg):
        """
        Callback function to processing incoming order msgs from ariac/orders
        Args:
            msg (ariac_msgs/msg/Order): Incoming order msg.
        """
        try:
            # Create a new Order object
            _next_order = CustomOrder(msg.id, msg.priority)
            _next_order.type = msg.type
            _next_order.agv_num = msg.kitting_task.agv_number
            _next_order.tray_id = msg.kitting_task.tray_id
            _next_order.destination = msg.kitting_task.destination
            _next_order.parts = msg.kitting_task.parts
            _next_order.parts_placed = [False] * len(_next_order.parts)
            _next_order.faulty_parts = [False] * 4

            # Add the order to order queue and sort based on priority
            with self._order_lock:
                self._orders_queue.append(_next_order)
                self._orders_queue.sort(
                    key=lambda order: order.priority, reverse=True)

        except Exception as e:
            self._logger.error(
                f"Failed to process incoming order msg from ariac/orders. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def lock_tray_agv(self, order):
        """
        Function to lock the tray of an AGV based on the given order.
        Args:
            order (ariac_msgs/msg/Order): Order msg from the queue.
        """
        try:
            # Create a client to call the service for locking the tray of the specified AGV
            self._lock_agv_tray_client = self.create_client(
                Trigger,
                f'/ariac/agv{order.agv_num}_lock_tray',
                callback_group=self._group
            )

            # Check if the service is ready
            if not self._lock_agv_tray_client.service_is_ready():
                self._logger.warn(
                    'Lock_AGV_Tray Service not available, waiting again...')
                return

            # Trigger service to lock the tray
            self._logger.debug(f'Locking tray for AGV #{order.agv_num}')
            request = Trigger.Request()
            future = self._lock_agv_tray_client.call_async(request)

            # Handle the response of locking the tray
            future.add_done_callback(self.lock_agv_tray_response_callback)

        except Exception as e:
            self._logger.error(
                f"Error encountered while locking the AGV tray. Error code: {e}")
            self._logger.error(traceback.format_exc())

    '''------------------------------------ Floor robot functions -----------------------'''

    def _move_robot_home(self, end_demo=False):
        """
        Move the floor robot to its home position
        """

        self.get_logger().info("👉 Moving robot home...")
        if end_demo:
            self._ending_demo = True
        else:
            self._current_order._moving_robot_home = True

        while not self._move_robot_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move robot Service not available, waiting...")

        request = Trigger.Request()
        future = self._move_robot_home_cli.call_async(request)
        future.add_done_callback(self._move_robot_home_done_cb)

    def _move_robot_home_done_cb(self, future):
        """
        Client callback for the service /competitor/floor_robot/go_home

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._current_order._moved_robot_home = True
            if self._current_order.discard_part:
                self._deactivate_gripper()
            else:
                self._to_table()
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _to_table(self):
        if self._current_order._moved_robot_home:
            _order = self._current_order
            if not self._current_order._moving_robot_to_table:
                if self.current_task == "trays":
                    if len(self._trays) > 0:
                        for _tray in self._trays[_order.tray_id]:

                            if _order.tray_id in self._trays.keys():
                                if _tray['camera'] == 1:
                                    self._move_robot_to_table(
                                        MoveRobotToTable.Request.KTS1)
                                    self._current_order.current_tray = _tray
                                    self._trays[self._current_order.tray_id].remove(
                                        self._current_order.current_tray)

                                    break
                                elif _tray['camera'] == 2:
                                    self._move_robot_to_table(
                                        MoveRobotToTable.Request.KTS2)
                                    self._current_order.current_tray = _tray
                                    self._trays[self._current_order.tray_id].remove(
                                        self._current_order.current_tray)

                                    break
                elif self.current_task == "parts":
                    self._move_robot_to_table(MoveRobotToTable.Request.KTS2)

    def _move_robot_to_table(self, table_id):
        '''
        Move the floor robot to a table

        Args:
            table_id (int): 1 for kts1 and 2 for kts2
        '''

        self.get_logger().info('👉 Moving robot to changing station...')
        self._current_order._moving_robot_to_table = True
        while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = MoveRobotToTable.Request()
        request.kts = table_id
        future = self._move_robot_to_table_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_table_done_cb)

    def _move_robot_to_table_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_table

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._moved_robot_to_table = True

            if (self.current_gripper == self.current_task):

                self._current_order.grip_tray = True

                if not self._current_order._activated_gripper:
                    self._current_order._activated_gripper = True

                else:
                    self._activate_gripper()
                self._current_order.working = False
            else:
                self._change_tool()

        else:
            self.get_logger().fatal(f'💀 {message}')
            self._failure = True

    def _change_tool(self):
        """ 
        Create a function to simulate change the tool
        """
        if self._current_order._moved_robot_to_table:
            if not self._current_order._entering_tool_changer:
                if self.current_task == "trays":
                    if self._current_order.current_tray['camera'] == 1:
                        self._enter_tool_changer("kts1", self.current_task)

                    # Changed this line
                    elif self._current_order.current_tray['camera'] == 2:
                        self._enter_tool_changer("kts2", self.current_task)
                elif self.current_task == "parts":
                    self._enter_tool_changer("kts2", self.current_task)

    def _enter_tool_changer(self, station, gripper_type):
        '''
        Move the end effector inside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Entering tool changer...')
        self._current_order._entering_tool_changer = True
        while not self._enter_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = EnterToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._enter_tool_changer_cli.call_async(request)
        future.add_done_callback(self._enter_tool_changer_done_cb)

    def _enter_tool_changer_done_cb(self, future):
        '''
        Client callback for the service /commander/enter_tool_changer

        output: result (boolean) - True for successful part pickup. False for failure
        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._entered_tool_changer = True
            self.grip_change()

        else:
            self.get_logger().fatal(f'💀 {message}')

    def grip_change(self):
        """ 
        Create a function to simulate the gripper change
        """
        if self._current_order._entered_tool_changer:
            if not self._current_order._changing_gripper:
                if self.current_task == "trays":
                    self._change_gripper(
                        ChangeGripper.Request.TRAY_GRIPPER)

                elif self.current_task == "parts":
                    self._change_gripper(
                        ChangeGripper.Request.PART_GRIPPER)

    def _change_gripper(self, gripper_type):
        '''
        Change the gripper
        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Changing gripper...')
        self._current_order._changing_gripper = True
        while not self._change_gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = ChangeGripper.Request()
        request.gripper_type = gripper_type
        future = self._change_gripper_cli.call_async(request)
        future.add_done_callback(self._change_gripper_done_cb)

    def _change_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_change_gripper

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info('✅ Gripper changed')
            self._current_order._changed_gripper = True
            self.exit_tool()

        else:
            self.get_logger().fatal(f'💀 {message}')

    def exit_tool(self):
        if self._current_order._changed_gripper:
            if not self._current_order._exiting_tool_changer:
                if self.current_task == "trays":

                    if self._current_order.current_tray['camera'] == 1:
                        self._exit_tool_changer("kts1", self.current_task)
                        
                    elif self._current_order.current_tray['camera'] == 2:
                        self._exit_tool_changer("kts2", self.current_task)

                elif self.current_task == "parts":
                    self._exit_tool_changer("kts2", self.current_task)

    def picking_part(self, color, part_type, pose, bins_location):
        '''
        Moving the floor robot to bin for picking up the part
        Args:
            agv_number (int): AGV number
            quadrant (int): Quadrant to which the part belongs to
        '''
        self.get_logger().info('👉 Moving robot to part...')
        self._current_order._moving_robot_to_part = True  # change1-1
        while not self._move_robot_to_part_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MoveRobotToPart.Request()
        request.color = color
        request.type = part_type
        request.part_pose_in_world = pose
        request.bin_location = bins_location
        future = self._move_robot_to_part_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_part_done_cb)

    def _move_robot_to_part_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_part

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._moved_robot_to_part = True 
            self.part_agv()
        else:
            self.get_logger().fatal(f'💀 {message}')

    def part_agv(self):
        if self._current_order._moved_robot_to_part and not self._current_order._moving_part_to_agv:
            self._move_part_to_agv(
                self._current_order.agv_num, self._current_order.quadrant)
        else:
            self._logger().info(
                f" Part AGV status -  {self._current_order.moved_robot_to_part} moving part to agv status - { self._current_order._moving_part_to_agv}")

    def _move_part_to_agv(self, agv_number, quadrant):
        '''
        Move floor robot to agv and place the part
        Args:
            agv_number (int): AGV number
            quadrant (int): Quadrant to which the part belongs to
        '''
        self.get_logger().info('👉 Move the robot and place part on AGV...')
        self._current_order._moving_part_to_agv = True
        self._current_order.part_picked = True

        while not self._move_part_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MovePartToAGV.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._move_part_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_part_to_agv_done_cb)

    def _exit_tool_changer(self, station, gripper_type):
        '''
        Move the end effector outside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Exiting tool changer...')
        self._current_order._exiting_tool_changer = True
        while not self._exit_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        '''
        output: result (boolean) - True for successful part pickup. False for failure
        '''
        request = ExitToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._exit_tool_changer_cli.call_async(request)
        future.add_done_callback(self._exit_tool_changer_done_cb)

    def _exit_tool_changer_done_cb(self, future):
        '''
        Client callback for the service /commander/exit_tool_changer
        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._exited_tool_changer = True
            self.grip_activate()
        else:
            self.get_logger().fatal(f'💀 {message}')

    def grip_activate(self):
        if self._current_order._exited_tool_changer:
            if not self._current_order._activating_gripper:
                self._activate_gripper()

    def _activate_gripper(self):
        '''
        Activate the gripper
        '''
        self.get_logger().info('👉 Activating gripper...')
        self._current_order._activating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = True
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._activate_gripper_done_cb)

    def _activate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper activated')
            self._current_order._activated_gripper = True
            self.current_gripper = self.current_task
            if (self.current_task == "trays"):
                if not self._current_order.grip_tray:
                    self._current_order.grip_tray = True
                    self._current_order.working = False

            elif (self.current_task == "parts"):
                if not self._current_order.grip_part:
                    self._current_order.grip_part = True
                    self._current_order.working = False

        else:

            self.get_logger().fatal(
                f'💀 Gripper not activated  {self.current_task}')

    def _deactivate_gripper(self):
        '''
        Deactivate the gripper
        '''
        self.get_logger().info('👉 Deactivating gripper...')
        self._current_order._deactivating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = False
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._deactivate_gripper_done_cb)

    def _deactivate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper deactivated')
            self._current_order.part_picked = False
            self._current_order._deactivated_gripper = True
            if (self.current_task == "trays"):
                self._drop_tray(self._current_order.tray_id,
                                self._current_order.agv_num)

            elif (self.current_task == "parts"):

                if self._current_order.discard_part:
                    self._current_order.discard_part = False
                    self._current_order.discarded_part = True
                    self._activate_gripper()

                elif self._current_order.discarded_part:
                    self.drop_part()
                
                elif  self._current_order.solve_faulty_gripper and self._current_order.faulty_gripper_detected :
                    self._current_order.solve_faulty_gripper = False
                    self._current_order.faulty_gripper_detected = False
                    self.drop_part()

                else:
                    self.drop_part()

        else:
            self.get_logger().fatal('💀 Gripper not deactivated')

    def _move_part_to_agv_done_cb(self, future):
        '''
        Client callback for the service /commander/move_part_to_agv

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._moved_part_to_agv = True
            if not self._current_order.faulty_gripper_detected:
                self.check_faulty_part(self._current_order.order_id)
            else:
                if not self._current_order.solve_faulty_gripper:
                    self._current_order.solve_faulty_gripper = True
                    self._current_order.missed_part =  self._current_order.current_part
                
                else:

                    self.check_faulty_part(self._current_order.order_id)

        else:
            self.get_logger().fatal(f'💀 {message}')

    def drop_part(self):
        if self._current_order._moved_part_to_agv:
            self._drop_part(self._current_order.agv_num,
                            self._current_order.quadrant)

    def _move_robot_to_tray(self, tray_id, tray_pose):
        '''
        Move the floor robot to a tray to pick it up
        '''
        self.get_logger().info('👉 Moving robot to tray...')
        self._current_order._moving_robot_to_tray = True
        while not self._move_robot_to_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MoveRobotToTray.Request()
        request.tray_id = tray_id
        request.tray_pose_in_world = tray_pose
        future = self._move_robot_to_tray_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_tray_done_cb)

    def _move_robot_to_tray_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_tray

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._moved_robot_to_tray = True
            self.tray_to_agv()

        else:
            self.get_logger().fatal(f'💀 {message}')

    def tray_to_agv(self):
        if self._current_order._moved_robot_to_tray:
            if not self._current_order._moving_tray_to_agv:
                self._move_tray_to_agv(self._current_order.agv_num)

    def _move_tray_to_agv(self, agv_number):
        self._current_order._moving_tray_to_agv = True

        while not self._move_tray_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = MoveTrayToAGV.Request()
        request.agv_number = agv_number
        future = self._move_tray_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_tray_to_agv_done_cb)

    def _move_tray_to_agv_done_cb(self, future):
        '''
        Client callback for the service /commander/move_tray_to_agv

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._moved_tray_to_agv = True
            if not self._current_order._deactivating_gripper:
                self._deactivate_gripper()
        else:
            self.get_logger().fatal(f'💀 {message}')
            self._failure = True

    def _drop_tray(self, tray_id, agv_number):
        self._current_order._dropping_tray = True

        while not self._drop_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = DropTray.Request()
        request.tray_id = tray_id
        request.agv_number = agv_number
        future = self._drop_tray_cli.call_async(request)
        future.add_done_callback(self._drop_tray_done)

    def _drop_tray_done(self, future):
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._dropped_tray = True
            self._current_order.working = False
            self._current_order.move_tray = True

            if not self._current_order.tray_locked and not self._current_order.tray_lock_action:
                self.lock_tray_agv(self._current_order)
                self._current_order.tray_lock_action = True

        else:
            self.get_logger().fatal(f'💀 {message}')

    def _drop_part(self, agv_number, quadrant):
        self._current_order._dropping_part = True

        while not self._drop_part_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = DropPart.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._drop_part_cli.call_async(request)
        future.add_done_callback(self._drop_part_done)

    def _drop_part_done(self, future):
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._current_order._dropped_part = True
            self._current_order.part_picked = False

            self._current_order.parts_placed[self._current_order.current_part] = True

            # if all parts are placed
            if all(self._current_order.parts_placed):
                self._current_order.processed = True
                self._current_order.working = False
                self._current_order.move_part = True
                self._logger.info(
                    f" ORDER {self._current_order.order_id} processed")
            else:
                self._activate_gripper()
                if  self._current_order.discarded_part:                  
                    self._current_order.discarded_part = False
                    self.post_discard_flag = True
        else:
            self.get_logger().fatal(f'💀 {message}')
            self._failure = True

    '''---------------------------------------- AGV functions ----------------------------'''

    def lock_agv_tray_response_callback(self, future):
        """
        Callback function to handle the response for locking the tray.
        Args:
            future (Future): Future representing the locking of the tray
        """
        try:
            self._logger.debug('[LOCK_AGV_TRAY] Response received.')
            if future.result().success:
                self._logger.debug('Tray locked!')
                self._current_order.tray_locked = True
            else:
                self._logger.warn('Unable to lock tray!')

        except Exception as e:
            self._logger.error(
                f"Failed to handle LOCK_AGV_TRAY response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_agv(self, order):
        """
        Function to move the AGV to a specified destination based on the given order.
        Args:
            order (ariac_msgs/msg/Order): Order msg from the queue.
        """
        try:
            # Create a client to call the service for moving the AGV
            self._move_agv_client = self.create_client(
                MoveAGV,
                f'/ariac/move_agv{order.agv_num}'
            )

            # Trigger move agv service asynchrononously
            self._logger.info(
                f"Moving  AGV #{order.agv_num} to location '{order.destination}'")  # Enums
            _move_agv_msg = MoveAGV.Request()
            _move_agv_msg.location = order.destination
            future = self._move_agv_client.call_async(_move_agv_msg)

            # Handle the response of moving the AGV
            future.add_done_callback(self.move_agv_response_callback)

        except Exception as e:
            self._logger.error(
                f"Error encountered while moving the AGV. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_agv_response_callback(self, future):
        """
        Callback function to handle the response for moving the AGV.
        Args:
            future (Future): Future representing the moving of the AGV.        
        """
        try:
            self._logger.debug('[MOVE_AGV] Response received.')
            if future.result().success:
                self._current_order.shipped = True
                self._submit_order_status = False

                # Create a subscriber for the given AGV
                self._agv_status = self.create_subscription(
                    AGVStatus,
                    f'/ariac/agv{self._current_order.agv_num}_status',
                    self.agv_status_callback,
                    QoSProfile(depth=10),
                    callback_group=self._group
                )

        except Exception as e:
            self._logger.error(
                f"Failed to handle MOVE_AGV response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def submit_order(self, order):
        """
        Function to submit the order.
        Args:
            order (CustomOrder): Order to be submitted.
        """
        try:
            # If order is ready to be shipped.
            if order.shipped:
                # Create a client to call the service to submit the order
                self._submit_order_client = self.create_client(
                    SubmitOrder,
                    '/ariac/submit_order',
                    callback_group=self._group3
                )

                # Trigger submit_order service asynchrononously
                self._logger.info(f'Submitting Order #{order.order_id}...')
                _request = SubmitOrder.Request()
                _request.order_id = order.order_id
                future = self._submit_order_client.call_async(_request)

                # Handle the response for order submission
                future.add_done_callback(self.submit_order_response_callback)

            else:
                self._logger.warn(
                    f'Order #{order.order_id} is not ready for submission')

        except Exception as e:
            self._logger.error(
                f"Error encountering while submitting the order. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def submit_order_response_callback(self, future):
        """
        Callback function to handle the response for submitting the order.
        Args:
            future (Future): Future representing the submission of order.
        """
        try:
            self._logger.info('[SUBMIT_ORDER] Response received.')
            if future.result().success:
                self._logger.info('Order submitted')
                self._current_order.submitted = True
                self._submit_order_status = True
        except Exception as e:
            self._logger.error(
                f"Failed to handle SUBMIT_ORDER response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def agv_status_callback(self, msg):
        """
        Callback function to handle the incoming AGV status msgs.
        Args:
            msg (ariac_msgs/msg/AGVStatus): Incoming AGV Status msg.
        """
        try:
            if msg.location == AGVStatus.WAREHOUSE and not self._submit_order_status and self._current_order.shipped:
                self.submit_order(self._current_order)
                self._submit_order_status = True

        except Exception as e:
            self._logger.error(
                f"Failed to handle incoming AGVStatus msg. Error code: {e}")
            self._logger.error(traceback.format_exc())


    '''---------------------------------------- challenges ----------------------------'''

    def check_faulty_part(self, order_id):
        """
        Function to check if the part is faulty.
        Args:
            order_id (int): Order ID to be checked for faulty part.
        """
        try:

            # Trigger check_faulty_part service asynchrononously
            self._logger.info(f"Checking for faulty part in Order #{order_id}")
            _check_faulty_part_msg = PerformQualityCheck.Request()
            _check_faulty_part_msg.order_id = order_id
            future = self._check_faulty_part_cli.call_async(
                _check_faulty_part_msg)

            # Handle the response of checking the faulty part
            future.add_done_callback(self.check_faulty_part_response_callback)

        except Exception as e:
            self._logger.error(
                f"Error encountered while checking the faulty part. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def check_faulty_part_response_callback(self, future):
        """
        Callback function to handle the response for checking the faulty part.
        Args:
            future (Future): Future representing the checking of the faulty part.
        """
        try:
            self._logger.info('[CHECK_FAULTY_PART] Response received.')
            quadrant = self._current_order.parts[self._current_order.current_part].quadrant

            if future.result().all_passed:
                self._logger.warn('No Faulty part detected!')
                self._current_order.faulty_parts = [False] * 4
            else:

                if quadrant == 1:
                    self._current_order.faulty_parts[0] = future.result(
                    ).quadrant1.faulty_part
                elif quadrant == 2:
                    self._current_order.faulty_parts[1] = future.result(
                    ).quadrant2.faulty_part
                elif quadrant == 3:
                    self._current_order.faulty_parts[2] = future.result(
                    ).quadrant3.faulty_part
                elif quadrant == 4:
                    self._current_order.faulty_parts[3] = future.result(
                    ).quadrant4.faulty_part

            self._current_order.part_picked = True
            if (self._current_order.faulty_parts[quadrant-1]):
                self._discard_part() 

            else:
                self._deactivate_gripper()

        except Exception as e:
            self._logger.error(
                f"Failed to handle CHECK_FAULTY_PART response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _discard_part(self):
        self._current_order.discard_part = True
        self._move_robot_home()

    def check_faulty_gripper_cb(self, msg):
        """
        Callback function to handle the incoming faulty gripper msgs.
        Args:
            msg (std_msgs/msg/Bool): Incoming faulty gripper msg.
        """
        try:
            if msg.enabled and self._current_order.part_picked:
                if not msg.attached:
                    if not self._current_order.faulty_gripper_detected:
                        self._logger.warn('Faulty gripper detected!')
                        self._current_order.faulty_gripper_detected = True

        except Exception as e:
            self._logger.error(
                f"Failed to handle incoming faulty gripper msg. Error code: {e}")
            self._logger.error(traceback.format_exc())
