#!/usr/bin/python3

"""
This script defines a ROS2 node for Main Node.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
import traceback

class CompetitionInterface(Node):
    """
    Class for reading orders and monitoring competition state.
    Attributes:
        _competition_states (dict): Mapping of competition states to their string representations.
        _subscriber: Subscriber for receiving competition state messages.
        _timer_service: Timer for running a callback function periodically.
        _timer_end: Timer for running a callback function periodically.
        _start_competition_client (bool): Flag indicating whether the start competition client has been created.
        _end_client_created (bool): Flag indicating whether the end competition client has been created.
        _all_orders_submitted (bool): Flag indicating whether the competition has ended.
        _competition_state (CompetitionStateMsg): Current competition state message.
        _request: Request object for triggering events.
        _orders_sub: Subscriber for receiving submitted orders.
    """
    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    def __init__(self):
        super().__init__('CompetitionInterface')
        self._logger = self.get_logger()
        
        try:
            # Setting simulation time parameter
            sim_time = Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )
            self.set_parameters([sim_time])

            # Create callback groups
            _group_1 = None
            _group_2 = ReentrantCallbackGroup()

            # Create a subscriber to competition state topic
            self._subscriber = self.create_subscription(
                CompetitionStateMsg,  
                "/ariac/competition_state",  
                self.competition_state_cb,
                100,
                callback_group=_group_2
            )

            # Create a timer for callback groups for services and end_competition            
            self._timer_service = self.create_timer(1, self.start_competition_service, callback_group=_group_2)
            self._timer_end = self.create_timer(1, self.end_competition_service, callback_group=_group_2)

            # Initialize variables
            self._start_competition_client = False
            self._end_client_created = False
            self._all_orders_submitted = False
            self._competition_state: CompetitionStateMsg = None

            # Trigger service client for first order
            # self._request = Trigger.Request()
            self._logger.info("Client created!")

            # Create a subscriber to submitted orders topic
            self._orders_sub = self.create_subscription(
                Bool,
                '/submitted_orders',
                self.submitted_orders_cb,
                10, 
                callback_group=_group_1
            )

        except Exception as e:
            self._logger.error(f"Failed while initializing Class CompetitionInterface. Error code: {e}")
            self._logger.error(traceback.format_exc())
        
    def start_competition_service(self):
        """
        Function to trigger start competition service.
        """
        try:
            self._start_competition_client = self.create_client(Trigger, "/ariac/start_competition")

            # while not self._start_competition_client.wait_for_service(timeout_sec=1.0):
            #     self.logger.info("Service not available, waiting...")

        except Exception as e:
            self._logger.error(f"Error encountered while triggering start competition service. Error code: {e}")
            self._logger.error(traceback.format_exc())
    
    def end_competition_service(self):
        """
        Function to trigger end competition service.        
        """
        try:
            if self._end_client_created:
                return  # If the client is already created, no need to repeat this process

            self._end_competition_client = self.create_client(Trigger, "/ariac/end_competition")
            self._end_client_created = True

            # while not self._end_competition_client.wait_for_service(timeout_sec=1.0):
            #     self.logger.info("Service not available, waiting...")

            # Stop the timer after the callback is triggered once
            self._timer_end.cancel()

        except Exception as e:
            self._logger.error(f"Error encountered while triggering end competition service. Error code: {e}")
            self._logger.error(traceback.format_exc())
    
    def competition_state_cb(self, msg: CompetitionStateMsg):
        """
        Callback function to handle incoming ariac/competition_state messages.
        Args:
            msg (ariac_msgs/msg/CompetitionState): Incoming message.
        """
        try:
            self._logger.debug(f"Competition State: {msg.competition_state}")
            
            # Only log when the state changes.
            if self._competition_state != msg.competition_state:
                _log_txt = f'Competition state is: {self._competition_states[msg.competition_state]}'
                self._logger.info(_log_txt, throttle_duration_sec=1.0)
                self._logger.debug(_log_txt)

            self._competition_state = msg.competition_state

            # If all orders have been sent, end competition
            if self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE:
                self.end_competition()

        except Exception as e:
            self._logger.error(f"Error encountered while handling ariac/competition_state messages. Error code: {e}")
            self._logger.error(traceback.format_exc())
    
    def start_competition_cb(self):
        """
        Callback function to handle the start of the competition
        """
        try:
            self._logger.info('Waiting for competition to be ready')
            
            # If the competition is already started, ignore
            if self._competition_state == CompetitionStateMsg.STARTED:
                return
            
            # Wait for competition to be ready
            while self._competition_state != CompetitionStateMsg.READY:
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt:
                    return
                
            self._logger.info('Competition is ready. Starting...')

            # Check if service is available
            if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
                self._logger.error('Service \'/ariac/start_competition\' is not available.')
                return

            # Create trigger request and call starter service
            _request = Trigger.Request()
            future = self._start_competition_client.call_async(_request)

            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self._logger.info('Competition started successfully.')
            else:
                self._logger.warn('Unable to start competition.')

        except Exception as e:
            self._logger.error(f"Error encountered while starting the competition. Error code: {e}")
            self._logger.error(traceback.format_exc())
    
    def submitted_orders_cb(self, msg: Bool):
        """
        Callback function to handle the incoming /submitted_orders messages.
        Args:
            msg (std_msgs/msg/Bool): Incoming message - contains info whether the order is complete or not.
        """
        try:
            self._logger.debug(f"/submitted_orders: {msg.data}")
            self._all_orders_submitted = msg.data
        except Exception as e:
            self._logger.error(f"Error encountered while handling incoming /submitted_orders messages. Error code: {e}")
            self._logger.error(traceback.format_exc())
                
    def check_orders_completed(self):
        """
        Function to check if all orders have been processed or not.
        Returns:
            bool: True if all orders are submitted, False otherwise.
        """
        if self._all_orders_submitted == True:
            return True
        else:
            return False
        
    def check_no_more_orders(self):
        """
        Function to check if all orders have been announced or not.
        Returns:
            bool: True if all orders have been announced, False otherwise.
        """
        try:
            if self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE:
                self._logger.info("No more orders to process.")
                return True
            else:
                return False
        except Exception as e:
            self._logger.error(f"Error encountered while checking for order completition. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def end_competition(self):
        """
        Function to handle the service to end the competition.
        """
        try:
            # Check if there are no more orders left to process
            if self.check_orders_completed() and self.check_no_more_orders():
                if self._end_competition_client is None:
                    self._logger.error("End competition client not created.")
                    return
                
                self._logger.info("Ending the competition...")
                _request = Trigger.Request()
                future = self._end_competition_client.call_async(_request)

                # Wait until the service call is completed
                rclpy.spin_until_future_complete(self, future)

                if future.result() is not None:
                    if future.result().success:
                        self._logger.info('Competition ended successfully.')
                    else:
                        self._logger.warn('Unable to end competition.')
                else:
                    self._logger.error('Service call failed.')
                    
        except Exception as e:
            self._logger.error(f"Error encountered while ending the competition. Error code: {e}")
            self._logger.error(traceback.format_exc())
 