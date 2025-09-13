#!/usr/bin/env python3

import rclpy
import time
import threading
from swl_shared_interfaces.srv import DroneCommand, DroneState
from rclpy.node import Node
from statemachine import StateMachine, State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class DroneStateMachine(StateMachine):

    # Define States
    Idle = State(initial=True)
    Ready_To_Fly = State()

    # Define Transitions
    initialise_drone = Idle.to(Ready_To_Fly)

    def on_enter_Idle(self):
        """Called when entering Idle state"""
        self.model.get_logger().info("System boot... collecting telem")
        
    def on_enter_Ready_To_Fly(self):
        """Called when entering Home state"""
        self.model.get_logger().info("Entered READY_TO_FLY state - drone ready to receive missions")

class DroneStateMachineNode(Node):
    def __init__(self):
        super().__init__('drone_state_machine')
        self.state_machine = DroneStateMachine(model=self)

        self.callback_group = ReentrantCallbackGroup()

