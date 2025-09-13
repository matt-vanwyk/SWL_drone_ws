#!/usr/bin/env python3

import rclpy
import time
import threading
from swl_shared_interfaces.srv import DroneCommand
from swl_shared_interfaces.msg import DroneState
from swl_drone_interfaces.msg import Telemetry
from swl_drone_interfaces.srv import UploadMission
from std_msgs.msg import Header
from rclpy.node import Node
from statemachine import StateMachine, State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class DroneStateMachine(StateMachine):

    # Define States
    Idle = State(initial=True)
    Ready_To_Fly = State()
    Mission_Uploaded = State()
    Mission_In_Progress = State()
    Loiter = State()
    Pan = State()
    RTL = State()
    Landing = State()
    Landed = State()
    Charging = State()

    system_ready = Idle.to(Ready_To_Fly)
    mission_uploaded = Ready_To_Fly.to(Mission_Uploaded)
    mission_started = Mission_Uploaded.to(Mission_In_Progress)
    waypoint_reached = Mission_In_Progress.to(Loiter)
    start_yawing = Loiter.to(Pan)
    finish_yawing = Pan.to(Loiter)
    return_to_launch = Mission_In_Progress.to(RTL) | Loiter.to(RTL) | Pan.to(RTL)
    start_landing = RTL.to(Landing)
    drone_landed = Landing.to(Landed)
    start_charging = Landed.to(Charging)
    fully_charged = Charging.to(Ready_To_Fly)

    # Methods for 'on entering' new states
    def on_enter_Idle(self):
        """Called when entering Idle state"""
        self.model.get_logger().info("Drone system initializing...")
        
    def on_enter_Ready_To_Fly(self):
        """Called when entering Ready_To_Fly state"""
        self.model.get_logger().info("Drone ready for missions - awaiting mission upload")

    def on_enter_Mission_Uploaded(self):
        """Called when mission upload completes"""
        self.model.get_logger().info("Mission uploaded successfully - ready for arming")

    def on_enter_Mission_In_Progress(self):
        """Called when flying mission"""
        self.model.get_logger().info("Mission in progress - flying to waypoints")

    def on_enter_Loiter(self):
        """Called when loitering at waypoint"""
        self.model.get_logger().info("Loitering at waypoint")

    def on_enter_Pan(self):
        """Called when yawing/panning at waypoint"""
        self.model.get_logger().info("Panning/yawing at waypoint")

    def on_enter_RTL(self):
        """Called when returning to launch"""
        self.model.get_logger().info("Returning to launch point")

    def on_enter_Landing(self):
        """Called when landing sequence started"""
        self.model.get_logger().info("Landing sequence initiated")

    def on_enter_Landed(self):
        """Called when drone has landed"""
        self.model.get_logger().info("Drone landed safely")

    def on_enter_Charging(self):
        """Called when charging started"""
        self.model.get_logger().info("Drone charging...")

class DroneStateMachineNode(Node):
    
    def __init__(self):
        super().__init__('drone_state_machine')
        self.state_machine = DroneStateMachine(model=self)

        self.callback_group = ReentrantCallbackGroup()

        # Service server for base station commands
        self.drone_command_service = self.create_service(
            DroneCommand,
            'drone/command',
            self.handle_drone_command
        )

        # Publisher for drone state (to base station)
        self.drone_state_publisher = self.create_publisher(
            DroneState,
            'drone/state',
            10
        )

        # Service client for MAVSDK node
        self.mavsdk_upload_mission_client = self.create_client(
            UploadMission,
            'drone/upload_mission',
            callback_group=self.callback_group
        )

        self.mavsdk_upload_mission_client.wait_for_service(timeout_sec=10.0)

        # Subscriber for MAVSDK telemetry (from MAVSDK node)
        self.telemetry_subscriber = self.create_subscription(
            Telemetry,
            'drone/telemetry',
            self.telemetry_callback,
            10
        )

        # Globals
        self.current_mission_waypoints = []
        self.current_mission_id = None
        self.raw_telemetry = None

        # Processed drone data for state publishing
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.current_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.battery_percentage = 100.0
        self.is_armed = False
        self.is_in_air = False
        self.flight_mode = "UNKNOWN"
        self.num_satellites = 0
        self.landed_state = "ON_GROUND"
        self.current_yaw = 0.0
        self.mission_complete = False

        self.get_logger().info('Drone State Machine node started')
        self.get_logger().info(f'Initial state: {self.state_machine.current_state.name}')

        self.publish_drone_state()

        self.state_publish_timer = self.create_timer(1.0, self.publish_drone_state)

    def telemetry_callback(self, msg):
        """Process raw telemetry from MAVSDK node and drive state transitions"""
        # Store the raw telemetry
        self.raw_telemetry = msg
        
        # Update all telemetry fields from MAVSDK
        self.current_position = {'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude}
        self.current_velocity = {'x': msg.velocity_x, 'y': msg.velocity_y, 'z': msg.velocity_z}
        self.battery_percentage = msg.battery_percentage
        self.is_armed = msg.armed
        self.is_in_air = msg.is_in_air
        self.flight_mode = msg.flight_mode
        self.num_satellites = msg.num_satellites
        self.landed_state = msg.landed_state
        self.current_yaw = msg.current_yaw
        self.mission_complete = msg.mission_complete
        
        # Drive state transitions based on telemetry
        self.evaluate_state_transitions()

    def evaluate_state_transitions(self):
        """Evaluate if state transitions should occur based on current telemetry"""
        
        current_state = self.state_machine.current_state.name
        
        # Idle -> Ready_To_Fly: Good battery and GPS lock
        if current_state == 'Idle':
            if (self.battery_percentage > 20.0 and 
                self.num_satellites >= 6 and 
                not self.is_armed and 
                self.landed_state == "ON_GROUND"):
                
                self.get_logger().info(f'Telemetry conditions met for Ready_To_Fly: battery={self.battery_percentage:.1f}%, sats={self.num_satellites}')
                self.state_machine.system_ready()
        
        # Mission_Uploaded -> Mission_In_Progress: Armed and in air
        elif current_state == 'Mission_Uploaded':
            if self.is_armed and self.is_in_air:
                self.get_logger().info('Drone armed and airborne - mission in progress')
                self.state_machine.mission_started()
        
        # Mission_In_Progress -> RTL: Mission complete or return command
        elif current_state == 'Mission_In_Progress':
            if self.mission_complete:
                self.get_logger().info('Mission complete - returning to launch')
                self.state_machine.return_to_launch()
        
        # RTL -> Landing: Altitude getting low and near home
        elif current_state == 'RTL':
            if self.current_position['alt'] < 5.0 and not self.is_in_air:
                self.get_logger().info('Landing sequence detected')
                self.state_machine.start_landing()
        
        # Landing -> Landed: On ground and disarmed
        elif current_state == 'Landing':
            if not self.is_armed and self.landed_state == "ON_GROUND":
                self.get_logger().info('Drone has landed and disarmed')
                self.state_machine.drone_landed()

    def publish_drone_state(self):
        """Publish current drone state for base station"""
        msg = DroneState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "drone"

        # Position and motion
        msg.latitude = self.current_position['lat']
        msg.longitude = self.current_position['lon']
        msg.altitude = self.current_position['alt']
        msg.velocity_x = self.current_velocity['x']
        msg.velocity_y = self.current_velocity['y']
        msg.velocity_z = self.current_velocity['z']
        msg.current_yaw = self.current_yaw

        # Status
        msg.armed = self.is_armed
        msg.flight_mode = self.flight_mode
        msg.is_in_air = self.is_in_air
        msg.battery_percentage = self.battery_percentage
        msg.num_satellites = self.num_satellites
        msg.landed_state = self.landed_state
        msg.mission_complete = self.mission_complete

        # High-level state and ID
        msg.drone_id = "DRONE_001"
        msg.current_state = self.state_machine.current_state.name
        
        # Debug logging (can remove later)
        if hasattr(self, '_last_published_state') and self._last_published_state != msg.current_state:
            self.get_logger().info(f'State changed: {self._last_published_state} -> {msg.current_state}')
        self._last_published_state = msg.current_state

        self.drone_state_publisher.publish(msg)

    def handle_drone_command(self, request, response):
        """Handle commands from base station"""
        self.get_logger().info(f'Received drone command: {request.command_type}')
        
        if request.command_type == 'upload_mission':
            response.success = self.handle_mission_upload(request)
        else:
            response.success = False
            self.get_logger().warn(f'Unknown drone command: {request.command_type}')
        
        return response

    def handle_mission_upload(self, request):
        """Handle mission upload from base station"""
        
        # Validate state
        if self.state_machine.current_state.name != 'Ready to fly':
            self.get_logger().error(f'Cannot upload mission - drone not in Ready to fly state. Current: {self.state_machine.current_state.name}')
            return False

        # Store mission data (but don't transition state yet)
        self.current_mission_waypoints = request.waypoints
        self.current_mission_id = request.drone_id

        self.get_logger().info(f'Mission received from base station - {len(request.waypoints)} waypoints')

        # Forward mission to MAVSDK node - state transition happens in callback
        return self.forward_mission_to_mavsdk(request)

    def forward_mission_to_mavsdk(self, request):
        """Forward mission to MAVSDK node for actual drone upload"""
    
        def mavsdk_upload_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'MAVSDK mission upload successful: {response.message}')
                    self.state_machine.mission_uploaded()
                else:
                    self.get_logger().error(f'MAVSDK mission upload failed: {response.message}')
                    # Stay in Ready_To_Fly state since upload failed
                    # Clear the stored mission data
                    self.current_mission_waypoints = []
                    self.current_mission_id = None
                    
            except Exception as e:
                self.get_logger().error(f'MAVSDK upload service call failed: {str(e)}')
                self.current_mission_waypoints = []
                self.current_mission_id = None
        
        try:
            # Create MAVSDK upload request
            mavsdk_request = UploadMission.Request()
            mavsdk_request.waypoints = request.waypoints
            
            # Make the async service call
            future = self.mavsdk_upload_mission_client.call_async(mavsdk_request)
            future.add_done_callback(mavsdk_upload_callback)
            
            self.get_logger().info('Mission forwarded to MAVSDK node - awaiting confirmation...')
            return True  # Return immediately since this is async
            
        except Exception as e:
            self.get_logger().error(f'Failed to forward mission to MAVSDK: {str(e)}')
            return False

def main():
    rclpy.init()
    
    drone_state_machine = DroneStateMachineNode()
    
    try:
        rclpy.spin(drone_state_machine)
    except KeyboardInterrupt:
        drone_state_machine.get_logger().info('Drone State Machine shutting down...')
    finally:
        drone_state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
