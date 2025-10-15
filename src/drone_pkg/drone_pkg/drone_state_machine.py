#!/usr/bin/env python3

import rclpy
import time
import threading
from concurrent.futures import Future as ConcurrentFuture
from swl_shared_interfaces.srv import DroneCommand
from swl_shared_interfaces.msg import DroneState, BaseState
from swl_drone_interfaces.msg import Telemetry
from swl_drone_interfaces.srv import UploadMission, SetYaw, Land, Return
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from statemachine import StateMachine, State
from rclpy.task import Future

class DroneStateMachine(StateMachine):
    # Define States
    Idle = State(initial=True)
    Ready_To_Fly = State()
    Mission_Uploaded = State()
    Mission_In_Progress = State()
    RTL = State()
    Loiter = State()
    Landing = State()
    Landed = State()
    Charging = State()

    # Define transitions
    system_ready = Idle.to(Ready_To_Fly)
    mission_uploaded = Ready_To_Fly.to(Mission_Uploaded)
    mission_started = Mission_Uploaded.to(Mission_In_Progress)
    return_to_launch = Mission_In_Progress.to(RTL)
    reached_home = RTL.to(Loiter)
    start_landing = Loiter.to(Landing)
    drone_landed = Landing.to(Landed)
    start_charging = Landed.to(Charging)
    fully_charged = Charging.to(Ready_To_Fly)

    # Methods for 'on entering' new states
    def on_enter_Idle(self):
        """Called when entering Idle state"""
        self.model.get_logger().info("Drone system initializing...")
        
    def on_enter_Ready_To_Fly(self):
        """Called when entering Ready_To_Fly state"""
        self.model.get_logger().info("State change: IDLE -> READY TO FLY")

    def on_enter_Mission_Uploaded(self):
        """Called when mission upload completes"""
        self.model.get_logger().info("State change: READY_TO_FLY -> MISSION_UPLOADED")

    def on_enter_Mission_In_Progress(self):
        """Called when flying mission"""
        self.model.get_logger().info("State change: MISSION_UPLOADED -> MISSION_IN_PROGRESS")

    def on_enter_RTL(self):
        """Called when returning to launch"""
        self.model.get_logger().info("State change: MISSION_PROGRESS -> RTL")
        self.model.rtl_start_time = time.time()

    def on_enter_Loiter(self):
        """Called when drone reaches home and enters loiter"""
        self.model.get_logger().info("State change: RTL -> LOITER")

    def on_enter_Landing(self):
        """Called when landing sequence started"""
        self.model.get_logger().info("State change: LOITER -> LANDING")

    def on_enter_Landed(self):
        """Called when drone has landed"""
        self.model.get_logger().info("State change: LANDING -> LANDED")

    def on_enter_Charging(self):
        """Called when charging started"""
        self.model.get_logger().info("State change: LANDED -> CHARGING")

class DroneStateMachineNode(Node):
    def __init__(self):
        super().__init__('drone_state_machine')
        self.state_machine = DroneStateMachine(model=self)

        # Create callback groups for different types of operations
        self.service_callback_group = ReentrantCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Service server for DroneCommand.srv sent from base state machine
        self.drone_command_service = self.create_service(
            DroneCommand,
            'drone/command',
            self.handle_drone_command,
            callback_group=self.service_callback_group
        )

        # Service client for mavsdk_node UploadMission.srv
        self.mavsdk_upload_mission_client = self.create_client(
            UploadMission,
            'drone/upload_mission',
            callback_group=self.client_callback_group
        )

        # Service client for mavsdk_node SetYaw.srv
        self.mavsdk_yaw_client = self.create_client(
            SetYaw,
            'drone/set_yaw',
            callback_group=self.client_callback_group
        )

        # Service client for mavsdk_node Return.srv (but this is the client for reroute_mission command type)
        self.mavsdk_reroute_mission_client = self.create_client(
            UploadMission,
            'drone/reroute_mission',
            callback_group=self.client_callback_group
        )

        # Service client for mavsdk_node Return.srv
        self.mavsdk_return_to_base_client = self.create_client(
            Return,
            'drone/return',
            callback_group=self.client_callback_group
        )

        # Service client for mavsdk_node Return.srv (but this is the client for abort_mission command type)
        self.mavsdk_abort_mission_client = self.create_client(
            Return,
            'drone/abort_mission',
            callback_group=self.client_callback_group
        )

        # Service client for mavsdk_node Land.srv
        self.mavsdk_land_client = self.create_client(
            Land,
            'drone/land',
            callback_group=self.client_callback_group
        )

        # Subscriber for mavsdk_node Telemetry.msg
        self.telemetry_subscriber = self.create_subscription(
            Telemetry,
            'drone/telemetry',
            self.telemetry_callback,
            10
        )

        # Subscriber for base state machine BaseState.msg 
        self.base_state_subscriber = self.create_subscription(
            BaseState,
            'base/state',
            self.base_state_callback,
            10
        )

        # Publisher for DroneState.msg (to base state machine)
        self.drone_state_publisher = self.create_publisher(
            DroneState,
            'drone/state',
            10
        )

        # Timer for publishing DroneState.msg
        self.state_publish_timer = self.create_timer(
            1.0, 
            self.publish_drone_state,
            callback_group=self.timer_callback_group
        )

        # Globals
        self.current_mission_waypoints = []
        self.current_mission_id = None
        self.raw_telemetry = None # Stores raw telemetry from Telemetry.msg to package into a DroneState.msg

        # Processed telemetry for DroneState.msg publishing
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

        self.fake_battery_percentage = 50 #TODO Remove!
        self.charging_timer = None #TODO Remove!

        self.previous_base_state = None
        self.current_base_state = None

        self.get_logger().info('Drone State Machine node started')
        self.get_logger().info(f'Initial state: {self.state_machine.current_state.name}')

        # Wait for MAVSDK UploadMission service to be available
        self.get_logger().info('Waiting for MAVSDK upload mission service...')
        if not self.mavsdk_upload_mission_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('MAVSDK upload mission service not available!')
        else:
            self.get_logger().info('MAVSDK upload mission service connected')

        # Wait for MAVSDK SetYaw service to be available
        self.get_logger().info('Waiting for MAVSDK yaw service...')
        if not self.mavsdk_yaw_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('MAVSDK yaw service not available!')
        else:
            self.get_logger().info('MAVSDK yaw service connected')

        # Wait for MAVSDK Land service to be available
        self.get_logger().info('Waiting for MAVSDK land service...')
        if not self.mavsdk_land_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('MAVSDK land service not available!')
        else:
            self.get_logger().info('MAVSDK land service connected')

        # Initial DroneState.msg publish
        self.publish_drone_state()

# TELEMETRY.MSG CALLBACK FROM MAVSDK_NODE
    def telemetry_callback(self, msg):
        """Process raw telemetry from MAVSDK node and drive state transitions"""
        # Store the raw telemetry
        self.raw_telemetry = msg
        
        # Update all DroneState.msg fields from Telemetry.msg received from mavsdk_node
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
        
        # Drive state transitions based on Telemetry.msg updates
        self.evaluate_state_transitions()

# METHOD TO DRIVE TRANSITIONS IN DRONE STATE MACHINE BASED OFF OF TELEMETRY RECEIVED
    def evaluate_state_transitions(self):
        """Evaluate if state transitions should occur based on current telemetry"""
        
        current_state = self.state_machine.current_state.name
        
        # Idle -> Ready_To_Fly: Good battery and GPS lock
        if current_state == 'Idle':
            if (#self.battery_percentage > 90.0 and 
                self.num_satellites >= 6 and 
                not self.is_armed and 
                self.landed_state == "ON_GROUND"):
                
                self.state_machine.system_ready()
        
        # Mission_Uploaded -> Mission_In_Progress: Armed and in air
        elif current_state == 'Mission uploaded':
            if (self.is_armed and self.is_in_air and self.current_position['alt'] >= 8):
                self.state_machine.mission_started()
        
        # Mission_In_Progress -> RTL: Mission complete or return command
        #TODO we will need to somehow handle the automatic RTL when the drone is doing a patrol. For now we are doing automatic deployment and manual RTL
        # elif current_state == 'Mission_In_Progress':
        #     if self.mission_complete:
        #         self.get_logger().info('Mission complete - returning to launch')
        #         self.state_machine.return_to_launch()

        # Mission_In_Progress -> RTL: Mission complete (handled by return_to_base command)
        elif current_state == 'Mission_In_Progress':
            if self.mission_complete:
                self.get_logger().info('Mission complete - would transition to RTL (if not manually commanded)')
                # Note: RTL transition now happens via return_to_base command, not automatic
        
        # RTL -> Loiter: Mission complete AND in HOLD mode (reached home)
        elif self.state_machine.current_state.name == 'Rtl':
            # Only check transition if we've been in RTL state for sufficient time
            # and the mission is actually complete (not just from previous mission)
            if hasattr(self, 'rtl_start_time'):
                time_in_rtl = time.time() - self.rtl_start_time
                
                if (time_in_rtl > 5.0 and  # Been in RTL for at least 5 seconds
                    self.mission_complete and 
                    str(self.flight_mode) == "HOLD"):  # RTL missions should disarm when complete
                    
                    self.get_logger().info('RTL mission complete - drone has reached home and is disarmed')
                    self.state_machine.reached_home()
                    delattr(self, 'rtl_start_time')  # Clean up
            else:
                # This shouldn't happen, but just in case
                self.get_logger().warn('In RTL state but no start time recorded')
        
        # Landing -> Landed: On ground and disarmed
        elif current_state == 'Landing':
            if not self.is_armed and self.landed_state == "ON_GROUND":
                self.get_logger().info('Drone has landed and disarmed')
                self.state_machine.drone_landed()
        
        # Landed -> Charging: Landed and base station is charging now (state 011)
        elif current_state == 'Landed':
            if not self.is_armed and self.landed_state == "ON_GROUND" and self.current_base_state == 'Charging':
                self.get_logger().info('Drone has landed and disarmed')
                self.state_machine.start_charging()
                self.fake_charging() # SIMULATION
        
        # Charging -> Ready_To_Fly: Battery fully charged
        elif current_state == 'Charging':
            if self.fake_battery_percentage >= 95.0: #TODO change to real batt percentage
                self.get_logger().info('Battery fully charged - drone ready for next mission')
                self.state_machine.fully_charged()

##############
# SIMULATION
##############
    def fake_charging(self):
        """Start fake charging simulation"""
        if self.charging_timer is not None:
            self.charging_timer.cancel()  # Cancel any existing timer
        
        # Charge 5% every 2 seconds (very fast for testing)
        self.charging_timer = self.create_timer(2.0, self.increment_fake_battery)
        self.get_logger().info(f"Started charging simulation from {self.fake_battery_percentage:.1f}%")

    def increment_fake_battery(self):
        """Increment battery by 5% every call"""
        if self.fake_battery_percentage < 100.0:
            self.fake_battery_percentage += 5.0
            self.get_logger().info(f"Charging: Battery at {self.fake_battery_percentage:.1f}%")
        
        if self.fake_battery_percentage >= 95.0:
            # Stop the charging timer
            if self.charging_timer is not None:
                self.charging_timer.cancel()
                self.charging_timer = None
##############
# SIMULATION
##############

# BASESTATE.MSG CALLBACK FROM BASE STATE MACHINE
    def base_state_callback(self, msg):
        """Process base state from base state machine node and drive state transitions"""
        self.previous_base_state = self.current_base_state
        self.current_base_state = msg.current_state

        # Only process if state actually changed
        if self.previous_base_state != self.current_base_state:
            
            # Trigger landing when base station is prepared for landing and drone is in Loiter
            if (self.current_base_state == 'Prepared for landing' and 
                self.state_machine.current_state.name == 'Loiter'):
                self.get_logger().info('Base station prepared for landing - initiating drone landing sequence')
                self.handle_landing_sequence()

# METHOD TO PUBLISH DRONE STATE TO BASE STATE MACHINE
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
        msg.drone_id = "DRONE_001" # Placeholder
        msg.current_state = self.state_machine.current_state.name
        
        self._last_published_state = msg.current_state

        self.drone_state_publisher.publish(msg)

# HANDLER FOR ALL DRONE COMMANDS THAT COME FROM BASE STATE MACHINE
    def handle_drone_command(self, request, response):
        """Handle commands from base station - SYNCHRONOUS but allows nested async calls"""
        self.get_logger().info(f'Received drone command: {request.command_type}')
        
        if request.command_type == 'upload_mission':
            # This will block until the nested service call completes
            response.success = self.handle_mission_upload_sync(request)
        elif request.command_type == 'pan':
            response.success = self.handle_pan(request)
        elif request.command_type == 'return_to_base':
            response.success = self.handle_return_to_base_sync(request)
        elif request.command_type == 'abort_mission':
            response.success = self.handle_abort_mission_sync(request)
        elif request.command_type == 'reroute_mission':
            response.success = self.handle_reroute_mission_sync(request)
        else:
            response.success = False
            self.get_logger().warn(f'Unknown drone command: {request.command_type}')
        
        return response

####################################################
# START - HANDLERS FOR DISTINCT DRONE COMMANDS (THESE METHODS ALSO ACT AS SERVICE CLIENTS TO MAVSDK NODE)
####################################################

    def handle_mission_upload_sync(self, request):
        """Handle mission upload synchronously - waits for MAVSDK response before returning"""
        
        # Validate state
        if self.state_machine.current_state.name != 'Ready to fly':
            self.get_logger().error(f'Cannot upload mission - drone not in Ready_To_Fly state. Current: {self.state_machine.current_state.name}')
            return False

        # Store mission data
        self.current_mission_waypoints = request.waypoints
        self.current_mission_id = request.drone_id

        # Use threading event to wait for async response
        success_event = threading.Event()
        upload_success = [False]  # Use list to allow modification in nested function
        error_message = ['']

        def mavsdk_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'MAVSDK mission upload successful. Taking off!')
                    self.state_machine.mission_uploaded()
                    upload_success[0] = True
                else:
                    self.get_logger().error(f'MAVSDK mission upload failed: {response.message}')
                    error_message[0] = response.message
                    # Clear the stored mission data on failure
                    self.current_mission_waypoints = []
                    self.current_mission_id = None
                    
            except Exception as e:
                self.get_logger().error(f'MAVSDK upload service call failed: {str(e)}')
                error_message[0] = str(e)
                self.current_mission_waypoints = []
                self.current_mission_id = None
            finally:
                success_event.set()  # Signal completion regardless of success/failure

        try:
            # Create MAVSDK upload request
            mavsdk_request = UploadMission.Request()
            mavsdk_request.waypoints = request.waypoints
            
            # Make the async service call
            future = self.mavsdk_upload_mission_client.call_async(mavsdk_request)
            future.add_done_callback(mavsdk_callback)
            
            self.get_logger().info('Mission forwarded to MAVSDK node - waiting for response...')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):  # 30 second timeout
                if upload_success[0]:
                    return True
                else:
                    self.get_logger().error(f'Mission upload failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('Mission upload timed out!')
                # Cancel the future if possible and clean up
                if not future.done():
                    future.cancel()
                self.current_mission_waypoints = []
                self.current_mission_id = None
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to initiate mission upload to MAVSDK: {str(e)}')
            return False

    def handle_pan(self, request):
        """Flexible pan control with exact degrees"""

        yaw_clockwise = request.yaw_cw > 0
        yaw_degrees = abs(request.yaw_cw)

        self.get_logger().info(f'Sending pan: {yaw_degrees:.1f}° {"clockwise" if yaw_clockwise else "counter-clockwise"}')

        pan_request = SetYaw.Request()
        pan_request.yaw_cw = yaw_clockwise
        pan_request.yaw_degrees = yaw_degrees

        def pan_mavsdk_callback(future):
            try:
                response = future.result()
                if response.success:
                    direction = "right" if yaw_clockwise else "left"
                    self.get_logger().info(f'MAVSDK pan executed: {yaw_degrees:.1f}° {direction}')
                else:
                    self.get_logger().warn(f'MAVSDK pan failed: {response.message}')
            except Exception as e:
                self.get_logger().error(f'MAVSDK pan service failed: {str(e)}')

        future = self.mavsdk_yaw_client.call_async(pan_request)
        future.add_done_callback(pan_mavsdk_callback)

        return True

    def handle_reroute_mission_sync(self, request):
        """Handle reroute mission synchronously - waits for MAVSDK response before returning"""
    
        # Validate state - reroute should only work during mission
        current_state = self.state_machine.current_state.name
        if current_state != 'Mission in progress':
            self.get_logger().error(f'Cannot reroute mission from state: {current_state}')
            return False
        
        # Store new mission data
        self.current_mission_waypoints = request.waypoints
        self.current_mission_id = request.drone_id
        
        # Use threading event to wait for async response
        success_event = threading.Event()
        upload_success = [False]
        error_message = ['']

        def mavsdk_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('MAVSDK reroute mission successful. Following new route!')
                    upload_success[0] = True
                else:
                    self.get_logger().error(f'MAVSDK reroute mission failed: {response.message}')
                    error_message[0] = response.message
                    # Clear the stored mission data on failure
                    self.current_mission_waypoints = []
                    self.current_mission_id = None

            except Exception as e:
                self.get_logger().error(f'MAVSDK reroute mission service call failed: {str(e)}')
                error_message[0] = str(e)
                self.current_mission_waypoints = []
                self.current_mission_id = None
            finally:
                success_event.set()
        
        try:
            mavsdk_request = UploadMission.Request()
            mavsdk_request.waypoints = request.waypoints

            future = self.mavsdk_reroute_mission_client.call_async(mavsdk_request)
            future.add_done_callback(mavsdk_callback)

            self.get_logger().info('Reroute mission forwarded to MAVSDK node - waiting for response...')
        
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):
                if upload_success[0]:
                    return True
                else:
                    self.get_logger().error(f'Reroute mission failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('Reroute mission timed out!')
                if not future.done():
                    future.cancel()
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to initiate reroute mission to MAVSDK: {str(e)}')
            return False

    def handle_return_to_base_sync(self, request):
        """Handle return to base synchronously - waits for MAVSDK response before returning"""
        
        # Validate state (should be Mission_In_Progress or related)
        current_state = self.state_machine.current_state.name
        if current_state != 'Mission in progress':
            self.get_logger().error(f'Cannot return to base from state: {current_state}')
            return False
        
        # Store RTL mission data
        self.current_mission_waypoints = request.waypoints
        self.current_mission_id = request.drone_id
        
        # Use threading event to wait for async response
        success_event = threading.Event()
        upload_success = [False]  # Use list to allow modification in nested function
        error_message = ['']

        def mavsdk_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('MAVSDK RTL mission upload successful. Returning to base!')
                    self.mission_complete = False
                    self.state_machine.return_to_launch()
                    upload_success[0] = True
                else:
                    self.get_logger().error(f'MAVSDK RTL mission upload failed: {response.message}')
                    error_message[0] = response.message
                    # Clear the stored mission data on failure
                    self.current_mission_waypoints = []
                    self.current_mission_id = None
                    
            except Exception as e:
                self.get_logger().error(f'MAVSDK RTL upload service call failed: {str(e)}')
                error_message[0] = str(e)
                self.current_mission_waypoints = []
                self.current_mission_id = None
            finally:
                success_event.set()  # Signal completion regardless of success/failure

        try:
            # Create MAVSDK upload request
            mavsdk_request = Return.Request()
            mavsdk_request.waypoints = request.waypoints
            
            # Make the async service call
            future = self.mavsdk_return_to_base_client.call_async(mavsdk_request)
            future.add_done_callback(mavsdk_callback)
            
            self.get_logger().info('RTL mission forwarded to MAVSDK node - waiting for response...')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):  # 30 second timeout
                if upload_success[0]:
                    return True
                else:
                    self.get_logger().error(f'RTL mission upload failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('RTL mission upload timed out!')
                # Cancel the future if possible and clean up
                if not future.done():
                    future.cancel()
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to initiate RTL mission upload to MAVSDK: {str(e)}')
            return False

    def handle_abort_mission_sync(self, request):
        """Handle abort mission synchronously - waits for MAVSDK response before returning"""
        # Validate state - abort can be called from more states than return_to_base
        valid_abort_states = ['Mission in progress', 'Loiter', 'Pan']
        current_state = self.state_machine.current_state.name
        if current_state not in valid_abort_states:
            self.get_logger().error(f'Cannot abort mission from state: {current_state}')
            return False

        self.current_mission_waypoints = request.waypoints
        self.current_mission_id = request.drone_id

        # Use threading event to wait for async response
        success_event = threading.Event()
        upload_success = [False]  # Use list to allow modification in nested function
        error_message = ['']

        def mavsdk_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('MAVSDK abort mission successful. Returning to base!')
                    self.mission_complete = False
                    self.state_machine.return_to_launch()  # Same state transition as return_to_base
                    upload_success[0] = True
                else:
                    self.get_logger().error(f'MAVSDK abort mission failed: {response.message}')
                    error_message[0] = response.message
                    # Clear the stored mission data on failure
                    self.current_mission_waypoints = []
                    self.current_mission_id = None

            except Exception as e:
                self.get_logger().error(f'MAVSDK abort mission service call failed: {str(e)}')
                error_message[0] = str(e)
                self.current_mission_waypoints = []
                self.current_mission_id = None
            finally:
                success_event.set()  # Signal completion regardless of success/failure

        try:
            mavsdk_request = Return.Request()
            mavsdk_request.waypoints = request.waypoints

            future = self.mavsdk_abort_mission_client.call_async(mavsdk_request)
            future.add_done_callback(mavsdk_callback)

            self.get_logger().info('Abort mission forwarded to MAVSDK node - waiting for response...')
        
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):  # 30 second timeout
                if upload_success[0]:
                    return True
                else:
                    self.get_logger().error(f'Abort mission failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('Abort mission timed out!')
                # Cancel the future if possible and clean up
                if not future.done():
                    future.cancel()
                return False
            
        except Exception as e:
            self.get_logger().error(f'Failed to initiate abort mission to MAVSDK: {str(e)}')
            return False

    def handle_landing_sequence(self):
        """Initiate landing when base station is ready"""
        
        # Use threading event for synchronous landing
        success_event = threading.Event()
        landing_success = [False]
        error_message = ['']

        def landing_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('MAVSDK landing command successful - drone landing!')
                    self.state_machine.start_landing()  # Transition to Landing state
                    landing_success[0] = True
                else:
                    self.get_logger().error(f'MAVSDK landing command failed: {response.message}')
                    error_message[0] = response.message
            except Exception as e:
                self.get_logger().error(f'Landing service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            # Create landing request (you'll need to create this service)
            landing_request = Land.Request()  # You'll need to create Land.srv
            landing_request.land_trigger = True
            
            future = self.mavsdk_land_client.call_async(landing_request)
            future.add_done_callback(landing_callback)
            
            self.get_logger().info('Landing command sent to MAVSDK - waiting for response...')
            
            if success_event.wait(timeout=30.0):
                if landing_success[0]:
                    self.get_logger().info('Landing sequence initiated successfully!')
                else:
                    self.get_logger().error(f'Landing initiation failed: {error_message[0]}')
            else:
                self.get_logger().error('Landing command timed out!')
                if not future.done():
                    future.cancel()
                    
        except Exception as e:
            self.get_logger().error(f'Failed to initiate landing sequence: {str(e)}')

####################################################
# END - HANDLERS FOR DISTINCT DRONE COMMANDS (THESE METHODS ALSO ACT AS SERVER CLIENTS TO MAVSDK NODE)
####################################################

def main():
    rclpy.init()
    
    # Create the node
    drone_state_machine = DroneStateMachineNode()
    
    # Create MultiThreadedExecutor with multiple threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(drone_state_machine)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        drone_state_machine.get_logger().info('Drone State Machine shutting down...')
    finally:
        executor.shutdown()
        drone_state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()