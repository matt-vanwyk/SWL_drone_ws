#!/usr/bin/env python3

import time
import asyncio
import threading
import rclpy
import base64
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FlightMode
from mavsdk.rtk import RtcmData
from swl_drone_interfaces.srv import UploadMission, SetYaw, Land, Return # Reroute
from swl_drone_interfaces.msg import Telemetry
from std_msgs.msg import UInt8MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

async def spin(node: Node):
    def _spin_func():
        while rclpy.ok():
            rclpy.spin_once(node)
        node.get_logger().info("ROS 2 spin thread finished")

    spin_thread = threading.Thread(target=_spin_func, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            await asyncio.sleep(1.0)
    finally:
        node.get_logger().info("Waiting for ROS 2 spin thread to finish")
        spin_thread.join()

class MAVSDKNode(Node):
    def __init__(self):
        super().__init__('mavsdk_node')
        self.drone = System()

        # Services for drone state machine clients
        self.upload_mission_server = self.create_service(
            UploadMission,
            'drone/upload_mission',
            self.handle_upload_mission
        )

        self.set_yaw_server = self.create_service(
            SetYaw,
            'drone/set_yaw',
            self.handle_set_yaw
        )

        self.reroute_mission_server = self.create_service(
            UploadMission,
            'drone/reroute_mission',
            self.handle_reroute_mission
        )

        self.return_server = self.create_service(
            Return,
            'drone/return',
            self.handle_return
        )

        self.abort_mission_server = self.create_service(
            Return,
            'drone/abort_mission',
            self.handle_abort_mission
        )

        self.land_server = self.create_service(
            Land,
            'drone/land',
            self.handle_land
        )

        # Drone telemetry publisher
        self.drone_telemetry_publisher = self.create_publisher(
            Telemetry,
            'drone/telemetry',
            10
        )

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rtcm_subscriber = self.create_subscription(
            UInt8MultiArray,
            'rtcm/frames',
            self.rtcm_corrections_callback,
            qos
        )

        # Create variables to store telemetry data
        self.position = None
        self.armed = False
        self.flight_mode = None
        self.battery = None
        self.in_air = False
        self.num_satellites = 0
        self.landed = None
        self.velocity = None
        self.mission_complete = False
        self.heading = None
        self.gps_fix_type = None 
        
        self.loop = asyncio.get_event_loop()

################################################
# START UP AND CONNECT FUNCTIONS
################################################
    async def start(self):
        await self.connect_to_drone()
        
        # Start telemetry updates in separate tasks
        self.loop.create_task(self.update_position())
        self.loop.create_task(self.update_armed_state())
        self.loop.create_task(self.update_flight_mode())
        self.loop.create_task(self.update_battery_state())
        self.loop.create_task(self.update_in_air())
        self.loop.create_task(self.update_num_satellites())
        self.loop.create_task(self.update_landed_state())
        self.loop.create_task(self.update_velocity())
        self.loop.create_task(self.update_mission_progress())
        self.loop.create_task(self.update_drone_heading())
        self.loop.create_task(self.update_gps_info())
        
        await asyncio.sleep(1)
        # Create timers for publishers
        self.create_timer(0.5, self.publish_telemetry)

    async def connect_to_drone(self):
        try:
            self.get_logger().info("Connecting to drone...")
            await self.drone.connect(system_address="udp://:14550")  # Connect to PX4 SITL
            self.get_logger().info("Drone connected.")
        except Exception as e:
            self.get_logger().error(f"Error during connection and initialization: {str(e)}")

################################################
#START - ASYNC FUNCTIONS TO UPDATE TELEMETRY DATA
################################################

    async def update_position(self):
        try:
            async for position in self.drone.telemetry.position():
                self.position = position
                # self.get_logger().info(f"Raw position data: {position}")
                # self.get_logger().info(f"Latitude: {position.latitude_deg}, Longitude: {position.longitude_deg}, Relative Altitude: {position.relative_altitude_m}")
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in position update: {str(e)}")
            
    async def update_armed_state(self):
        try:
            async for armed in self.drone.telemetry.armed():
                self.armed = armed
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in arming update: {str(e)}")
    
    async def update_flight_mode(self):
        try:
            async for fmode in self.drone.telemetry.flight_mode():
                self.flight_mode = fmode
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in flight mode update: {str(e)}")

    async def update_battery_state(self):
        try:
            async for battery in self.drone.telemetry.battery():
                self.battery = battery
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in battery status update: {str(e)}")
            
    async def update_in_air(self):
        try:
            async for in_air in self.drone.telemetry.in_air():
                self.in_air = in_air
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in in_air status update: {str(e)}")
     
    async def update_num_satellites(self):
        try:
            async for num_satellites in self.drone.telemetry.gps_info():
                self.num_satellites = num_satellites
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in num_satellites status update: {str(e)}")   

    async def update_gps_info(self):
        """Track GPS fix type and satellite count; log on fix transitions."""
        last_fix = None
        try:
            async for info in self.drone.telemetry.gps_info():
                # info has: fix_type, num_satellites
                self.gps_fix_type = info.fix_type
                self.num_satellites = info  # keep your existing usage in publish_telemetry

                if info.fix_type != last_fix:
                    self.get_logger().info(
                        f"GPS fix: {info.fix_type}, sats={info.num_satellites}"
                    )
                    last_fix = info.fix_type
                # no sleep needed; this is an async generator
        except Exception as e:
            self.get_logger().error(f"Error in GPS info update: {str(e)}")
 
    async def update_landed_state(self):
        try:
            async for landed in self.drone.telemetry.landed_state():
                self.landed = landed
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in landed status update: {str(e)}") 

    async def update_velocity(self):
        try:
            async for velocity in self.drone.telemetry.velocity_ned():
                # self.get_logger().info(f"Time: {time.time()}, Velocity: {velocity}")
                self.velocity = velocity
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in velocity update: {str(e)}")
    
    async def update_mission_progress(self):
        try:
            async for mission_progress in self.drone.mission.mission_progress():
                self.mission_complete = mission_progress.current == mission_progress.total
                self.get_logger().info(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
                # await asyncio.sleep(0.1)  # Small delay to prevent tight loop
        except Exception as e:
            self.get_logger().error(f"Error in mission progress update: {str(e)}")
    
    async def update_drone_heading(self):
        try:
            async for heading in self.drone.telemetry.heading():
                self.heading = heading
                # self.get_logger().info(f"Current heading: {self.heading}")
                # await asyncio.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Error in heading update: {str(e)}")

################################################
#START - ASYNC FUNCTIONS TO UPDATE TELEMETRY DATA
################################################
            
    # Telemetry publish method
    def publish_telemetry(self):
        if self.position:
            state_msg = Telemetry()
            state_msg.latitude = self.position.latitude_deg
            state_msg.longitude = self.position.longitude_deg
            state_msg.altitude = self.position.relative_altitude_m
            state_msg.armed = self.armed
            state_msg.flight_mode = str(self.flight_mode)
            state_msg.is_in_air = self.in_air
            state_msg.num_satellites = self.num_satellites.num_satellites
            state_msg.landed_state = str(self.landed)
            state_msg.velocity_x = self.velocity.north_m_s
            state_msg.velocity_y = self.velocity.east_m_s
            state_msg.velocity_z = self.velocity.down_m_s
            state_msg.battery_percentage = self.battery.remaining_percent
            state_msg.mission_complete = self.mission_complete
            state_msg.current_yaw = self.heading.heading_deg
            # state_msg.current_yaw = self.attitude
            
            state_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.drone_telemetry_publisher.publish(state_msg)

################################################
#START - HANDLER FUNCTIONS FOR SERVICE REQUESTS FROM DRONE STATE MACHINE AND ASSOCIATED ASYNC MAVSDK FUNCTIONS 
################################################   

    def handle_upload_mission(self, request, response):
        future = asyncio.run_coroutine_threadsafe(self.upload_mission(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def upload_mission(self, request):
        await self.drone.mission.clear_mission()
        try:
            mission_items = []
            for wp in request.waypoints:
                mission_items.append(MissionItem(
                    wp.latitude,
                    wp.longitude,
                    wp.altitude,
                    wp.speed,
                    True,  # is_fly_through
                    float('nan'),  # gimbal_pitch_deg
                    float('nan'),  # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,
                    float('nan'),  # loiter_time_s
                    float('nan'),  # camera_photo_interval_s
                    float('nan'),  # acceptance_radius_m
                    float('nan'),  # yaw_deg
                    float('nan'),   # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE
                ))

            mission_plan = MissionPlan(mission_items)
            self.mission_length = len(mission_items)
            self.mission_progress = 0
            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info("Mission uploaded successfully")

            await self.drone.mission.set_return_to_launch_after_mission(False)

            await self.drone.action.arm()
            self.get_logger().info("Drone armed")
            
            await self.drone.mission.start_mission()
            self.get_logger().info("Mission Started")

            return {"success": True, "message": "Mission uploaded, flight mode set to MISSION, and drone armed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}

    def handle_set_yaw(self, request, response):
        """Handle yaw/pan command"""
        future = asyncio.run_coroutine_threadsafe(self.set_yaw(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def set_yaw(self, request):
        """Execute yaw/pan with simple debugging and strategic waits"""
        try:
            if str(self.flight_mode) != "HOLD":
                return {"success": False, "message": f"Unsafe mode: {self.flight_mode}"}
            
            # Debug 2: Get current position and heading
            current_position = await self.drone.telemetry.position().__anext__()
            current_heading = await self.drone.telemetry.heading().__anext__()
            current_yaw = current_heading.heading_deg
            
            # Calculate target yaw
            if request.yaw_cw:
                target_yaw = (current_yaw + request.yaw_degrees) % 360
            else:
                target_yaw = (current_yaw - request.yaw_degrees) % 360
            
            # Create mission item
            mission_items = [MissionItem(
                current_position.latitude_deg,
                current_position.longitude_deg,
                current_position.relative_altitude_m,
                0,
                True,
                float('nan'), 
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'), 
                float('nan'),
                target_yaw,
                float('nan'),
                MissionItem.VehicleAction.NONE
            )]
            
            mission_plan = MissionPlan(mission_items)
            
            await self.drone.mission.clear_mission()
            await asyncio.sleep(1.0)
            await self.drone.mission.upload_mission(mission_plan)
            await self.drone.mission.set_return_to_launch_after_mission(False)
            self.get_logger().info("Yaw mission uploaded successfully")
            await asyncio.sleep(0.5)
            await self.drone.mission.start_mission()
            
            return {"success": True, "message": f"Yaw changed to {target_yaw:.1f}°"}
            
        except Exception as e:
            self.get_logger().error(f"YAW FAILED: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            
            # Debug the current state when we fail
            try:
                error_mode = await self.drone.telemetry.flight_mode().__anext__()
                self.get_logger().error(f"Flight mode during error: {error_mode}")
            except:
                self.get_logger().error("Could not get flight mode during error")
                
            return {"success": False, "message": f"Error: {str(e)}"}

    def handle_reroute_mission(self, request, response):
        """Handle reroute mission command"""
        future = asyncio.run_coroutine_threadsafe(self.execute_reroute_mission(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def execute_reroute_mission(self, request):
        """Execute reroute mission sequence using MAVSDK"""
        try:
            self.get_logger().info("Starting reroute mission sequence...")
            
            # Step 1: Pause current mission
            self.get_logger().info("Pausing current mission...")
            await self.drone.mission.pause_mission()
            self.get_logger().info("Mission paused")

            # Step 2: Create mission items for base coordinates
            self.get_logger().info("Creating new mission with reroute waypoints..")
            mission_items = []
            for wp in request.waypoints:
                mission_items.append(MissionItem(
                    wp.latitude,
                    wp.longitude,
                    wp.altitude,
                    wp.speed,
                    True,  # is_fly_through
                    float('nan'),  # gimbal_pitch_deg
                    float('nan'),  # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,
                    float('nan'),  # loiter_time_s
                    float('nan'),  # camera_photo_interval_s
                    float('nan'),  # acceptance_radius_m
                    float('nan'),  # yaw_deg
                    float('nan'),   # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE
                ))
            
            # Step 2: Put drone in hold mode
            self.get_logger().info("Activating hold mode...")
            await self.drone.action.hold()
            self.get_logger().info("Hold mode activated")

            # Step 3: Upload new mission (base coordinates)
            mission_plan = MissionPlan(mission_items)
            
            # Step 4: Clear current mission
            self.get_logger().info("Clearing current mission...")
            await self.drone.mission.clear_mission()
            self.get_logger().info("Mission cleared")

            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info("Reroute mission uploaded successfully")
            
            # Step 6: Start the abort mission
            await self.drone.mission.start_mission()
            self.get_logger().info("Reroute mission started - following new route")

            return {"success": True, "message": "Reroute mission completed - drone following new waypoints"}
            
        except Exception as e:
            self.get_logger().error(f"Reroute mission failed: {str(e)}")
            return {"success": False, "message": f"Error: {str(e)}"}

    def handle_return(self, request, response):
        future = asyncio.run_coroutine_threadsafe(self.do_return(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def do_return(self, request):
        try:
            self.get_logger().info("Creating Mission")
            mission_items = []
            for wp in request.waypoints:
                mission_items.append(MissionItem(
                    wp.latitude,
                    wp.longitude,
                    wp.altitude,
                    wp.speed,
                    True,  # is_fly_through
                    float('nan'),  # gimbal_pitch_deg
                    float('nan'),  # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,
                    float('nan'),  # loiter_time_s
                    float('nan'),  # camera_photo_interval_s
                    float('nan'),  # acceptance_radius_m
                    float('nan'),  # yaw_deg
                    float('nan'),   # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE
                ))

            ### TODO: Go into hold mode before returning
            self.get_logger().info("Going into hold mode")
            await self.drone.action.hold()
            self.get_logger().info("Hold mode activated")

            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.clear_mission()
            self.get_logger().info("Mission Cleared")
            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info("Return mission uploaded successfully")

            await self.drone.mission.start_mission()
            self.get_logger().info("Return Mission Started")

            return {"success": True, "message": "Return mission uploaded and flight mode set to MISSION"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}

    def handle_abort_mission(self, request, response):
        """Handle abort mission command - pause, hold, clear, then upload base coordinates"""
        future = asyncio.run_coroutine_threadsafe(self.execute_abort_mission(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def execute_abort_mission(self, request):
        """Execute abort mission sequence using MAVSDK"""
        try:
            self.get_logger().info("Starting abort mission sequence...")
            
            # Step 1: Pause current mission
            self.get_logger().info("Pausing current mission...")
            await self.drone.mission.pause_mission()
            self.get_logger().info("Mission paused")

            # Step 2: Create mission items for base coordinates
            self.get_logger().info("Creating abort mission to base coordinates...")
            mission_items = []
            for wp in request.waypoints:
                mission_items.append(MissionItem(
                    wp.latitude,
                    wp.longitude,
                    wp.altitude,
                    wp.speed,
                    True,  # is_fly_through
                    float('nan'),  # gimbal_pitch_deg
                    float('nan'),  # gimbal_yaw_deg
                    MissionItem.CameraAction.NONE,
                    float('nan'),  # loiter_time_s
                    float('nan'),  # camera_photo_interval_s
                    float('nan'),  # acceptance_radius_m
                    float('nan'),  # yaw_deg
                    float('nan'),   # camera_photo_distance_m
                    MissionItem.VehicleAction.NONE
                ))
            
            # Step 2: Put drone in hold mode
            self.get_logger().info("Activating hold mode...")
            await self.drone.action.hold()
            self.get_logger().info("Hold mode activated")

            # Step 3: Upload new mission (base coordinates)
            mission_plan = MissionPlan(mission_items)
            
            # Step 4: Clear current mission
            self.get_logger().info("Clearing current mission...")
            await self.drone.mission.clear_mission()
            self.get_logger().info("Mission cleared")

            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info("Abort mission uploaded successfully")
            
            # Step 6: Start the abort mission
            await self.drone.mission.start_mission()
            self.get_logger().info("Abort mission started - returning to base")

            return {"success": True, "message": "Abort mission completed - drone returning to base coordinates"}
            
        except Exception as e:
            self.get_logger().error(f"Abort mission failed: {str(e)}")
            return {"success": False, "message": f"Error: {str(e)}"}
            
    def handle_land(self, request, response):
        """Handle landing command using MAVSDK return_to_launch action"""
        future = asyncio.run_coroutine_threadsafe(self.execute_landing(request), self.loop)
        result = future.result()
        response.success = result['success']
        response.message = result['message']
        return response

    async def execute_landing(self, request):
        """Execute landing using MAVSDK return_to_launch action"""
        try:
            self.get_logger().info("Initiating drone landing using RTL action...")
            
            # Use return_to_launch action which will land the drone at current position
            await self.drone.action.return_to_launch()
            #await self.drone.action.land()
            
            self.get_logger().info("Landing command executed successfully")
            return {"success": True, "message": "Landing initiated"}
            
        except Exception as e:
            self.get_logger().error(f"Landing failed: {str(e)}")
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def rtcm_corrections_callback(self, msg: UInt8MultiArray):
        # Convert list[int] -> bytes
        frame_bytes = bytes(msg.data)
        self.get_logger().info(f"RTCM chunk received: {len(frame_bytes)} bytes")
        # Base64 encode for MAVSDK 3.x
        b64 = base64.b64encode(frame_bytes).decode('utf-8')
        # Run async call in the node's event loop
        fut = asyncio.run_coroutine_threadsafe(
            self._send_rtcm(RtcmData(b64)),
            self.loop
        )
        # Optional: catch/log errors
        try:
            fut.result(timeout=0.5)
        except Exception as e:
            self.get_logger().warn(f"RTCM send failed: {e}")

    async def _send_rtcm(self, rtcm: RtcmData):
        try:
            await self.drone.rtk.send_rtcm_data(rtcm)
        except Exception as e:
            self.get_logger().error(f"MAVSDK send_rtcm_data error: {e}")

################################################
#END - HANDLER FUNCTIONS FOR SERVICE REQUESTS FROM DRONE STATE MACHINE AND ASSOCIATED ASYNC MAVSDK FUNCTIONS 
################################################   
            
def main():
    rclpy.init()
    node = MAVSDKNode()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start())
      
    try:
        loop.run_until_complete(spin(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()
            

            