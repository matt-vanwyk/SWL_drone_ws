#!/usr/bin/env python3

import time
import asyncio
import threading
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FlightMode
from swl_drone_interfaces.srv import UploadMission, SetYaw, Land # Reroute, Return
from swl_drone_interfaces.msg import Telemetry

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
            
            self.get_logger().info("Landing command executed successfully")
            return {"success": True, "message": "Landing initiated"}
            
        except Exception as e:
            self.get_logger().error(f"Landing failed: {str(e)}")
            return {"success": False, "message": f"Error: {str(e)}"}

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
            

            