# #!/usr/bin/env python3
# """
# NTRIP Publisher Node - Process Monitor Approach
# Runs NTRIP client as separate process, monitors output file
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import os
# import base64
# import subprocess
# import signal
# import sys

# class NTRIPPublisherNode(Node):
#     def __init__(self):
#         super().__init__('ntrip_publisher')
        
#         # ROS2 publisher for RTCM corrections
#         self.rtcm_publisher = self.create_publisher(
#             String,
#             'rtcm_corrections',
#             10
#         )
        
#         # NTRIP configuration
#         self.declare_parameter('ntrip_server', 'rtk2go.com')
#         self.declare_parameter('ntrip_port', 2101)
#         self.declare_parameter('ntrip_mountpoint', 'Renewables')
#         self.declare_parameter('ntrip_user', 's224482378-at-mandela.ac.za')
#         self.declare_parameter('ntrip_password', 'mukandagumbo')
        
#         # File for RTCM data
#         self.rtcm_file = '/tmp/rtcm_ntrip.log'
#         self.last_file_size = 0
#         self.ntrip_process = None
        
#         # Start NTRIP process
#         self.start_ntrip_process()
        
#         # Start file monitor timer
#         self.monitor_timer = self.create_timer(0.5, self.monitor_rtcm_file)
        
#         self.get_logger().info('NTRIP Publisher Node started')
    
#     def start_ntrip_process(self):
#         """Start NTRIP client as separate Python process"""
#         # Remove old file
#         if os.path.exists(self.rtcm_file):
#             os.remove(self.rtcm_file)
        
#         server = self.get_parameter('ntrip_server').value
#         port = self.get_parameter('ntrip_port').value
#         mountpoint = self.get_parameter('ntrip_mountpoint').value
#         user = self.get_parameter('ntrip_user').value
#         password = self.get_parameter('ntrip_password').value
        
#         # Use the standalone script that we know works
#         cmd = [
#             'python3', '/home/spiderweb/ntrip_standalone.py',
#             server, str(port), mountpoint, user, password, self.rtcm_file
#         ]
        
#         self.get_logger().info(f"Starting NTRIP process: {server}:{port}/{mountpoint}")
#         self.ntrip_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         self.get_logger().info(f"NTRIP process started with PID {self.ntrip_process.pid}")
    
#     def monitor_rtcm_file(self):
#         """Monitor RTCM file for new data and publish to ROS topic"""
#         try:
#             # Check if process is still running
#             if self.ntrip_process and self.ntrip_process.poll() is not None:
#                 self.get_logger().error(f"NTRIP process died with return code {self.ntrip_process.returncode}")
#                 return
            
#             if not os.path.exists(self.rtcm_file):
#                 return
                
#             current_size = os.path.getsize(self.rtcm_file)
            
#             if current_size > self.last_file_size:
#                 # Read new data
#                 with open(self.rtcm_file, "rb") as f:
#                     f.seek(self.last_file_size)
#                     new_data = f.read(current_size - self.last_file_size)
                    
#                     if new_data:
#                         # Split into chunks
#                         chunk_size = 1024
#                         for i in range(0, len(new_data), chunk_size):
#                             chunk = new_data[i:i + chunk_size]
#                             base64_data = base64.b64encode(chunk).decode('utf-8')
                            
#                             msg = String()
#                             msg.data = base64_data
#                             self.rtcm_publisher.publish(msg)
                        
#                         self.get_logger().info(f"Published {len(new_data)} bytes of RTCM data")
                
#                 self.last_file_size = current_size
                
#         except Exception as e:
#             self.get_logger().error(f"Error monitoring RTCM file: {e}")
    
#     def destroy_node(self):
#         """Clean up on node shutdown"""
#         if self.ntrip_process:
#             self.ntrip_process.terminate()
#         try:
#             if os.path.exists(self.rtcm_file):
#                 os.remove(self.rtcm_file)
#         except:
#             pass
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
    
#     ntrip_node = NTRIPPublisherNode()
    
#     try:
#         rclpy.spin(ntrip_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ntrip_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import threading
from queue import Queue, Empty
from logging import getLogger
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from pygnssutils import VERBOSITY_MEDIUM, GNSSNTRIPClient, set_logging

class NtripPublisher(Node):
    def __init__(self):
        super().__init__('ntrip_publisher')

        # ---- Parameters ----
        self.declare_parameter('server', 'rtk2go.com')
        self.declare_parameter('port', 2101)
        self.declare_parameter('https', 0)                 # 0=http, 1=https
        self.declare_parameter('datatype', 'RTCM')
        self.declare_parameter('mountpoint', 'Renewables')
        self.declare_parameter('user', 's224482378-at-mandela.ac.za')
        self.declare_parameter('password', 'mukandagumbo')
        # GGA control (usually not needed for RTK2go single-base)
        self.declare_parameter('ggainterval', -1)          # -1 none, 0 once, >0 sec
        self.declare_parameter('ggamode', 1)               # 1=fixed ref coords
        self.declare_parameter('reflat', 0.0)
        self.declare_parameter('reflon', 0.0)
        self.declare_parameter('refalt', 0.0)
        self.declare_parameter('refsep', 0.0)

        # ---- Publisher ----
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(UInt8MultiArray, 'rtcm/frames', qos)

        # ---- Logging for pygnssutils ----
        logger = getLogger("pygnssutils.gnssntripclient")
        set_logging(logger, VERBOSITY_MEDIUM)

        # ---- Shared queue & stop flag ----
        self._outqueue: Queue = Queue(maxsize=1000)
        self._stop = threading.Event()

        # ---- Read params ----
        self._args = dict(
            server       = self.get_parameter('server').value,
            port         = int(self.get_parameter('port').value),
            https        = int(self.get_parameter('https').value),
            datatype     = self.get_parameter('datatype').value,
            mountpoint   = self.get_parameter('mountpoint').value,
            user         = self.get_parameter('user').value,
            password     = self.get_parameter('password').value,
            ggainterval  = int(self.get_parameter('ggainterval').value),
            ggamode      = int(self.get_parameter('ggamode').value),
            reflat       = float(self.get_parameter('reflat').value),
            reflon       = float(self.get_parameter('reflon').value),
            refalt       = float(self.get_parameter('refalt').value),
            refsep       = float(self.get_parameter('refsep').value),
        )

        if not self._args['mountpoint']:
            self.get_logger().error("Parameter 'mountpoint' is required.")
            raise SystemExit(2)

        self.get_logger().info(
            f"Starting NTRIP stream: {self._args['server']}:{self._args['port']}/"
            f"{self._args['mountpoint']} (ggainterval={self._args['ggainterval']})"
        )

        # ---- Threads ----
        self._ntrip_thread = threading.Thread(target=self._ntripthread, daemon=True)
        self._data_thread  = threading.Thread(target=self._datathread, daemon=True)
        self._ntrip_thread.start()
        self._data_thread.start()

        # Heartbeat
        self._pub_count = 0
        self.create_timer(5.0, self._heartbeat)

    def _heartbeat(self):
        self.get_logger().debug(f"RTCM published so far: {self._pub_count}")

    # Thread: run GNSSNTRIPClient and push (raw, parsed) into queue
    def _ntripthread(self):
        try:
            gnc = GNSSNTRIPClient()
            gnc.run(
                server        = self._args['server'],
                port          = self._args['port'],
                https         = self._args['https'],
                mountpoint    = self._args['mountpoint'],
                datatype      = self._args['datatype'],
                ntripuser     = self._args['user'],
                ntrippassword = self._args['password'],
                ggainterval   = self._args['ggainterval'],
                ggamode       = self._args['ggamode'],
                reflat        = self._args['reflat'],
                reflon        = self._args['reflon'],
                refalt        = self._args['refalt'],
                refsep        = self._args['refsep'],
                output        = self._outqueue,  
            )
            # Keep thread alive
            while rclpy.ok() and not self._stop.is_set():
                import time; time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"NTRIP thread error: {e}")

    # Thread: drain queue and publish raw bytes
    def _datathread(self):
        try:
            while rclpy.ok() and not self._stop.is_set():
                try:
                    raw, parsed = self._outqueue.get(timeout=1.0)
                except Empty:
                    continue

                # raw: bytes-like RTCM payload(s)
                if raw:
                    # Publish as UInt8MultiArray
                    msg = UInt8MultiArray(data=list(bytes(raw)))
                    self.pub.publish(msg)
                    self._pub_count += 1
                    if self._pub_count % 50 == 0:
                        self.get_logger().info(
                            f"Published {self._pub_count} RTCM chunks "
                            f"(last msg {getattr(parsed, 'identity', 'unknown')})"
                        )

                self._outqueue.task_done()
        except Exception as e:
            self.get_logger().error(f"Data thread error: {e}")

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

def main():
    rclpy.init()
    node = NtripPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

