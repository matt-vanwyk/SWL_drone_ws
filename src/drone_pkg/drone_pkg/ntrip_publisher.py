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
        # self.declare_parameter('mountpoint', 'Spiderweb')
        # self.declare_parameter('user', 's224482378-at-mandela.ac.za')
        # self.declare_parameter('password', 'DSm2aDn2')
        self.declare_parameter('mountpoint', 'Renewables')
        self.declare_parameter('user', 's229701698-at-mandela.ac.za')
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

