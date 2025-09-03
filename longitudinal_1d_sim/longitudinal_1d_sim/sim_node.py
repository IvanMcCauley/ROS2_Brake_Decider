from std_msgs.msg import Float64

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SimNode(Node):
    def __init__(self):   # constructor
        super().__init__('sim_node') #calls parent node and sets node name

        # --- Parameters (simple defaults but will externalize later)
        self.declare_parameter('dt', 0.05)           # time step [s]  (20 Hz)
        self.declare_parameter('v0', 20.0)           # initial speed [m/s] (~72 km/h)
        self.declare_parameter('d0', 80.0)           # initial distance to obstacle [m]
        self.declare_parameter('decel', 9.0)         # braking deceleration magnitude [m/s^2]

        # Read parameters
        self.dt = float(self.get_parameter('dt').value)
        self.v = float(self.get_parameter('v0').value)
        self.x = 0.0
        self.x = 0.0   # position [m], start at 0
        self.x_obs = float(self.get_parameter('d0').value)  # obstacle at +d0
        self.decel = float(self.get_parameter('decel').value)  

        # Control state driven by /brake_cmd
        self.brake_cmd = False
        self.last_brake_cmd = None

        # I/O
        self.subscription = self.create_subscription(Bool, '/brake_cmd', self.brake_callback, 10)    
        self.get_logger().info('Sim node started, listening to /brake_cmd')
        
        self.pub_speed = self.create_publisher(Float64, '/ego_speed', 10)  # /ego_speed
        self.pub_dist  = self.create_publisher(Float64, '/obstacle_distance', 10) # /obstacle_distance

        # CSV logging setup (no extra libs): create a timestamped file in ~/ros2_ws/logs/
        logs_dir = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(logs_dir, exist_ok=True)
        self.csv_path = os.path.join(logs_dir, f'longitudinal_sim_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        with open(self.csv_path, 'w') as f:
            f.write('t,x,v,d,brake_cmd,u\n')

        # Timer: runs at 1/dt Hz; calls self.on_tick each period
        # create_timer(period_sec, callback)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.on_tick)
        self.finished = False
        self.tick = 0  # heartbeat

    def brake_callback(self, msg: Bool):
        new_val = bool(msg.data)
        if new_val != self.brake_cmd:
            self.brake_cmd = new_val
            self.get_logger().info(f'brake_cmd changed -> {self.brake_cmd}')



    def on_tick(self):
        self.tick += 1

        # Distance to obstacle
        d = self.x_obs - self.x

        # publish current speed and (non-negative) distance
        m_v = Float64(); m_v.data = float(self.v)
        self.pub_speed.publish(m_v)

        m_d = Float64(); m_d.data = float(d if d > 0.0 else 0.0)
        self.pub_dist.publish(m_d)

        # Control method
        u = -self.decel if self.brake_cmd else 0.0

        # Update dynamics
        new_v = self.v + u * self.dt
        if new_v < 0.0:
            new_v = 0.0
        self.x = self.x + self.v * self.dt  # integrate position with current v
        self.v = new_v

        # Log row
        with open(self.csv_path, 'a') as f:
            f.write(f'{self.t:.3f},{self.x:.3f},{self.v:.3f},{d:.3f},{int(self.brake_cmd)},{u:.3f}\n')

        # Heartbeat every 0.5 s
        if self.tick % 10 == 0:
            self.get_logger().info(f't={self.t:.2f}s v={self.v:.2f} m/s d={d:.2f} m brake={self.brake_cmd}')

        # Outcome (one-time)
        if not self.finished:
            # Success: fully stopped and obstacle still ahead
            if (self.v <= 1e-3) and (d >= 0.0):
                self.finished = True
                self.get_logger().info(
                    f'SUCCESS — stopped before obstacle at t={self.t:.2f}s; d={d:.2f} m; v={self.v:.2f} m/s. CSV: {self.csv_path}'
                )
                self.timer.cancel()
                return
            # Collision: crossed obstacle while still moving
            if (d < 0.0) and (self.v > 1e-3):
                self.finished = True
                self.get_logger().info(
                    f'COLLISION — passed obstacle while moving at t={self.t:.2f}s; d={d:.2f} m; v={self.v:.2f} m/s. CSV: {self.csv_path}'
                )
                self.timer.cancel()
                return

        # Advance time only if still running
        self.t += self.dt

def main(args=None):
    rclpy.init(args=args) # initializes ROS client library
    node = SimNode()      # create node instance
    rclpy.spin(node)      #keep node alive
    node.destroy_node()   # cleanup when shutting down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
