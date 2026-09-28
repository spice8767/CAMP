import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
import math

class WASDTeleop(Node):
    def __init__(self, label):
        super().__init__('wasd_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.keys_pressed = {'w': False, 'a': False, 's': False, 'd': False}
        self.linear_speed = 3.0  # m/s
        
        # We track simulated velocity to match Gazebo's internal acceleration profile
        self.current_v = 0.0
        self.max_accel = 2.0  # Must match <max_acceleration> in URDF
        self.dt = 0.05        # 20Hz refresh
        self.wheel_base = 2.124 # Must match <wheel_base> in URDF
        
        self.label = label
        
    def publish_twist(self):
        msg = Twist()
        
        # 1. Determine Target Velocity
        target_v = 0.0
        if self.keys_pressed['w']:
            target_v = self.linear_speed
        elif self.keys_pressed['s']:
            target_v = -self.linear_speed
            
        # 2. Simulate Gazebo's internal acceleration (to perfectly predict what it thinks current velocity is)
        if target_v > self.current_v:
            self.current_v += min(target_v - self.current_v, self.max_accel * self.dt)
        else:
            self.current_v += max(target_v - self.current_v, -self.max_accel * self.dt)
            
        msg.linear.x = self.current_v

        # 3. Calculate mathematically perfect steering input
        # Gazebo Ackermann plugin calculates steering angle: theta = atan(wheel_base * angular.z / linear.x)
        # To force theta to be EXACTLY 0.8 rad (max steering), we reverse the math:
        
        target_angle = 0.0
        if self.keys_pressed['a']:
            target_angle = 0.8
        elif self.keys_pressed['d']:
            target_angle = -0.8
            
        if abs(self.current_v) < 0.05:
            # If parked, linear.x is ~0, so we just send a large angular.z to force the wheels to turn
            msg.angular.z = 2.5 if target_angle > 0 else (-2.5 if target_angle < 0 else 0.0)
        else:
            # If moving, we send exactly the yaw rate required to achieve the desired physical steering angle
            multiplier = math.tan(target_angle) / self.wheel_base
            msg.angular.z = multiplier * self.current_v
            
        self.publisher_.publish(msg)

    def update_label(self):
        pressed = [k.upper() for k, v in self.keys_pressed.items() if v]
        status = " + ".join(pressed) if pressed else "None"
        text = f"Click here to focus!\n\nUse W/S for Gas/Brake\nUse A/D for Steering\n\nCurrently Pressing: [{status}]\nSim Velocity: {self.current_v:.2f} m/s"
        self.label.config(text=text)

    def on_press(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed[key] = True
            self.update_label()
            
    def on_release(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed:
            self.keys_pressed[key] = False
            self.update_label()

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    root.title("WASD Teleop")
    root.geometry("300x200")
    label = tk.Label(root, text="Loading...", font=("Arial", 11))
    label.pack(expand=True)
    
    node = WASDTeleop(label)
    node.update_label()
    
    root.bind('<KeyPress>', node.on_press)
    root.bind('<KeyRelease>', node.on_release)
    
    def loop():
        rclpy.spin_once(node, timeout_sec=0)
        node.publish_twist()
        root.after(int(node.dt * 1000), loop)
        
    root.after(50, loop)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
