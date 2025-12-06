#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from swift_msgs.msg import SwiftMsgs
import sys
import select
import tty
import termios
import threading

class MasterKeyboardController(Node):
    """Master drone keyboard controller"""
    def __init__(self):
        super().__init__('master_keyboard_controller')
        
        # Publisher for master only
        self.master_pub = self.create_publisher(SwiftMsgs, '/master/rotors/drone_command', 10)
        
        # Tuned control values
        self.hover_throttle = 1520
        self.takeoff_throttle = 1600
        self.land_throttle = 1200
        
        self.roll_increment = 100
        self.pitch_increment = 100
        
        # Current command
        self.cmd = SwiftMsgs()
        self.reset_command()
        self.cmd.rc_throttle = 1500  # Start grounded
        
        # Flying state
        self.is_flying = False
        
        # Simple instructions
        print("\n" + "="*50)
        print("MASTER CONTROLLER")
        print("="*50)
        print("i=Forward  k=Backward  j=Left  l=Right")
        print("↑=Up  ↓=Down  h=Hover  s=Stop")
        print("t=Takeoff  g=Land  q=Quit")
        print("="*50)
        print("TIP: Press 'h' anytime to lock hover!")
        print("="*50 + "\n")
        
        # Keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    def reset_command(self):
        """Reset to neutral hover"""
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = self.hover_throttle
        self.cmd.rc_aux1 = 0
        self.cmd.rc_aux2 = 0
        self.cmd.rc_aux3 = 0
        self.cmd.rc_aux4 = 1500
        self.cmd.drone_index = 0

    def keyboard_listener(self):
        """Listen for keyboard"""
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    self.process_key(key)
                    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def process_key(self, key):
        """Process keyboard input"""
        
        if key == 'i':  # Forward - momentary
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500 - self.pitch_increment
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            print("Forward")
            
        elif key == 'k':  # Backward - momentary
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500 + self.pitch_increment
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            print("Backward")
            
        elif key == 'j':  # Left - momentary
            self.cmd.rc_roll = 1500 - self.roll_increment
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            print("Left")
            
        elif key == 'l':  # Right - momentary
            self.cmd.rc_roll = 1500 + self.roll_increment
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            print("Right")
            
        elif key == '\x1b':  # Arrow keys
            next_chars = sys.stdin.read(2)
            if next_chars == '[A':  # Up - continuous until hover
                self.cmd.rc_roll = 1500
                self.cmd.rc_pitch = 1500
                self.cmd.rc_yaw = 1500
                self.cmd.rc_throttle = self.takeoff_throttle
                print("Up")
            elif next_chars == '[B':  # Down - continuous until hover
                self.cmd.rc_roll = 1500
                self.cmd.rc_pitch = 1500
                self.cmd.rc_yaw = 1500
                self.cmd.rc_throttle = self.land_throttle
                print("Down")
                
        elif key == 't':  # Takeoff - switches to hover after climb
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.takeoff_throttle
            self.is_flying = True
            print("TAKEOFF - Press 'h' to stop climbing")
            
        elif key == 'g':  # Land - switches to hover after descent
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.land_throttle
            print("LANDING - Press 'h' to stop descending")
            
        elif key == 'h':  # Hover - LOCKS current altitude
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            self.is_flying = True  # Ensure it stays in flying mode
            print("HOVER LOCKED")
            
        elif key == 's':  # STOP - emergency hover
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = self.hover_throttle
            self.is_flying = True
            print("EMERGENCY HOVER")
            
        elif key == 'q':  # Quit
            print("Shutting down...")
            rclpy.shutdown()

    def publish_commands(self):
        """Publish master command"""
        if self.is_flying:
            self.master_pub.publish(self.cmd)
        else:
            # Keep grounded when not flying
            grounded = SwiftMsgs()
            grounded.rc_roll = 1500
            grounded.rc_pitch = 1500
            grounded.rc_yaw = 1500
            grounded.rc_throttle = 1500
            grounded.rc_aux1 = 0
            grounded.rc_aux2 = 0
            grounded.rc_aux3 = 0
            grounded.rc_aux4 = 1500
            grounded.drone_index = 0
            
            self.master_pub.publish(grounded)


class SlaveFollower(Node):
    """Slave drone that follows master commands"""
    def __init__(self, slave_name):
        super().__init__(f'{slave_name}_follower')
        
        self.slave_name = slave_name
        
        # Subscribe to master commands
        self.master_sub = self.create_subscription(
            SwiftMsgs,
            '/master/rotors/drone_command',
            self.master_command_callback,
            10
        )
        
        # Publisher for this slave
        self.slave_pub = self.create_publisher(
            SwiftMsgs,
            f'/{slave_name}/rotors/drone_command',
            10
        )
        
        print(f"{slave_name} following master commands...")

    def master_command_callback(self, msg):
        """Receive master command and publish to slave"""
        # Simply republish the same command to this slave
        self.slave_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create master controller
        master_controller = MasterKeyboardController()
        
        # Create slave followers
        slave1_follower = SlaveFollower('slave1')
        slave2_follower = SlaveFollower('slave2')
        slave3_follower = SlaveFollower('slave3')
        
        # Create executor to spin all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(master_controller)
        executor.add_node(slave1_follower)
        executor.add_node(slave2_follower)
        executor.add_node(slave3_follower)
        
        print("\n✓ Master controller ready")
        print("✓ All slaves listening to master\n")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()