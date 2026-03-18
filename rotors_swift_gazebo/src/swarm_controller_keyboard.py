#!/usr/bin/env python3
"""
Keyboard controller for swarm_demo.
Master drone is controlled via keyboard. All other drones (read from
robots.yaml) mirror the master command automatically.

Controls:
    t / g       takeoff / land
    h           hover (lock current altitude)
    i / k       forward / backward
    j / l       left / right
    arrow up/dn throttle up / down
    s           emergency hover
    q           quit
"""

import os
import select
import sys
import termios
import threading
import tty

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from swift_msgs.msg import SwiftMsgs


def _load_slave_names():
    """Read robots.yaml and return all robot names except master."""
    cfg = os.path.join(
        get_package_share_directory("rotors_swift_gazebo"),
        "config", "robots.yaml"
    )
    with open(cfg) as f:
        fleet = yaml.safe_load(f)
    names = [r["name"] for r in fleet.get("robots", [])]
    return [n for n in names if n != "master"]


class MasterController(Node):
    def __init__(self):
        super().__init__("master_keyboard_controller")

        self.pub = self.create_publisher(SwiftMsgs, "/master/rotors/drone_command", 10)

        self.hover_thr    = 1520
        self.takeoff_thr  = 1600
        self.land_thr     = 1200
        self.roll_step    = 100
        self.pitch_step   = 100

        self.cmd        = self._neutral(self.hover_thr)
        self.is_flying  = False

        print("\n  MASTER CONTROLLER")
        print("  t=takeoff  g=land  h=hover  s=stop  q=quit")
        print("  i/k=fwd/bk  j/l=left/right  arrows=alt\n")

        t = threading.Thread(target=self._kb_loop, daemon=True)
        t.start()

        self.create_timer(0.1, self._publish)

    def _neutral(self, throttle=1500):
        cmd = SwiftMsgs()
        cmd.rc_roll     = 1500
        cmd.rc_pitch    = 1500
        cmd.rc_yaw      = 1500
        cmd.rc_throttle = throttle
        cmd.rc_aux4     = 1500
        cmd.drone_index = 0
        return cmd

    def _kb_loop(self):
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self._handle(sys.stdin.read(1).lower())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

    def _handle(self, key):
        c = self.cmd
        if key == 'i':
            c.rc_pitch = 1500 - self.pitch_step;  print("forward")
        elif key == 'k':
            c.rc_pitch = 1500 + self.pitch_step;  print("backward")
        elif key == 'j':
            c.rc_roll  = 1500 - self.roll_step;   print("left")
        elif key == 'l':
            c.rc_roll  = 1500 + self.roll_step;   print("right")
        elif key == 't':
            c = self._neutral(self.takeoff_thr);  self.is_flying = True;  print("takeoff")
        elif key == 'g':
            c = self._neutral(self.land_thr);     print("landing")
        elif key == 'h':
            c = self._neutral(self.hover_thr);    self.is_flying = True;  print("hover")
        elif key == 's':
            c = self._neutral(self.hover_thr);    self.is_flying = True;  print("stop")
        elif key == '\x1b':
            nxt = sys.stdin.read(2)
            if nxt == '[A':
                c.rc_throttle = self.takeoff_thr; print("up")
            elif nxt == '[B':
                c.rc_throttle = self.land_thr;    print("down")
        elif key == 'q':
            print("quit"); rclpy.shutdown(); return
        self.cmd = c

    def _publish(self):
        if self.is_flying:
            self.pub.publish(self.cmd)
        else:
            self.pub.publish(self._neutral())


class SlaveFollower(Node):
    """Mirrors master commands to one slave drone."""
    def __init__(self, name):
        super().__init__(f"{name}_follower")
        self.pub = self.create_publisher(SwiftMsgs, f"/{name}/rotors/drone_command", 10)
        self.create_subscription(SwiftMsgs, "/master/rotors/drone_command", self.pub.publish, 10)
        print(f"  {name} -> following master")


def main():
    rclpy.init()

    slaves = _load_slave_names()
    print(f"\n[swarm_ctrl] master + {len(slaves)} slave(s): {slaves}")

    executor = MultiThreadedExecutor()
    executor.add_node(MasterController())
    for name in slaves:
        executor.add_node(SlaveFollower(name))

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()