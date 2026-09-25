"""
jog — mover el SCARA con el teclado, a pasos, en espacio articular.

    ros2 run scara_teleop jog

    q / a : θ1 + / −        w / s : θ2 + / −        e / d : s3 + / −
    + / - : paso más grande / más chico              espacio : STOP     x : salir

Cada tecla publica una meta = posición actual + paso en /scara/joint_goal. El bridge
aplica límites, homing obligatorio y la rampa de velocidad.
"""

import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from scara_bridge import protocol as P
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

NAMES = ['joint1', 'joint2', 'joint3']
KEYS = {'q': (0, 1), 'a': (0, -1), 'w': (1, 1), 's': (1, -1), 'e': (2, 1), 'd': (2, -1)}
STEPS_DEG = [0.5, 1.0, 2.0, 5.0, 10.0]   # para θ1, θ2
STEPS_MM = [0.2, 0.5, 1.0, 2.0, 5.0]     # para s3


class JogNode(Node):

    def __init__(self):
        super().__init__('scara_jog')
        self.pub = self.create_publisher(JointState, P.TOPIC_JOINT_GOAL, 10)
        self.stop_cli = self.create_client(Trigger, '/scara/stop')
        self.pos = None
        self.target = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)

    def on_js(self, msg: JointState):
        if all(n in msg.name for n in NAMES):
            self.pos = [msg.position[msg.name.index(n)] for n in NAMES]


def main(args=None):
    rclpy.init(args=args)
    node = JogNode()
    level = 1
    old = termios.tcgetattr(sys.stdin)
    print(__doc__)
    enabled = P.fetch_enabled_from_bridge(node)
    print(f'juntas habilitadas: {enabled}')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            c = sys.stdin.read(1)
            if c == 'x':
                break
            if c == ' ':
                if node.stop_cli.service_is_ready():
                    node.stop_cli.call_async(Trigger.Request())
                node.target = None
                print('STOP')
                continue
            if c in '+-':
                level = max(0, min(len(STEPS_DEG) - 1, level + (1 if c == '+' else -1)))
                print(f'paso: {STEPS_DEG[level]}° / {STEPS_MM[level]} mm')
                continue
            if c not in KEYS or node.pos is None:
                continue
            j, sgn = KEYS[c]
            if not enabled[j]:
                print(f'{NAMES[j]} deshabilitada (joints_enabled en scara.yaml)')
                continue
            # Acumular sobre la meta anterior si existe (varias teclas seguidas = más lejos).
            base = list(node.target if node.target is not None else node.pos)
            step = math.radians(STEPS_DEG[level]) if j < 2 else STEPS_MM[level] / 1000.0
            base[j] += sgn * step
            node.target = base
            node.pub.publish(JointState(name=NAMES, position=base))
            print(f'meta: θ1={math.degrees(base[0]):7.2f}°  θ2={math.degrees(base[1]):7.2f}°  '
                  f's3={base[2] * 1000:7.2f} mm')
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
