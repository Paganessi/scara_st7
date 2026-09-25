"""goto — mover el SCARA a una pose articular desde la terminal.

    ros2 run scara_teleop goto 30 -45 50          # θ1 = 30°, θ2 = −45°, s3 = 50 mm
    ros2 run scara_teleop goto 0 0 0 --no-wait    # publica y sale sin esperar

Convierte grados/mm → rad/m (REP 103), publica /scara/joint_goal y (por defecto) espera
a que /joint_states llegue a la meta, mostrando el error. El bridge es quien valida
límites y homing: si rechaza la meta, lo dice en su log y aquí se vence el tiempo.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from scara_bridge import protocol as P

NAMES = ['joint1', 'joint2', 'joint3']


class GotoNode(Node):

    def __init__(self):
        super().__init__('scara_goto')
        self.pub = self.create_publisher(JointState, P.TOPIC_JOINT_GOAL, 10)
        self.pos = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)

    def on_js(self, msg: JointState):
        if all(n in msg.name for n in NAMES):
            self.pos = [msg.position[msg.name.index(n)] for n in NAMES]


def main(argv=None):
    ap = argparse.ArgumentParser(description='Mover el SCARA a (θ1[°], θ2[°], s3[mm])')
    ap.add_argument('theta1_deg', type=float)
    ap.add_argument('theta2_deg', type=float)
    ap.add_argument('s3_mm', type=float)
    ap.add_argument('--no-wait', action='store_true', help='no esperar a que llegue')
    ap.add_argument('--timeout', type=float, default=60.0, help='s de espera máx.')
    ap.add_argument('--tol-deg', type=float, default=0.5)
    ap.add_argument('--tol-mm', type=float, default=0.5)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv else sys.argv)[1:])

    goal = [math.radians(args.theta1_deg), math.radians(args.theta2_deg), args.s3_mm / 1000.0]
    rclpy.init(args=argv)
    node = GotoNode()
    try:
        # Esperar a que el bridge esté suscrito (si no, el mensaje se pierde).
        t0 = time.time()
        while node.pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pub.get_subscription_count() == 0:
            print('ERROR: nadie escucha /scara/joint_goal (¿está corriendo el bridge?)')
            return 1
        node.pub.publish(JointState(name=NAMES, position=goal))
        print(f'meta enviada: θ1={args.theta1_deg}° θ2={args.theta2_deg}° s3={args.s3_mm} mm')
        if args.no_wait:
            return 0

        tol = [math.radians(args.tol_deg), math.radians(args.tol_deg), args.tol_mm / 1000.0]
        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.pos is None:
                continue
            err = [g - p for g, p in zip(goal, node.pos)]
            if time.time() - last_print > 0.5:
                last_print = time.time()
                print(f'  error: θ1={math.degrees(err[0]):+7.2f}°  θ2={math.degrees(err[1]):+7.2f}°'
                      f'  s3={err[2] * 1000:+7.2f} mm')
            if all(abs(e) <= t for e, t in zip(err, tol)):
                print(f'LLEGÓ en {time.time() - t0:.1f} s')
                return 0
        print('TIEMPO VENCIDO sin llegar (¿meta rechazada por el bridge? ver su log)')
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
