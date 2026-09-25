"""
measure_joint — mide en el robot el origen (home_offset) y los límites de software.

Requiere el sistema lanzado (bridge + homing) y el homing hecho. Escribe en scara.yaml.

    ros2 run scara_tools measure_joint home-offset 1        # con θ1 parada en la marca de θ1 = 0
    ros2 run scara_tools measure_joint limits 1             # ángulo de FC1_MIN y FC1_MAX → límites

home-offset — el origen se MIDE, no se supone:
  1. home_offset[j] = 0 en scara.yaml y homing (el final de home queda en 0 cuentas).
  2. Llevar la junta (a mano o con `scara_teleop jog` lento) hasta la marca física de θ = 0
     (brazo recto, cero de la tabla DH).
  3. Esta herramienta lee las cuentas N en la marca y escribe home_offset = −N (convertido a
     unidades SI con la calibración: −joint_sign·N / counts_per_rad). En general, con un
     home_offset anterior h cualquiera: nuevo = h − θ_marca.
  4. Repetir el homing y verificar que en la marca se lean ~0 cuentas: `home-offset --check`.

limits — límites de software = ángulo medido de cada final ±margen hacia adentro:
  Mueve la junta despacio (VELOCITY por /scara/raw_cmd, duty bajo) hacia cada final hasta que
  su bit se activa. El firmware la frena solo al pisarlo (enclavamiento). Registra el ángulo,
  retrocede y calcula lower = θ_MIN + margen y upper = θ_MAX − margen (3° por defecto). El
  final físico queda solo como respaldo.
"""

import argparse
import math
import os
import sys
import time

import rclpy
from scara_bridge import protocol as P
from scara_tools.calibrate_joint import _set_list_item, default_yaml_path
from scara_tools.common import is_simulation, SIM_LABEL
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, UInt8


class Probe:
    """Lee /joint_states, /scara/enc_counts y /scara/limits; manda comandos crudos."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('scara_measure_joint')
        self.js = None
        self.counts = None
        self.limits = 0
        self.node.create_subscription(JointState, '/joint_states', self._js, 10)
        self.node.create_subscription(Int32MultiArray, P.TOPIC_ENC, self._c, P.QOS_SENSOR)
        self.node.create_subscription(UInt8, P.TOPIC_LIMITS, self._l, P.QOS_SENSOR)
        self.pub = self.node.create_publisher(Int32MultiArray, P.TOPIC_RAW_CMD, 10)

    def _js(self, m):
        self.js = list(m.position)

    def _c(self, m):
        self.counts = list(m.data[:3])

    def _l(self, m):
        self.limits = m.data

    def spin(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_data(self, timeout=5.0):
        t_end = time.time() + timeout
        while (self.js is None or self.counts is None) and time.time() < t_end:
            self.spin(0.05)
        return self.js is not None and self.counts is not None

    def hold(self, data, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.pub.publish(Int32MultiArray(data=[int(x) for x in data]))
            self.spin(0.02)

    def average(self, j, seconds=0.5):
        a, c = [], []
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.spin(0.02)
            a.append(self.js[j])
            c.append(self.counts[j])
        return sum(a) / len(a), sum(c) / len(c)

    def close(self):
        self.hold([P.MODE_STOP, 0, 0, 0], 0.1)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def read_home_offset(path, j):
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    return float(data['scara_homing']['ros__parameters']['home_offset'][j])


def confirm(args, text):
    if args.yes:
        return True
    return input(f'{text} [s/N] ').strip().lower() in ('s', 'si', 'sí', 'y', 'yes')


def cmd_home_offset(args, probe, j, path, unit, to_unit):
    theta, n = probe.average(j)
    old = read_home_offset(path, j)
    new = old - theta
    print(f'En la marca: N = {n:+.1f} cuentas, θ{j + 1} = {to_unit(theta):+.3f} {unit} '
          f'(home_offset actual = {old:+.5f})')
    if args.check:
        ok = abs(n) <= args.tol_counts
        print(f'VERIFICACIÓN: {"OK" if ok else "NO OK"} (|N| = {abs(n):.1f} cuentas, '
              f'tolerancia ±{args.tol_counts})')
        return 0 if ok else 2
    print(f'⇒ home_offset[{j}] = {new:+.5f} (SI) = {to_unit(new):+.3f} {unit}')
    if not args.dry_run and confirm(args, f'¿Escribir home_offset[{j}] = {new:.5f} en {path}?'):
        with open(path) as f:
            text = f.read()
        with open(path, 'w') as f:
            f.write(_set_list_item(text, 'home_offset', j, f'{new:.5f}'))
        print('escrito. Relanzar, repetir el homing y verificar con --check en la marca.')
    return 0


def seek_switch(probe, j, which, duty, timeout):
    """Avanza despacio hasta que se activa el final 'which'. Devuelve θ (SI) o None."""
    bit = P.limit_bit(j, which)
    sign = -1 if which == 'MIN' else 1
    v = [0, 0, 0]
    v[j] = sign * duty
    t_end = time.time() + timeout
    while not (probe.limits & bit):
        if time.time() > t_end:
            probe.hold([P.MODE_STOP, 0, 0, 0], 0.3)
            return None
        probe.hold([P.MODE_VELOCITY] + v, 0.02)
    probe.hold([P.MODE_STOP, 0, 0, 0], 0.4)       # quieto (el firmware ya frenó)
    theta, _ = probe.average(j, 0.3)
    v[j] = -sign * duty                              # soltar el final
    probe.hold([P.MODE_VELOCITY] + v, 1.0)
    probe.hold([P.MODE_STOP, 0, 0, 0], 0.3)
    return theta


def cmd_limits(args, probe, j, path, unit, to_unit, prismatic):
    got = {}
    for which in ('MIN', 'MAX'):
        print(f'buscando FC{j + 1}_{which} a {args.duty} ‰ ...')
        th = seek_switch(probe, j, which, args.duty, args.timeout)
        if th is None:
            print(f'ERROR: no se activó FC{j + 1}_{which} en {args.timeout} s')
            return 2
        got[which] = th
        print(f'   FC{j + 1}_{which} se activa en θ{j + 1} = {to_unit(th):+.3f} {unit}')
    margin = args.margin_mm / 1000.0 if prismatic else math.radians(args.margin_deg)
    lo, hi = got['MIN'] + margin, got['MAX'] - margin
    mtxt = f'{args.margin_mm} mm' if prismatic else f'{args.margin_deg}°'
    print(f'⇒ límites de software (±{mtxt} hacia adentro): lower = {to_unit(lo):+.3f} {unit}, '
          f'upper = {to_unit(hi):+.3f} {unit}')
    if not args.dry_run and confirm(args, f'¿Escribir lower/upper_limits[{j}] en {path}?'):
        with open(path) as f:
            text = f.read()
        text = _set_list_item(text, 'lower_limits', j, f'{lo:.5f}')
        text = _set_list_item(text, 'upper_limits', j, f'{hi:.5f}')
        with open(path, 'w') as f:
            f.write(text)
        print('escrito. Relanzar el bringup para aplicarlos.')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='Medir home_offset y límites de software')
    ap.add_argument('what', choices=['home-offset', 'limits'])
    ap.add_argument('joint', type=int, choices=[1, 2, 3])
    ap.add_argument('--check', action='store_true', help='home-offset: solo verificar ~0 cuentas')
    ap.add_argument('--tol-counts', type=float, default=20.0)
    ap.add_argument('--duty', type=int, default=150, help='limits: duty de aproximación (‰)')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--margin-deg', type=float, default=3.0)
    ap.add_argument('--margin-mm', type=float, default=3.0)
    ap.add_argument('--yaml', default='')
    ap.add_argument('--yes', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    if argv is None:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    args = ap.parse_args(argv)

    j = args.joint - 1
    prismatic = j == 2
    unit = 'mm' if prismatic else '°'

    def to_unit(x):
        return x * 1000.0 if prismatic else math.degrees(x)

    path = args.yaml or default_yaml_path()
    if not os.path.exists(path):
        print(f'ERROR: no existe {path}')
        return 1
    probe = Probe()
    try:
        if not probe.wait_data():
            print('ERROR: no llegan /joint_states o /scara/enc_counts (¿bringup lanzado?)')
            return 1
        if is_simulation(probe.node):
            print(f'*** {SIM_LABEL} ***')
        if args.what == 'home-offset':
            return cmd_home_offset(args, probe, j, path, unit, to_unit)
        return cmd_limits(args, probe, j, path, unit, to_unit, prismatic)
    except KeyboardInterrupt:
        return 130
    finally:
        probe.close()


if __name__ == '__main__':
    sys.exit(main())
