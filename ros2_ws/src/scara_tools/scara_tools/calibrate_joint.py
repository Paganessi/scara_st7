"""
calibrate_joint — mide cuentas por radián (o por metro) y las escribe en scara.yaml.

Escribe counts_per_rad / counts_per_m y el signo de la junta (joint_sign).

Idea: la relación teórica (64 cuentas × 50:1 × polea 2:1 / 2π ≈ 1018.6 cuentas/rad) depende
de datos sin confirmar. Medirla es fácil: poner la junta en una marca, girarla un ángulo
conocido (una vuelta, o 90° hasta otra marca) y ver cuántas cuentas cambió el encoder.

    # Firmware de PRUEBA de la Fase 1 (sin ROS), por el puerto serie:
    ros2 run scara_tools calibrate_joint 1 --angle 90 --serial /dev/ttyUSB0
    # Firmware real / simulador (lee /scara/enc_counts; con el bridge apagado si se usa --jog):
    ros2 run scara_tools calibrate_joint 1 --angle 360
    # Eje Z (prismático): distancia en mm
    ros2 run scara_tools calibrate_joint 3 --distance-mm 50 --serial /dev/ttyUSB0

Cómo mover la junta entre las dos marcas:
  - a mano (por defecto). Con --serial se pone el motor en RUEDA LIBRE (comando "c") para que
    no frene; los encoders necesitan los 12 V encendidos. Si la reductora 50:1 es muy dura
    para girarla desde la junta, usar --jog.
  - --jog: con el motor, a pulsos cortos de duty bajo: tecla "d" = sentido +, "a" = sentido −,
    Enter = llegué a la marca. (Con ROS, publica en /scara/cmd: SOLO con el bridge apagado.)

El sentido POSITIVO de θ1/θ2 es ANTIHORARIO visto desde arriba (REP 103, z hacia arriba);
el de Z (s3) es hacia ABAJO. Se gira en sentido positivo: si las cuentas bajan, joint_sign = −1.
"""

import argparse
import math
import os
import re
import select
import statistics
import sys
import termios
import threading
import time
import tty

PULSE_MS = 60   # duración de cada pulso de jog


# ============================================================ fuentes de cuentas
class SerialSource:
    """Firmware de prueba de la Fase 1: imprime "c1=... c2=... c3=..." cada 200 ms."""

    RX = re.compile(r'c1=\s*(-?\d+)\s+c2=\s*(-?\d+)\s+c3=\s*(-?\d+)')

    def __init__(self, port, joint, jog_duty):
        import serial   # pyserial (python3-serial)
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.05
        self.ser.dtr = False    # no resetear el ESP32 al abrir el puerto
        self.ser.rts = False
        self.ser.open()
        self.joint, self.jog_duty = joint, jog_duty
        self.counts = None
        self.stamp = 0.0
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b''
        while not self._stop:
            buf += self.ser.read(256)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                m = self.RX.search(line.decode(errors='replace'))
                if m:
                    self.counts = [int(x) for x in m.groups()]
                    self.stamp = time.time()

    def send(self, text):
        self.ser.write(text.encode() + b'\r')

    def free(self):
        self.send(f'c {self.joint}')        # rueda libre para girar a mano

    def jog(self, sign):
        self.send(f'm {self.joint} {sign * self.jog_duty} {PULSE_MS}')

    def close(self):
        self.send('s')
        self._stop = True
        time.sleep(0.1)
        self.ser.close()


class RosSource:
    """Lee /scara/enc_counts (firmware micro-ROS real o simulador)."""

    def __init__(self, joint, jog_duty):
        import rclpy
        from std_msgs.msg import Int32MultiArray

        from scara_bridge import protocol as P
        self.rclpy, self.P, self.Msg = rclpy, P, Int32MultiArray
        rclpy.init()
        self.node = rclpy.create_node('scara_calibrate_joint')
        self.pub = None
        self.joint, self.jog_duty = joint, jog_duty
        self.counts = None
        self.stamp = 0.0
        self.node.create_subscription(Int32MultiArray, P.TOPIC_ENC, self._on, P.QOS_SENSOR)
        self._stop = False
        threading.Thread(target=self._spin, daemon=True).start()

    def _on(self, msg):
        self.counts = list(msg.data[:3])
        self.stamp = time.time()

    def _spin(self):
        while not self._stop and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def free(self):
        pass    # el firmware real no tiene rueda libre: girar con --jog (o en simulación)

    def jog(self, sign):
        if self.pub is None:
            self.pub = self.node.create_publisher(self.Msg, self.P.TOPIC_CMD, self.P.QOS_CMD)
            time.sleep(0.3)
        v = [0, 0, 0]
        v[self.joint - 1] = sign * self.jog_duty
        t_end = time.time() + PULSE_MS / 1000.0
        while time.time() < t_end:
            self.pub.publish(self.Msg(data=[self.P.MODE_VELOCITY] + v))
            time.sleep(0.02)
        self.pub.publish(self.Msg(data=[self.P.MODE_STOP, 0, 0, 0]))

    def close(self):
        if self.pub is not None:
            self.pub.publish(self.Msg(data=[self.P.MODE_STOP, 0, 0, 0]))
        self._stop = True
        time.sleep(0.1)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# ============================================================ cálculo y YAML
def compute(delta_counts, amount, prismatic):
    """Calcular (cuentas/unidad, signo); amount en rad (rotacional) o m (prismática)."""
    if delta_counts == 0 or amount <= 0:
        raise ValueError('sin cambio de cuentas: ¿encoder conectado y con 12 V?')
    return abs(delta_counts) / amount, (1 if delta_counts > 0 else -1)


def _set_list_item(text, key, index, value):
    """
    Cambiar el elemento 'index' de una lista YAML de una línea ('key: [a, b, c]').

    Conserva los comentarios del archivo.
    """
    rx = re.compile(r'^(\s*' + re.escape(key) + r':\s*\[)([^\]]*)(\].*)$', re.M)
    m = rx.search(text)
    if not m:
        raise KeyError(f'no encuentro "{key}: [...]" en el YAML')
    items = [x.strip() for x in m.group(2).split(',')]
    items[index] = value
    return text[:m.start()] + m.group(1) + ', '.join(items) + m.group(3) + text[m.end():]


def _set_scalar(text, key, value):
    rx = re.compile(r'^(\s*' + re.escape(key) + r':\s*)([^\s#]+)(.*)$', re.M)
    m = rx.search(text)
    if not m:
        raise KeyError(f'no encuentro "{key}:" en el YAML')
    return text[:m.start()] + m.group(1) + value + m.group(3) + text[m.end():]


def update_yaml(path, joint, per_unit, sign):
    """Escribe la calibración de la junta (1..3) en el YAML y devuelve el texto nuevo."""
    with open(path) as f:
        text = f.read()
    if joint in (1, 2):
        text = _set_list_item(text, 'counts_per_rad', joint - 1, f'{per_unit:.2f}')
    else:
        text = _set_scalar(text, 'counts_per_m', f'{per_unit:.1f}')
    text = _set_list_item(text, 'joint_sign', joint - 1, str(sign))
    with open(path, 'w') as f:
        f.write(text)
    return text


def default_yaml_path():
    """El scara.yaml de las fuentes (con --symlink-install el instalado es un enlace)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('scara_bringup'), 'config', 'scara.yaml')
        return os.path.realpath(p)
    except Exception:  # noqa: BLE001
        return os.path.expanduser('~/scara_st7/ros2_ws/src/scara_bringup/config/scara.yaml')


# ============================================================ interacción
def wait_enter_with_jog(src, allow_jog, prompt):
    """Espera Enter mostrando las cuentas en vivo. Con jog: 'a'/'d' mueven a pulsos."""
    print(prompt)
    j = src.joint - 1
    if not sys.stdin.isatty():          # uso no interactivo (pruebas): una línea = Enter
        sys.stdin.readline()
        return
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            c = '' if src.counts is None else src.counts[j]
            sys.stdout.write(f'\r   cuentas junta {src.joint}: {c}          ')
            sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1)
                if k in ('\n', '\r'):
                    print()
                    return
                if allow_jog and k in 'ad':
                    src.jog(1 if k == 'd' else -1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def read_stable(src, seconds=0.5):
    """Promedio de las cuentas durante 'seconds' (quita el parpadeo de ±1)."""
    j = src.joint - 1
    vals = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        if src.counts is not None:
            vals.append(src.counts[j])
        time.sleep(0.02)
    if not vals:
        raise RuntimeError('no llegan cuentas (¿puerto/agente correcto? ¿firmware imprimiendo?)')
    return statistics.mean(vals)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Calibrar cuentas/rad (o cuentas/m) de una junta')
    ap.add_argument('joint', type=int, choices=[1, 2, 3])
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--angle', type=float, default=None, help='grados girados (θ1/θ2)')
    g.add_argument('--distance-mm', type=float, default=None, help='mm recorridos (Z)')
    ap.add_argument('--serial', default='', help='puerto del firmware de PRUEBA (Fase 1)')
    ap.add_argument('--jog', action='store_true', help='mover con el motor (teclas a/d)')
    ap.add_argument('--jog-duty', type=int, default=200, help='‰ de cada pulso de jog')
    ap.add_argument('--trials', type=int, default=1, help='repeticiones a promediar')
    ap.add_argument('--yaml', default='', help='scara.yaml a modificar')
    ap.add_argument('--yes', action='store_true', help='escribir sin preguntar')
    ap.add_argument('--dry-run', action='store_true', help='no escribir el YAML')
    if argv is None:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    args = ap.parse_args(argv)

    prismatic = args.joint == 3
    if prismatic:
        amount_txt = f'{args.distance_mm or 50.0} mm'
        amount = (args.distance_mm or 50.0) / 1000.0
    else:
        deg = args.angle if args.angle is not None else 360.0
        amount_txt = f'{deg}°'
        amount = math.radians(deg)
    unit = 'cuentas/m' if prismatic else 'cuentas/rad'
    direction = 'hacia ABAJO (s3 +)' if prismatic else 'ANTIHORARIO visto desde arriba (+)'

    src = SerialSource(args.serial, args.joint, args.jog_duty) if args.serial \
        else RosSource(args.joint, args.jog_duty)
    try:
        t0 = time.time()
        while src.counts is None and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if src.counts is None:
            print('ERROR: no llegan cuentas en 5 s')
            return 1
        if not args.jog:
            src.free()
        results = []
        for t in range(args.trials):
            print(f'\n--- medición {t + 1}/{args.trials}: junta {args.joint}, {amount_txt} ---')
            wait_enter_with_jog(src, args.jog,
                                '1) Pon la junta en la MARCA DE INICIO y presiona Enter.')
            c0 = read_stable(src)
            wait_enter_with_jog(src, args.jog,
                                f'2) Muévela {amount_txt} {direction} hasta la marca final '
                                f'{"(a/d = jog) " if args.jog else "a mano "}y presiona Enter.')
            c1 = read_stable(src)
            per_unit, sign = compute(c1 - c0, amount, prismatic)
            results.append((per_unit, sign))
            print(f'   Δ = {c1 - c0:+.1f} cuentas → {per_unit:.2f} {unit}, signo {sign:+d}')
        signs = {s for _, s in results}
        if len(signs) > 1:
            print('ERROR: el signo cambió entre mediciones (¿giraste en sentidos distintos?)')
            return 2
        per_unit = statistics.mean(p for p, _ in results)
        sign = results[0][1]
        values = [p for p, _ in results]
        spread = max(values) - min(values)
        cpr, trans = 64.0, (2.0 if not prismatic else 1.0)
        print(f'\n=== RESULTADO junta {args.joint}: {per_unit:.2f} {unit} '
              f'(dispersión {spread:.2f}), joint_sign = {sign:+d}')
        if prismatic:
            print(f'   ⇒ con 64 cuentas/vuelta y reductora 50:1: paso ≈ '
                  f'{64 * 50 / per_unit * 1000:.3f} mm/vuelta')
        else:
            gear = per_unit * 2 * math.pi / (cpr * trans)
            print(f'   ⇒ con 64 cuentas/vuelta y polea 2:1: reductora efectiva ≈ {gear:.1f}:1 '
                  f'(teórico 50:1 → 1018.59 cuentas/rad)')
        if args.dry_run:
            return 0
        path = args.yaml or default_yaml_path()
        if not args.yes:
            key = 'counts_per_m' if prismatic else f'counts_per_rad[{args.joint - 1}]'
            ans = input(f'¿Escribir {key} = {per_unit:.2f} y joint_sign[{args.joint - 1}] = '
                        f'{sign} en {path}? [s/N] ')
            if ans.strip().lower() not in ('s', 'si', 'sí', 'y', 'yes'):
                print('no se escribió nada')
                return 0
        update_yaml(path, args.joint, per_unit, sign)
        print(f'escrito en {path}. Relanzar el bringup para usarlo (no hay que reflashear).')
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        src.close()


if __name__ == '__main__':
    sys.exit(main())
