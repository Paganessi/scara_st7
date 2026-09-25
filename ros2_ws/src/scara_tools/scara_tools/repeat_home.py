"""repeat_home — evidencia de repetibilidad del homing (Fase 5, rúbrica "Rutina Home").

    ros2 run scara_tools repeat_home            # 5 corridas medidas
    ros2 run scara_tools repeat_home -n 10

Idea: cada homing, JUSTO antes del ZERO, registra dónde quedó la junta sobre su final
medido en el sistema de referencia del homing ANTERIOR ("delta"). Si el homing es
repetible, el final se toca siempre en el mismo punto y delta ≈ 0. La dispersión de
delta (desviación estándar y rango) ES la repetibilidad.

La primera corrida tras encender no tiene referencia previa (el contador arrancó en 0
donde estuviera el robot), así que se descarta y se hacen n+1 corridas. Entre corridas
el robot va a la pose de reposo (go_to_rest), así cada homing arranca lejos del final.

Resultado: tabla en pantalla + Markdown y CSV en docs/evidencias/ (para Notion).
"""

import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scara_bridge import protocol as P
from scara_tools.common import evidence_dir


class RepeatHome(Node):

    def __init__(self):
        super().__init__('scara_repeat_home')
        self.cli = self.create_client(Trigger, '/scara/home')
        self.state = ''
        self.reports = []
        self.js = None
        self.js_time = 0.0
        self.create_subscription(String, P.TOPIC_HOMING_STATE, self.on_state, 10)
        self.create_subscription(String, P.TOPIC_HOMING_REPORT, self.on_report, 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)

    def on_state(self, msg):
        self.state = msg.data

    def on_report(self, msg):
        self.reports.append(json.loads(msg.data))

    def on_js(self, msg):
        self.js = list(msg.position)
        self.js_time = time.time()

    def spin_for(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_once(self, timeout):
        n_before = len(self.reports)
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().success:
            msg = fut.result().message if fut.done() else 'sin respuesta'
            return None, f'no arrancó: {msg}'
        self.state = 'RUNNING'
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state.startswith('ERROR'):
                return None, self.state
            if self.state.startswith(('DONE', 'REST_WAIT')) and len(self.reports) > n_before:
                return self.reports[-1], 'ok'
        return None, 'timeout esperando el homing'

    def wait_still(self, timeout=40.0, window=0.6):
        """Espera a que el robot quede quieto (llegó a reposo) mirando /joint_states."""
        t0 = time.time()
        last = None
        t_still = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.1)
            if self.js is None:
                continue
            if last is not None and max(abs(a - b) for a, b in zip(self.js, last)) > 1e-4:
                t_still = time.time()
            last = list(self.js)
            if time.time() - t_still > window and time.time() - t0 > 1.0:
                return True
        return False


def fmt_unit(j, v):
    return f'{math.degrees(v):.3f}°' if j < 2 else f'{v * 1000:.3f} mm'


def main(argv=None):
    ap = argparse.ArgumentParser(description='Repetibilidad del homing')
    ap.add_argument('-n', type=int, default=5, help='corridas medidas (se hace 1 extra)')
    ap.add_argument('--timeout', type=float, default=300.0, help='s máx. por homing')
    ap.add_argument('--out', default='')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv else sys.argv)[1:])

    rclpy.init(args=argv)
    node = RepeatHome()
    names = ['joint1 (θ1)', 'joint2 (θ2)', 'joint3 (Z)']
    try:
        if not node.cli.wait_for_service(timeout_sec=5.0):
            print('ERROR: no existe /scara/home (¿está lanzado scara_bringup?)')
            return 1
        measured = []
        run = 0
        while len(measured) < args.n:
            run += 1
            print(f'--- homing #{run} ---')
            rep, why = node.run_once(args.timeout)
            if rep is None:
                print(f'FALLÓ: {why}')
                return 2
            if rep.get('previous_frame_valid'):
                measured.append(rep)
                print(f"  delta (cuentas) = {rep['delta_counts']}")
            else:
                print('  (sin referencia previa: corrida de calentamiento, no se mide)')
            node.wait_still()

        rows = []
        for j in range(3):
            if any(r['delta_counts'][j] is None for r in measured):
                continue   # junta deshabilitada: no participó en el homing
            d = [r['delta_counts'][j] for r in measured]
            dsi = [r['delta_si'][j] for r in measured]
            sd = statistics.stdev(d) if len(d) > 1 else 0.0
            sd_si = statistics.stdev(dsi) if len(dsi) > 1 else 0.0
            rows.append((j, names[j], d, statistics.mean(d), sd, max(d) - min(d),
                         statistics.mean(dsi), sd_si, max(dsi) - min(dsi)))

        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        lines = [f'# Repetibilidad del homing — {stamp}', '',
                 f'{len(measured)} corridas medidas (+ calentamiento). delta = posición al tocar '
                 'el final (tras la 2.ª aproximación lenta) en el origen del homing anterior.', '',
                 '| Junta | deltas (cuentas) | media | desv. est. | rango | desv. est. (físico) '
                 '| rango (físico) |', '|---|---|---|---|---|---|---|']
        for (j, n, d, m, sd, rg, m_si, sd_si, rg_si) in rows:
            lines.append(f'| {n} | {d} | {m:.1f} | {sd:.2f} | {rg} | {fmt_unit(j, sd_si)} '
                         f'| {fmt_unit(j, rg_si)} |')
        text = '\n'.join(lines) + '\n'
        print('\n' + text)
        out = evidence_dir(args.out)
        base = f'repetibilidad_home_{time.strftime("%Y%m%d_%H%M%S")}'
        (out / f'{base}.md').write_text(text)
        with open(out / f'{base}.csv', 'w') as f:
            f.write('corrida,d1_counts,d2_counts,d3_counts\n')
            for i, r in enumerate(measured, 1):
                f.write(f"{i},{','.join('' if x is None else str(x) for x in r['delta_counts'])}\n")
        print(f'guardado en {out / base}.md / .csv')
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
