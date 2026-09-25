"""
step_test — respuesta al escalón del PID de UNA junta (Fase 3).

    # con el bridge APAGADO (habla directo con /scara/cmd):
    ros2 run scara_tools step_test 1 300 --kp 1000
    ros2 run scara_tools step_test 3 3200 --kp 400 --ki 50 --kd 5 --tol 10

Qué hace:
  1. (opcional) manda al ESP32 las ganancias/duty a probar (modos 4–6, sin reflashear)
  2. sostiene POSITION en la posición actual 0.5 s
  3. pide POSITION = actual + escalón en la junta elegida y graba las cuentas
  4. STOP, calcula métricas y guarda un CSV en docs/evidencias/

Métricas: sobreimpulso (%), tiempo de establecimiento dentro de ±tol, error final y
número de cruces por la meta (oscilación). Criterio de la Fase 3: llega sin oscilar
y se queda dentro de ±tol cuentas.

OJO: es un escalón RELATIVO (no necesita homing). El enclavamiento con finales y la
protección de bloqueo del ESP32 siguen activos.
"""

import argparse
import sys
import time

import rclpy
from scara_bridge import protocol as P
from scara_tools.common import evidence_dir, RobotIO


def analyze(samples, j, base, target, tol):
    t0 = samples[0][0]
    ts = [s[0] - t0 for s in samples]
    ys = [s[1 + j] for s in samples]
    step = target - base
    sgn = 1 if step >= 0 else -1
    err = [target - y for y in ys]
    peak = max(sgn * (y - base) for y in ys)
    overshoot = max(0.0, (peak - abs(step)) / abs(step) * 100.0) if step else 0.0
    t_settle = 0.0
    for t, e in zip(ts, err):
        if abs(e) > tol:
            t_settle = t
    tail = [e for t, e in zip(ts, err) if t >= ts[-1] - 0.3] or err[-1:]
    final_err = sum(tail) / len(tail)
    # Oscilación = cruces de la meta FUERA de la banda ±tol (un sobrepaso de 1–2 cuentas
    # dentro de la banda no es oscilar). Mismo criterio que la prueba en C del firmware.
    crossings = 0
    prev = None
    for e in err:
        s = 1 if e > tol else (-1 if e < -tol else 0)
        if s != 0 and prev is not None and s != prev:
            crossings += 1
        if s != 0:
            prev = s
    return {
        'overshoot_pct': overshoot, 't_settle_s': t_settle, 'final_err_counts': final_err,
        'max_abs_err_tail': max(abs(e) for e in tail), 'crossings': crossings,
        'within_tol_at_end': all(abs(e) <= tol for e in tail),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description='Escalón de posición en una junta')
    ap.add_argument('joint', type=int, choices=[1, 2, 3])
    ap.add_argument('step', type=int, help='escalón en cuentas (con signo)')
    ap.add_argument('--hold', type=float, default=3.0, help='s grabando tras el escalón')
    ap.add_argument('--kp', type=int)
    ap.add_argument('--ki', type=int)
    ap.add_argument('--kd', type=int)
    ap.add_argument('--duty-min', type=int)
    ap.add_argument('--duty-max', type=int)
    ap.add_argument('--tol', type=int, default=10, help='banda ±cuentas (N de la Fase 3)')
    ap.add_argument('--max-step', type=int, default=4000, help='tope de seguridad')
    ap.add_argument('--topic', default=P.TOPIC_CMD)
    ap.add_argument('--out', default='')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv else sys.argv)[1:])

    if abs(args.step) > args.max_step:
        print(f'escalón {args.step} > --max-step {args.max_step}: abortado')
        return 1
    rclpy.init(args=argv)
    io = RobotIO('scara_step_test', args.topic)
    j = args.joint - 1
    try:
        if not io.wait_counts():
            print('ERROR: no llegan /scara/enc_counts (¿agente y ESP32?)')
            return 1
        if any(v is not None for v in (args.kp, args.ki, args.kd)):
            kp_ki_given = args.kp is not None or args.ki is not None
            io.send_config(args.joint, kp=args.kp if kp_ki_given else None,
                           ki=args.ki if kp_ki_given else None, kd=args.kd)
        if args.duty_min is not None or args.duty_max is not None:
            io.send_config(args.joint, dmin=args.duty_min, dmax=args.duty_max)

        base_all = list(io.counts)
        io.hold([P.MODE_POSITION] + base_all, 0.5)
        base = io.counts[j]
        target_all = list(base_all)
        target_all[j] = base + args.step
        io.samples = []
        io.record = True
        io.hold([P.MODE_POSITION] + target_all, args.hold)
        io.record = False
        io.hold([P.MODE_STOP, 0, 0, 0], 0.2)

        if len(io.samples) < 5:
            print('ERROR: muy pocas muestras grabadas')
            return 1
        m = analyze(io.samples, j, base, base + args.step, args.tol)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = evidence_dir(args.out) / f'step_j{args.joint}_{stamp}.csv'
        with open(path, 'w') as f:
            f.write(f'# junta={args.joint} escalon={args.step} kp={args.kp} ki={args.ki} '
                    f'kd={args.kd} tol={args.tol}\n')
            f.write('t_s,counts,target\n')
            t0 = io.samples[0][0]
            for s in io.samples:
                f.write(f'{s[0] - t0:.4f},{s[1 + j]},{base + args.step}\n')
        ok = m['within_tol_at_end'] and m['crossings'] == 0
        print(f'\n=== Escalón junta {args.joint}: {args.step} cuentas ===')
        print(f"  sobreimpulso      : {m['overshoot_pct']:.1f} %")
        print(f"  t establecimiento : {m['t_settle_s']:.2f} s (banda ±{args.tol})")
        print(f"  error final       : {m['final_err_counts']:.1f} cuentas "
              f"(máx |e| últimos 0.3 s = {m['max_abs_err_tail']})")
        print(f"  cruces por la meta: {m['crossings']}")
        verdict = 'CUMPLE' if ok else 'NO CUMPLE'
        print(f'  RESULTADO         : {verdict} (sin oscilar y dentro de ±tol)')
        print(f'  CSV               : {path}')
        return 0 if ok else 2
    except KeyboardInterrupt:
        return 130
    finally:
        io.send([P.MODE_STOP, 0, 0, 0])
        io.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
