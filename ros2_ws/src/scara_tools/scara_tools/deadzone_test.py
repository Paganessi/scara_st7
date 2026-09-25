"""
deadzone_test — encuentra el duty mínimo que vence la fricción estática de una junta.

    # con el bridge APAGADO:
    ros2 run scara_tools deadzone_test 1
    ros2 run scara_tools deadzone_test 3 --max 450 --step 10

Sube el duty en lazo abierto (VELOCITY) de a 'step' ‰ cada 'dwell' s, primero hacia +
y luego hacia −, hasta que el encoder se mueva más de 'threshold' cuentas. Ese es el
duty de arranque ("breakaway"). La compensación de zona muerta del PID (duty_min) se
pone un poco POR DEBAJO (fricción de arranque > fricción en movimiento): la sugerencia
impresa es 0.8 × el menor de los dos sentidos.

Seguridad: duty tope --max, pulsos cortos, y el ESP32 frena solo si pisa un final.
"""

import argparse
import sys

import rclpy
from scara_bridge import protocol as P
from scara_tools.common import RobotIO


def ramp(io, j, sign, start, step, top, dwell, threshold):
    duty = start
    while duty <= top and rclpy.ok():
        c0 = io.counts[j]
        v = [0, 0, 0]
        v[j] = sign * duty
        io.hold([P.MODE_VELOCITY] + v, dwell)
        moved = io.counts[j] - c0
        print(f'  duty {sign * duty:+5d} ‰ → Δ = {moved:+6d} cuentas')
        if abs(moved) > threshold:
            io.hold([P.MODE_STOP, 0, 0, 0], 0.5)
            return duty, moved
        duty += step
    io.hold([P.MODE_STOP, 0, 0, 0], 0.5)
    return None, 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='Duty mínimo de arranque por junta')
    ap.add_argument('joint', type=int, choices=[1, 2, 3])
    ap.add_argument('--start', type=int, default=40)
    ap.add_argument('--step', type=int, default=10)
    ap.add_argument('--max', type=int, default=400)
    ap.add_argument('--dwell', type=float, default=0.25)
    ap.add_argument('--threshold', type=int, default=3)
    ap.add_argument('--topic', default=P.TOPIC_CMD)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv else sys.argv)[1:])

    rclpy.init(args=argv)
    io = RobotIO('scara_deadzone_test', args.topic)
    j = args.joint - 1
    try:
        if not io.wait_counts():
            print('ERROR: no llegan /scara/enc_counts')
            return 1
        res = {}
        for sign in (1, -1):
            print(f'\nJunta {args.joint}, sentido {"+" if sign > 0 else "−"}:')
            duty, moved = ramp(io, j, sign, args.start, args.step, args.max, args.dwell,
                               args.threshold)
            res[sign] = duty
            if duty is None:
                print(f'  no se movió hasta {args.max} ‰ (¿trabada? ¿final presionado?)')
            else:
                ok = 'cuentas en el sentido del duty: OK' if moved * sign > 0 else \
                    '¡cuentas al revés! revisar signos'
                print(f'  ARRANCA con {duty} ‰ (Δ={moved:+d}); {ok}')
        found = [d for d in res.values() if d is not None]
        if found:
            print(f'\nSugerencia duty_min junta {args.joint} ≈ {int(0.8 * min(found))} ‰ '
                  f'(arranque +: {res[1]} ‰, −: {res[-1]} ‰)')
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        io.send([P.MODE_STOP, 0, 0, 0])
        io.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
