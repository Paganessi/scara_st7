"""fake_esp32 — simulador del ESP32 que habla EXACTAMENTE el contrato ESP32 ↔ ROS.

    ros2 run scara_tools fake_esp32
    ros2 launch scara_bringup scara.launch.py agent:=false   # en otra terminal

Para probar bridge, homing, goto, RViz y repeat_home SIN hardware (y para ensayar el
pitch). NO reemplaza las pruebas reales: no modela fricción real, backlash ni la
corriente del TB6612.

Modelo por junta: motor de primer orden (duty → velocidad, τ = 50 ms), finales de
carrera en las posiciones que implican los límites del YAML con la transmisión
supuesta, un pequeño jitter en el punto de disparo de cada final (para que la
repetibilidad no salga perfecta) y la misma lógica del firmware: modos 0–6, PID
(solo P+D aquí), enclavamiento, watchdog de 500 ms y bits de diagnóstico.
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt8

from scara_bridge import protocol as P

SIM_DT = 0.005               # 200 Hz de simulación
TAU = 0.05                   # s, constante de tiempo del motor
VMAX_COUNTS_S = 10667.0      # 64 cuentas × ~10 000 rpm / 60 a duty 1000 ‰
WATCHDOG_S = 0.5
SWITCH_JITTER = 2.0          # cuentas de dispersión del punto de disparo


class FakeEsp32(Node):

    def __init__(self):
        super().__init__('scara_esp32_fake')
        P.declare_transmission_params(self)
        self.declare_parameter('start_fraction', [0.6, 0.4, 0.3])  # dónde "enciende" el robot
        conv = P.build_conversions(self)
        lo, hi = P.joint_limits(self)
        frac = list(self.get_parameter('start_fraction').value)
        # Posición "verdadera" en cuentas respecto a un cero físico arbitrario;
        # el contador arranca en 0 donde esté el robot (como el real).
        self.sw_min = [min(conv[j].to_counts(lo[j]), conv[j].to_counts(hi[j])) for j in range(3)]
        self.sw_max = [max(conv[j].to_counts(lo[j]), conv[j].to_counts(hi[j])) for j in range(3)]
        self.true = [self.sw_min[j] + frac[j] * (self.sw_max[j] - self.sw_min[j]) for j in range(3)]
        self.offset = [-t for t in self.true]  # contador = verdadero + offset
        self.vel = [0.0, 0.0, 0.0]
        self.mode = P.MODE_STOP
        self.sp = [0, 0, 0]
        self.kp = [1000, 1000, 1000]
        self.kd = [0, 0, 0]
        self.duty_min = [0, 0, 0]
        self.duty_max = [600, 600, 600]
        self.last_cmd = self.now()
        self.trip = [self.new_trip(), self.new_trip(), self.new_trip()]

        self.pub_enc = self.create_publisher(Int32MultiArray, P.TOPIC_ENC, P.QOS_SENSOR)
        self.pub_lim = self.create_publisher(UInt8, P.TOPIC_LIMITS, P.QOS_SENSOR)
        self.create_subscription(Int32MultiArray, P.TOPIC_CMD, self.on_cmd, P.QOS_SENSOR)
        self.create_timer(SIM_DT, self.step)
        self.create_timer(0.02, self.publish)
        self.get_logger().info('ESP32 SIMULADO listo (contrato §6)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def new_trip():
        return (random.gauss(0, SWITCH_JITTER), random.gauss(0, SWITCH_JITTER))

    def counts(self, j):
        return int(round(self.true[j] + self.offset[j]))

    def limits(self):
        m = 0
        for j in range(3):
            if self.true[j] <= self.sw_min[j] + self.trip[j][0]:
                m |= P.limit_bit(j, 'MIN')
            if self.true[j] >= self.sw_max[j] + self.trip[j][1]:
                m |= P.limit_bit(j, 'MAX')
        return m

    def on_cmd(self, msg):
        d = list(msg.data) + [0, 0, 0, 0]
        mode, v1, v2, v3 = d[:4]
        self.last_cmd = self.now()
        j = v1 - 1
        if mode == P.MODE_ZERO and 0 <= j < 3:
            self.offset[j] = v2 - self.true[j]
            self.mode = P.MODE_STOP
            self.get_logger().info(f'ZERO j{v1}: verdadero={self.true[j]:.1f} '
                                   f'(final MIN en {self.sw_min[j]}+{self.trip[j][0]:.1f})')
        elif mode == P.MODE_SET_GAINS and 0 <= j < 3:
            self.kp[j] = v2
        elif mode == P.MODE_SET_KD and 0 <= j < 3:
            self.kd[j] = v2
        elif mode == P.MODE_SET_DUTY and 0 <= j < 3:
            self.duty_min[j], self.duty_max[j] = v2, min(v3, 800)
        elif mode in (P.MODE_STOP, P.MODE_POSITION, P.MODE_VELOCITY):
            self.mode = mode
            self.sp = [v1, v2, v3]

    def step(self):
        wd = self.now() - self.last_cmd > WATCHDOG_S
        mode = P.MODE_STOP if wd else self.mode
        lim = self.limits()
        for j in range(3):
            out = 0.0
            if mode == P.MODE_VELOCITY:
                out = max(-self.duty_max[j], min(self.duty_max[j], self.sp[j]))
            elif mode == P.MODE_POSITION:
                e = self.sp[j] - self.counts(j)
                if abs(e) > 4:
                    out = self.kp[j] * e / 1000.0 - self.kd[j] * self.vel[j] / 1000.0
                    out += math.copysign(self.duty_min[j], out)
                    out = max(-self.duty_max[j], min(self.duty_max[j], out))
            if (lim & P.limit_bit(j, 'MIN') and out < 0) or (lim & P.limit_bit(j, 'MAX') and out > 0):
                out = 0.0
            target_v = out / 1000.0 * VMAX_COUNTS_S
            self.vel[j] += (target_v - self.vel[j]) * SIM_DT / TAU
            if out == 0.0:
                self.vel[j] *= 0.5  # freno activo
            self.true[j] += self.vel[j] * SIM_DT
            # Tope mecánico un poco más allá de cada final
            self.true[j] = max(self.sw_min[j] - 50, min(self.sw_max[j] + 50, self.true[j]))
        # Nuevo jitter de disparo cada vez que se suelta un final
        new_lim = self.limits()
        for j in range(3):
            if (lim & ~new_lim) & (P.limit_bit(j, 'MIN') | P.limit_bit(j, 'MAX')):
                self.trip[j] = self.new_trip()
        self.wd = wd

    def publish(self):
        self.pub_enc.publish(Int32MultiArray(data=[self.counts(j) for j in range(3)]))
        flags = self.limits() | (P.BIT_WATCHDOG if getattr(self, 'wd', True) else 0)
        self.pub_lim.publish(UInt8(data=flags))


def main(args=None):
    rclpy.init(args=args)
    node = FakeEsp32()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
