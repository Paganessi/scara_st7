"""
fake_esp32 — simulador del ESP32 + planta del robot, con la LÓGICA REAL del firmware.

    ros2 launch scara_bringup scara.launch.py use_fake:=true      # forma normal
    ros2 run scara_tools fake_esp32 --ros-args --params-file <scara.yaml>

Qué es real y qué es modelo:
  - REAL: el control. Se carga libscara_ctrl_logic.so, que es firmware/scara_esp32/main/
    ctrl_logic.c compilado para el PC (paquete scara_fw_sim). PID, enclavamiento con
    finales, watchdog, protección de bloqueo, debounce y modos 0–6 son el MISMO código
    que corre en el ESP32, ejecutado a 1 kHz (en lotes de 10 pasos cada 10 ms).
  - MODELO: la planta, por junta:
        motor de primer orden:  τ·dv/dt = v_max·u_ef − v      (u_ef = duty con zona muerta)
        integrador:             dx/dt = v                      (x en cuentas de encoder)
        freno activo con duty 0, topes mecánicos más allá de cada final
        encoder: cuantización + ruido (parpadeo de ±1 cuenta cerca de cada flanco)
        finales: posición configurable (SI) + dispersión aleatoria del punto de disparo
    y el contador del encoder arranca en 0 donde "se enciende" el robot (como el real).
  - motor_present: juntas cuyo motor no existe todavía (hoy J2 y J3) → no se mueven
    aunque les llegue duty (como un driver sin motor conectado).

Habla exactamente el contrato ESP32 ↔ ROS (mismos tópicos, tipos y QoS).
NO reemplaza las pruebas reales: no modela backlash, elasticidad de la correa, la
corriente del TB6612 ni el jitter real del USB.
"""

import ctypes
import os
import random

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scara_bridge import protocol as P
from std_msgs.msg import Int32MultiArray, UInt8

STEPS_PER_TICK = 10          # 10 pasos de 1 ms por cada tick de 10 ms
TICK_S = 0.010
DT = 0.001


class CtrlEffect(ctypes.Structure):
    _fields_ = [('zero', ctypes.c_bool), ('joint', ctypes.c_int32), ('value', ctypes.c_int32)]


I32x3 = ctypes.c_int32 * 3


def load_firmware_logic():
    """Carga la lógica del firmware compilada (scara_fw_sim) y declara las firmas."""
    path = os.path.join(get_package_prefix('scara_fw_sim'), 'lib', 'libscara_ctrl_logic.so')
    lib = ctypes.CDLL(path)
    lib.ctrl_state_size.restype = ctypes.c_size_t
    lib.debounce_state_size.restype = ctypes.c_size_t
    lib.ctrl_init.argtypes = [ctypes.c_void_p, I32x3]
    lib.ctrl_command.argtypes = [ctypes.c_void_p] + [ctypes.c_int32] * 4
    lib.ctrl_command.restype = CtrlEffect
    lib.ctrl_step.argtypes = [ctypes.c_void_p, I32x3, ctypes.c_uint8, I32x3]
    lib.ctrl_step.restype = ctypes.c_uint8
    lib.debounce_init.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
    lib.debounce_update.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
    lib.debounce_update.restype = ctypes.c_uint8
    return lib, path


class JointPlant:
    """Planta de UNA junta, en cuentas de encoder (lo que ve el firmware)."""

    def __init__(self, vmax, tau, deadzone, sw_min, sw_max, jitter, stop_margin, present,
                 noise):
        self.vmax, self.tau, self.deadzone = vmax, tau, deadzone
        self.sw_min, self.sw_max, self.jitter = sw_min, sw_max, jitter
        self.stop_lo, self.stop_hi = sw_min - stop_margin, sw_max + stop_margin
        self.present, self.noise = present, noise
        self.x = 0.0          # posición verdadera (cuentas, marco físico)
        self.v = 0.0          # cuentas/s
        self.offset = 0.0     # contador = x + offset
        self.trip = self.new_trip()

    def new_trip(self):
        return (random.gauss(0.0, self.jitter), random.gauss(0.0, self.jitter))

    def step(self, duty):
        if not self.present:
            return
        u = 0.0
        if duty > self.deadzone:
            u = (duty - self.deadzone) / (1000.0 - self.deadzone)
        elif duty < -self.deadzone:
            u = (duty + self.deadzone) / (1000.0 - self.deadzone)
        self.v += (self.vmax * u - self.v) * DT / self.tau
        if duty == 0:
            self.v *= 0.8                       # freno activo (bobinado en corto)
        self.x += self.v * DT
        if self.x <= self.stop_lo or self.x >= self.stop_hi:   # tope mecánico
            self.x = min(max(self.x, self.stop_lo), self.stop_hi)
            self.v = 0.0

    def switches(self):
        """(min, max) presionados según la posición verdadera."""
        return (self.x <= self.sw_min + self.trip[0], self.x >= self.sw_max + self.trip[1])

    def counts(self):
        # Sin motor no hay encoder conectado: contador quieto (sin ruido).
        n = random.uniform(-self.noise, self.noise) if (self.noise > 0 and self.present) else 0.0
        return int(round(self.x + self.offset + n))


class FakeEsp32(Node):

    def __init__(self):
        super().__init__('scara_esp32_fake')
        P.declare_transmission_params(self)
        # --- Realidad del hardware simulado (independiente de lo que "crea" el YAML de ROS)
        self.declare_parameter('motor_present', [True, False, False])
        self.declare_parameter('plant_vmax_counts_s', [10667.0, 10667.0, 10667.0])
        self.declare_parameter('plant_tau_s', [0.03, 0.03, 0.05])
        self.declare_parameter('plant_deadzone_permille', [80.0, 80.0, 120.0])
        self.declare_parameter('encoder_noise_counts', 0.6)
        self.declare_parameter('switch_min', [-1.5708, -2.4435, 0.0])     # SI, marco físico
        self.declare_parameter('switch_max', [1.5708, 2.4435, 0.120])
        self.declare_parameter('switch_jitter_counts', 2.0)
        self.declare_parameter('hard_stop_margin_counts', 150.0)
        self.declare_parameter('start_position', [0.35, 0.5, 0.03])       # SI al encender
        self.declare_parameter('seed', 0)                                  # 0 = aleatorio

        seed = int(self.get_parameter('seed').value)
        if seed:
            random.seed(seed)
        self.lib, libpath = load_firmware_logic()
        conv = P.build_conversions(self)

        def g(name):
            return list(self.get_parameter(name).value)

        present, vmax, tau, dz = g('motor_present'), g('plant_vmax_counts_s'), \
            g('plant_tau_s'), g('plant_deadzone_permille')
        smin, smax, start = g('switch_min'), g('switch_max'), g('start_position')
        jit = float(self.get_parameter('switch_jitter_counts').value)
        margin = float(self.get_parameter('hard_stop_margin_counts').value)
        noise = float(self.get_parameter('encoder_noise_counts').value)

        self.j = []
        for k in range(3):
            # Posiciones físicas en cuentas "verdaderas", ya con el signo de la junta: el
            # firmware es la referencia (duty+ ⇒ cuentas+ ⇒ hacia MAX).
            a, b = conv[k].to_counts(smin[k]), conv[k].to_counts(smax[k])
            jp = JointPlant(float(vmax[k]), float(tau[k]), float(dz[k]), min(a, b), max(a, b),
                            jit, margin, bool(present[k]), noise)
            jp.x = float(conv[k].to_counts(start[k]))
            jp.offset = -jp.x            # el contador arranca en 0 donde esté
            self.j.append(jp)

        self.state = ctypes.create_string_buffer(self.lib.ctrl_state_size())
        self.deb = ctypes.create_string_buffer(self.lib.debounce_state_size())
        self.lib.ctrl_init(self.state, I32x3(*[jp.counts() for jp in self.j]))
        self.lib.debounce_init(self.deb, self.raw_limits())
        self.flags = 0

        self.pub_enc = self.create_publisher(Int32MultiArray, P.TOPIC_ENC, P.QOS_SENSOR)
        self.pub_lim = self.create_publisher(UInt8, P.TOPIC_LIMITS, P.QOS_SENSOR)
        self.create_subscription(Int32MultiArray, P.TOPIC_CMD, self.on_cmd, P.QOS_SENSOR)
        self.create_timer(TICK_S, self.tick)
        self.create_timer(0.02, self.publish)
        self.get_logger().info(
            f'ESP32 SIMULADO listo: lógica del firmware = {libpath}; '
            f'motores presentes = {[bool(x) for x in present]}')

    def raw_limits(self):
        m = 0
        for k, jp in enumerate(self.j):
            lo, hi = jp.switches()
            if lo:
                m |= P.limit_bit(k, 'MIN')
            if hi:
                m |= P.limit_bit(k, 'MAX')
        return m

    def on_cmd(self, msg):
        d = [int(x) for x in msg.data[:4]] + [0] * max(0, 4 - len(msg.data))
        eff = self.lib.ctrl_command(self.state, *d)
        if eff.zero:
            jp = self.j[eff.joint]
            jp.offset = eff.value - jp.x    # encoder_set()
            self.get_logger().info(f'ZERO j{eff.joint + 1} = {eff.value} '
                                   f'(posición verdadera {jp.x:.1f} cuentas)')

    def tick(self):
        out = I32x3()
        for _ in range(STEPS_PER_TICK):
            before = self.raw_limits()
            lim = self.lib.debounce_update(self.deb, before)
            pos = I32x3(*[jp.counts() for jp in self.j])
            self.flags = self.lib.ctrl_step(self.state, pos, lim, out)
            for k, jp in enumerate(self.j):
                jp.step(out[k])
            # Nuevo punto de disparo cada vez que se suelta un final
            released = before & ~self.raw_limits()
            for k, jp in enumerate(self.j):
                if released & (P.limit_bit(k, 'MIN') | P.limit_bit(k, 'MAX')):
                    jp.trip = jp.new_trip()

    def publish(self):
        self.pub_enc.publish(Int32MultiArray(data=[jp.counts() for jp in self.j]))
        self.pub_lim.publish(UInt8(data=int(self.flags)))


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
