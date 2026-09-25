"""
bridge_node — traductor y multiplexor entre ROS 2 (unidades SI) y el ESP32 (cuentas).

Es el ÚNICO nodo que publica en /scara/cmd. Así nunca hay dos nodos
peleando por el robot. Prioridades en cada tick del keepalive:
  1. /scara/raw_cmd llegado hace < raw_cmd_timeout (lo usa el homing) → se reenvía tal cual
  2. meta articular activa (/scara/joint_goal) → POSITION con rampa de velocidad
  3. nada → STOP

Además:
  - publica /joint_states (rad, rad, m) a partir de /scara/enc_counts
  - rechaza metas fuera de límites o si el robot no ha hecho homing (require_homed)
  - reenvía cada ~2 s las ganancias PID y límites de duty del YAML al ESP32 (modos 4–6):
    si el micro se reinicia, recupera su configuración solo
  - servicio /scara/stop (std_srvs/Trigger): frena todo y descarta la meta
  - juntas deshabilitadas (joints_enabled = false, motor aún no instalado): siempre frenadas.
    Al ESP32 se le manda duty_max = 0 para esa junta (el firmware la deja en freno en
    cualquier modo), su setpoint es "donde está" y su valor en las metas se ignora.
    Activar una junta = cambiar joints_enabled en scara.yaml y relanzar.

Por qué la rampa (max_velocity) vive aquí y no en el micro: generar la trayectoria no
necesita tiempo real duro (un setpoint cada 20 ms basta), así que según el principio del
curso va en ROS. El micro solo sigue el setpoint con su PID de 1 kHz.
"""


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scara_bridge import protocol as P
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, UInt8
from std_srvs.srv import Trigger


class BridgeNode(Node):

    def __init__(self):
        super().__init__('scara_bridge')
        P.declare_transmission_params(self)
        self.declare_parameter('keepalive_hz', 50.0)
        self.declare_parameter('raw_cmd_timeout', 0.2)
        self.declare_parameter('require_homed', True)
        self.declare_parameter('max_velocity', [0.5, 0.5, 0.003])  # rad/s, rad/s, m/s
        self.declare_parameter('goal_tolerance', [0.01, 0.01, 0.0005])
        self.declare_parameter('pid_kp_milli', [1000, 1000, 1000])
        self.declare_parameter('pid_ki_milli', [0, 0, 0])
        self.declare_parameter('pid_kd_milli', [0, 0, 0])
        self.declare_parameter('duty_min', [0, 0, 0])
        self.declare_parameter('duty_max', [600, 600, 600])
        self.declare_parameter('send_config', True)
        self.declare_parameter('config_period', 0.2)  # s entre mensajes de configuración

        self.conv = P.build_conversions(self)
        self.lower, self.upper = P.joint_limits(self)
        self.names = list(self.get_parameter('joint_names').value)
        self.enabled = P.joints_enabled(self)

        # Estado
        self.counts = None            # últimas cuentas del ESP32
        self.counts_time = None
        self.limits = 0
        self.homed = False
        self.goal_counts = None       # meta final (cuentas)
        self.sp_counts = None         # setpoint con rampa (float, cuentas)
        self.raw_cmd = None
        self.raw_time = None
        self.config_queue = []

        # Tópicos
        self.pub_cmd = self.create_publisher(Int32MultiArray, P.TOPIC_CMD, P.QOS_CMD)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Int32MultiArray, P.TOPIC_ENC, self.on_counts, P.QOS_SENSOR)
        self.create_subscription(UInt8, P.TOPIC_LIMITS, self.on_limits, P.QOS_SENSOR)
        self.create_subscription(JointState, P.TOPIC_JOINT_GOAL, self.on_goal, 10)
        self.create_subscription(Int32MultiArray, P.TOPIC_RAW_CMD, self.on_raw, 10)
        self.create_subscription(Bool, P.TOPIC_HOMED, self.on_homed, P.QOS_LATCHED)
        self.create_service(Trigger, '/scara/stop', self.on_stop)

        self.dt = 1.0 / float(self.get_parameter('keepalive_hz').value)
        self.create_timer(self.dt, self.on_keepalive)
        if self.get_parameter('send_config').value:
            self.create_timer(float(self.get_parameter('config_period').value), self.on_config)

        k = [f'{c.counts_per_unit:.1f}' for c in self.conv]
        self.get_logger().info(
            f'bridge listo. juntas habilitadas = {self.enabled}; '
            f'cuentas/unidad (rad, rad, m) = {k}; '
            f'require_homed={self.get_parameter("require_homed").value}')

    # ------------------------------------------------------------------ entradas
    def on_counts(self, msg: Int32MultiArray):
        if len(msg.data) < 3:
            return
        self.counts = list(msg.data[:3])
        self.counts_time = self.get_clock().now()
        js = JointState()
        js.header.stamp = self.counts_time.to_msg()
        js.name = self.names
        js.position = [self.conv[j].to_si(self.counts[j]) for j in range(3)]
        self.pub_js.publish(js)

    def on_limits(self, msg: UInt8):
        new = msg.data & ~self.limits
        if new & P.BIT_STALL:
            self.get_logger().error('ESP32: BLOQUEO detectado (stall): junta cortada. '
                                    'Revisar mecánica; se limpia con STOP (/scara/stop).')
        for i, name in enumerate(P.LIMIT_NAMES):
            if new & (1 << i):
                self.get_logger().info(f'final {name} presionado')
        self.limits = msg.data

    def on_homed(self, msg: Bool):
        self.homed = bool(msg.data)

    def on_raw(self, msg: Int32MultiArray):
        if len(msg.data) < 1:
            return
        data = self.sanitize(list(msg.data[:4]) + [0] * (4 - min(4, len(msg.data))))
        self.raw_cmd = data
        self.raw_time = self.get_clock().now()
        # Un comando crudo cancela la meta propia: cuando el crudo expire, STOP.
        self.goal_counts = None
        self.sp_counts = None
        # Se reenvía de inmediato (el ZERO no espera al siguiente tick).
        self.send(data)

    def on_goal(self, msg: JointState):
        pos = self.goal_from_msg(msg)
        if pos is None:
            return
        if self.get_parameter('require_homed').value and not self.homed:
            self.get_logger().warn('meta rechazada: el robot no ha hecho homing '
                                   '(ros2 service call /scara/home std_srvs/srv/Trigger)')
            return
        if self.counts is None:
            self.get_logger().warn('meta rechazada: aún no llegan cuentas del ESP32')
            return
        ignored = []
        for j in range(3):
            if not self.enabled[j]:
                # Junta deshabilitada: se queda donde está, se ignora lo pedido.
                ignored.append(self.names[j])
                pos[j] = self.conv[j].to_si(self.counts[j])
                continue
            if not (self.lower[j] - 1e-9 <= pos[j] <= self.upper[j] + 1e-9):
                self.get_logger().warn(
                    f'meta rechazada: {self.names[j]}={pos[j]:.4f} fuera de '
                    f'[{self.lower[j]:.4f}, {self.upper[j]:.4f}]')
                return
        if self.sp_counts is None:
            self.sp_counts = [float(c) for c in self.counts]  # la rampa arranca donde está
        self.goal_counts = [self.conv[j].to_counts(pos[j]) for j in range(3)]
        self.raw_cmd = None
        extra = f' (deshabilitadas, se ignoran: {ignored})' if ignored else ''
        self.get_logger().info(f'meta aceptada: {[round(p, 4) for p in pos]} → '
                               f'cuentas {self.goal_counts}{extra}')

    def goal_from_msg(self, msg: JointState):
        """Acepta la meta por nombres (en cualquier orden) o, sin nombres, en orden j1..j3."""
        if msg.name:
            if not all(n in msg.name for n in self.names) or len(msg.position) != len(msg.name):
                self.get_logger().warn(f'meta rechazada: se esperan las juntas {self.names}')
                return None
            return [float(msg.position[msg.name.index(n)]) for n in self.names]
        if len(msg.position) != 3:
            self.get_logger().warn('meta rechazada: se esperan 3 posiciones')
            return None
        return [float(p) for p in msg.position]

    def on_stop(self, request, response):
        self.goal_counts = None
        self.sp_counts = None
        self.raw_cmd = None
        self.send([P.MODE_STOP, 0, 0, 0])
        response.success = True
        response.message = 'STOP enviado'
        return response

    # ------------------------------------------------------------------ salidas
    def send(self, data):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.pub_cmd.publish(msg)

    def sanitize(self, data):
        """Un comando crudo nunca mueve una junta deshabilitada."""
        mode = data[0]
        for j in range(3):
            if self.enabled[j]:
                continue
            if mode == P.MODE_VELOCITY:
                data[1 + j] = 0
            elif mode == P.MODE_POSITION and self.counts is not None:
                data[1 + j] = self.counts[j]
        return data

    def raw_is_fresh(self) -> bool:
        if self.raw_cmd is None or self.raw_time is None:
            return False
        age = (self.get_clock().now() - self.raw_time).nanoseconds * 1e-9
        return age < float(self.get_parameter('raw_cmd_timeout').value)

    def on_keepalive(self):
        # 1. Comando crudo vigente (homing, pruebas)
        if self.raw_is_fresh():
            data = self.raw_cmd
            if data[0] == P.MODE_ZERO or data[0] in P.CONFIG_MODES:
                # No repetir un ZERO (recargaría el contador con el motor aún asentándose)
                # ni la configuración: el keepalive de ese intervalo es un STOP.
                data = [P.MODE_STOP, 0, 0, 0]
            self.send(data)
            return
        self.raw_cmd = None

        # 2. Meta propia con rampa
        if self.goal_counts is not None and self.sp_counts is not None:
            vmax = list(self.get_parameter('max_velocity').value)
            for j in range(3):
                step = abs(vmax[j]) * self.conv[j].counts_per_unit * self.dt
                err = self.goal_counts[j] - self.sp_counts[j]
                self.sp_counts[j] += max(-step, min(step, err))
            self.send([P.MODE_POSITION] + [int(round(s)) for s in self.sp_counts])
            return

        # 3. Nada que hacer: freno (y keepalive)
        self.send([P.MODE_STOP, 0, 0, 0])

    def on_config(self):
        """
        Mandar UN mensaje de configuración por periodo.

        El micro guarda 1 mensaje por suscripción; en ráfaga se perderían.
        Ciclo completo = 9 mensajes.
        """
        if not self.config_queue:
            kp = self.get_parameter('pid_kp_milli').value
            ki = self.get_parameter('pid_ki_milli').value
            kd = self.get_parameter('pid_kd_milli').value
            dmin = self.get_parameter('duty_min').value
            dmax = self.get_parameter('duty_max').value
            for j in range(3):
                # Junta deshabilitada: duty_max = 0 → el firmware la mantiene en freno.
                duty = [dmin[j], dmax[j]] if self.enabled[j] else [0, 0]
                self.config_queue += [
                    [P.MODE_SET_DUTY, j + 1, duty[0], duty[1]],
                    [P.MODE_SET_GAINS, j + 1, kp[j], ki[j]],
                    [P.MODE_SET_KD, j + 1, kd[j], 0],
                ]
        self.send(self.config_queue.pop(0))


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Al cerrar: un STOP explícito (el watchdog del micro también lo haría en 0.5 s).
        try:
            node.send([P.MODE_STOP, 0, 0, 0])
        except Exception:  # noqa: BLE001 — el contexto puede estar cerrándose
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
