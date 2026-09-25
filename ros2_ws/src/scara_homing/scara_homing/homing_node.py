"""
homing_node — rutina de Home del SCARA.

Servicio /scara/home (std_srvs/Trigger): ARRANCA la secuencia y responde enseguida.
La secuencia corre en un timer de 50 Hz como máquina de estados, junta por junta,
en el orden de `homing_order` (por defecto Z → θ2 → θ1: primero subir la herramienta
para no barrer nada con ella).

Solo se hace homing de las juntas habilitadas (joints_enabled en scara.yaml); las otras
se saltan y quedan frenadas por el bridge. Hoy: solo θ1.

Por cada junta:
  FAST     VELOCITY rápido hacia su final de home hasta que el bit se active
  SETTLE   STOP un momento (que se detenga del todo)
  BACKOFF  retroceder hasta soltar el final y alejarse `backoff_counts`
  SETTLE   STOP
  SLOW     VELOCITY lento otra vez hacia el final → siempre lo toca a la MISMA velocidad
  SETTLE   STOP y registrar dónde quedó (para medir repetibilidad)
  ZERO     modo ZERO: cargar en el contador el valor (cuentas) de la posición del final
  VERIFY   confirmar que el ESP32 aplicó el ZERO (el tópico es best-effort: reintentar)

Por qué dos aproximaciones: la rápida encuentra el switch; la lenta lo toca siempre igual
(la inercia al frenar es pequeña y constante) → el origen queda en el mismo sitio cada vez.

Seguridad:
  - timeout por fase → STOP + ERROR (no sigue con las demás juntas)
  - si se activa el final OPUESTO de la junta → ERROR (signos mal configurados)
  - si el ESP32 reporta bloqueo (bit 6) → ERROR
  - si dejan de llegar cuentas → ERROR; si el ESP32 se reinicia, /scara/homed pasa a false
  - el enclavamiento del ESP32 corta el motor al pisar el final aunque este nodo se atrase

Publica: /scara/homed (Bool, transient_local), /scara/homing_state (String),
/scara/homing_report (String JSON con dónde quedó cada junta respecto al home anterior).
Todos los comandos al robot salen por /scara/raw_cmd → el bridge los reenvía con prioridad.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scara_bridge import protocol as P
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, String, UInt8
from std_srvs.srv import Trigger

TICK_S = 0.02            # 50 Hz
SETTLE_S = 0.3           # tiempo quieto antes de medir / cambiar de sentido
VERIFY_TOL_COUNTS = 3    # tolerancia para confirmar el ZERO
VERIFY_TIMEOUT_S = 0.5
ZERO_RETRIES = 3
STALE_COUNTS_S = 0.5     # sin cuentas más que esto durante el homing → error
ESP32_LOST_S = 1.0       # sin cuentas más que esto → el origen deja de ser confiable
REST_WAIT_S = 0.5


class HomingNode(Node):

    def __init__(self):
        super().__init__('scara_homing')
        P.declare_transmission_params(self)
        self.declare_parameter('homing_order', [3, 2, 1])
        self.declare_parameter('home_switch', ['MIN', 'MIN', 'MIN'])
        self.declare_parameter('home_offset', [-1.5708, -2.4435, 0.0])
        self.declare_parameter('homing_fast', [350, 350, 500])
        self.declare_parameter('homing_slow', [250, 250, 350])
        self.declare_parameter('backoff_counts', [180, 180, 3200])
        self.declare_parameter('homing_timeout_s', [20.0, 20.0, 90.0])
        self.declare_parameter('go_to_rest', True)
        self.declare_parameter('rest_pose', [0.0, 0.0, 0.0])

        self.conv = P.build_conversions(self)
        self.names = list(self.get_parameter('joint_names').value)

        self.counts = None
        self.counts_time = None
        self.limits = 0
        self.frame_valid = False     # ¿las cuentas actuales están referidas a un home previo?
        self.run_number = 0
        self.state = 'IDLE'
        self.detail = ''

        self.pub_raw = self.create_publisher(Int32MultiArray, P.TOPIC_RAW_CMD, 10)
        self.pub_homed = self.create_publisher(Bool, P.TOPIC_HOMED, P.QOS_LATCHED)
        self.pub_state = self.create_publisher(String, P.TOPIC_HOMING_STATE, 10)
        self.pub_report = self.create_publisher(String, P.TOPIC_HOMING_REPORT, 10)
        self.pub_goal = self.create_publisher(JointState, P.TOPIC_JOINT_GOAL, 10)
        self.create_subscription(Int32MultiArray, P.TOPIC_ENC, self.on_counts, P.QOS_SENSOR)
        self.create_subscription(UInt8, P.TOPIC_LIMITS, self.on_limits, P.QOS_SENSOR)
        self.create_service(Trigger, '/scara/home', self.on_home)
        self.create_service(Trigger, '/scara/home_abort', self.on_abort)
        self.create_timer(TICK_S, self.on_tick)
        self.create_timer(0.5, self.publish_state)

        self.set_homed(False)
        self.get_logger().info('homing listo: ros2 service call /scara/home std_srvs/srv/Trigger')

    # ------------------------------------------------------------ utilidades
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_homed(self, value: bool):
        self.pub_homed.publish(Bool(data=value))

    def raw(self, data):
        self.pub_raw.publish(Int32MultiArray(data=[int(x) for x in data]))

    def velocity(self, j: int, duty: int):
        v = [0, 0, 0]
        v[j] = int(duty)
        self.raw([P.MODE_VELOCITY] + v)

    def stop(self):
        self.raw([P.MODE_STOP, 0, 0, 0])

    def enter(self, state: str, detail: str = ''):
        self.state = state
        self.detail = detail
        self.t_state = self.now_s()
        self.publish_state()

    def elapsed(self) -> float:
        return self.now_s() - self.t_state

    def publish_state(self):
        txt = self.state
        if self.state not in ('IDLE', 'DONE') and self.state != 'ERROR':
            txt = f'{self.state} {self.names[self.j]}' if hasattr(self, 'j') else self.state
        if self.detail:
            txt += f': {self.detail}'
        self.pub_state.publish(String(data=txt))

    def fail(self, why: str):
        self.stop()
        self.get_logger().error(f'HOMING ABORTADO: {why}')
        self.enter('ERROR', why)

    # ------------------------------------------------------------ entradas
    def on_counts(self, msg: Int32MultiArray):
        if len(msg.data) < 3:
            return
        now = self.now_s()
        if self.counts_time is not None and now - self.counts_time > ESP32_LOST_S:
            if self.frame_valid:
                self.get_logger().warn('se perdió el ESP32 un momento: el origen ya no es '
                                       'confiable → /scara/homed = false, repetir homing')
            self.frame_valid = False
            self.set_homed(False)
        self.counts = list(msg.data[:3])
        self.counts_time = now

    def on_limits(self, msg: UInt8):
        self.limits = msg.data

    def on_home(self, request, response):
        if self.state not in ('IDLE', 'DONE', 'ERROR'):
            response.success = False
            response.message = f'homing ya en curso ({self.state})'
            return response
        if self.counts is None or self.now_s() - self.counts_time > STALE_COUNTS_S:
            response.success = False
            response.message = 'no llegan cuentas del ESP32 (¿agente/micro conectados?)'
            return response
        enabled = P.joints_enabled(self)
        self.order = [int(x) - 1 for x in self.get_parameter('homing_order').value
                      if enabled[int(x) - 1]]
        if not self.order:
            response.success = False
            response.message = 'no hay juntas habilitadas (joints_enabled) para hacer homing'
            return response
        self.switch = [s.upper() for s in self.get_parameter('home_switch').value]
        self.fast = list(self.get_parameter('homing_fast').value)
        self.slow = list(self.get_parameter('homing_slow').value)
        self.backoff = list(self.get_parameter('backoff_counts').value)
        self.timeout = list(self.get_parameter('homing_timeout_s').value)
        offs = list(self.get_parameter('home_offset').value)
        self.home_counts = [self.conv[j].to_counts(offs[j]) for j in range(3)]
        self.hit_counts = [None, None, None]
        self.was_valid = self.frame_valid
        self.run_number += 1
        self.set_homed(False)
        self.k = 0
        self.start_joint()
        response.success = True
        names = [self.names[j] for j in self.order]
        response.message = f'homing #{self.run_number} iniciado: {names}'
        self.get_logger().info(response.message)
        return response

    def on_abort(self, request, response):
        if self.state in ('IDLE', 'DONE', 'ERROR'):
            response.success = False
            response.message = 'no hay homing en curso'
        else:
            self.fail('abortado por el usuario')
            response.success = True
            response.message = 'homing abortado (STOP)'
        return response

    # ------------------------------------------------------------ máquina de estados
    def start_joint(self):
        self.j = self.order[self.k]
        j = self.j
        # Sentido hacia el final de home según la convención de signos del micro:
        # duty < 0 va hacia MIN, duty > 0 va hacia MAX.
        self.dir = -1 if self.switch[j] == 'MIN' else 1
        self.home_bit = P.limit_bit(j, self.switch[j])
        self.other_bit = P.limit_bit(j, 'MAX' if self.switch[j] == 'MIN' else 'MIN')
        if self.limits & self.home_bit:
            self.enter('SETTLE1', 'ya estaba sobre el final')
        else:
            self.enter('FAST')

    def pressed(self) -> bool:
        return bool(self.limits & self.home_bit)

    def on_tick(self):
        s = self.state
        if s in ('IDLE', 'DONE', 'ERROR'):
            return
        if self.counts_time is None or self.now_s() - self.counts_time > STALE_COUNTS_S:
            self.fail('dejaron de llegar cuentas del ESP32')
            return
        if self.limits & P.BIT_STALL:
            self.fail(f'el ESP32 detectó bloqueo en {self.names[self.j]}')
            return
        j = self.j
        if s in ('FAST', 'BACKOFF', 'SLOW'):
            if self.limits & self.other_bit:
                self.fail(f'se activó el final opuesto de {self.names[j]}: revisar '
                          f'home_switch / MOTOR_INVERT')
                return
            if self.elapsed() > float(self.timeout[j]):
                self.fail(f'timeout en {s} de {self.names[j]} ({self.timeout[j]} s)')
                return

        if s == 'FAST':
            if self.pressed():
                self.stop()
                self.enter('SETTLE1')
            else:
                self.velocity(j, self.dir * abs(self.fast[j]))
        elif s == 'SETTLE1':
            self.stop()
            if self.elapsed() >= SETTLE_S:
                self.backoff_ref = self.counts[j]
                self.enter('BACKOFF')
        elif s == 'BACKOFF':
            moved = abs(self.counts[j] - self.backoff_ref)
            if not self.pressed() and moved >= int(self.backoff[j]):
                self.stop()
                self.enter('SETTLE2')
            else:
                self.velocity(j, -self.dir * abs(self.slow[j]))
        elif s == 'SETTLE2':
            self.stop()
            if self.elapsed() >= SETTLE_S:
                self.enter('SLOW')
        elif s == 'SLOW':
            if self.pressed():
                self.stop()
                self.enter('SETTLE3')
            else:
                self.velocity(j, self.dir * abs(self.slow[j]))
        elif s == 'SETTLE3':
            self.stop()
            if self.elapsed() >= SETTLE_S:
                self.hit_counts[j] = self.counts[j]
                self.zero_tries = 0
                self.send_zero()
        elif s == 'VERIFY':
            self.stop()
            if abs(self.counts[j] - self.home_counts[j]) <= VERIFY_TOL_COUNTS:
                self.get_logger().info(
                    f'{self.names[j]}: origen fijado ({self.home_counts[j]} cuentas; '
                    f'antes del ZERO marcaba {self.hit_counts[j]})')
                self.k += 1
                if self.k < len(self.order):
                    self.start_joint()
                else:
                    self.finish()
            elif self.elapsed() > VERIFY_TIMEOUT_S:
                if self.zero_tries < ZERO_RETRIES:
                    self.send_zero()
                else:
                    self.fail(f'el ESP32 no aplicó el ZERO de {self.names[j]}')
        elif s == 'REST_WAIT':
            self.stop()
            if self.elapsed() >= REST_WAIT_S:
                pose = [float(x) for x in self.get_parameter('rest_pose').value]
                self.pub_goal.publish(JointState(name=self.names, position=pose))
                self.get_logger().info(f'yendo a pose de reposo {pose}')
                self.enter('DONE')

    def send_zero(self):
        j = self.j
        self.zero_tries += 1
        self.raw([P.MODE_ZERO, j + 1, self.home_counts[j], 0])
        self.enter('VERIFY')

    def finish(self):
        self.frame_valid = True
        self.set_homed(True)
        delta = [None if h is None else int(h - c)
                 for h, c in zip(self.hit_counts, self.home_counts)]
        report = {
            'run': self.run_number,
            'previous_frame_valid': self.was_valid,
            'hit_counts': self.hit_counts,
            'home_counts': self.home_counts,
            'delta_counts': delta,
            'delta_si': [None if d is None else self.conv[j].to_si(d)
                         for j, d in enumerate(delta)],
        }
        self.pub_report.publish(String(data=json.dumps(report)))
        self.get_logger().info(f'HOMING COMPLETO. reporte: {json.dumps(report)}')
        if self.get_parameter('go_to_rest').value:
            self.enter('REST_WAIT')
        else:
            self.stop()
            self.enter('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = HomingNode()
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
