"""Piezas compartidas por las herramientas de puesta en marcha."""

import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from scara_bridge import protocol as P
from std_msgs.msg import Int32MultiArray, UInt8


SIM_LABEL = 'SIMULACIÓN — no medido en el robot'


def is_simulation(node, wait: float = 1.0) -> bool:
    """Detectar si responde el simulador (nodo scara_esp32_fake) y no el robot real."""
    t_end = time.time() + wait
    while time.time() < t_end:
        if any('scara_esp32_fake' in n for n in node.get_node_names()):
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def evidence_dir(override: str = '') -> Path:
    """Carpeta donde se guardan resultados (docs/evidencias del repo si existe)."""
    if override:
        d = Path(override).expanduser()
    elif os.environ.get('SCARA_EVIDENCE_DIR'):
        d = Path(os.environ['SCARA_EVIDENCE_DIR']).expanduser()
    else:
        d = Path.home() / 'scara_st7' / 'docs' / 'evidencias'
        if not d.parent.exists():
            d = Path.cwd()
    d.mkdir(parents=True, exist_ok=True)
    return d


class RobotIO(Node):
    """
    Nodo mínimo: lee cuentas/finales y manda comandos crudos al ESP32.

    topic='/scara/cmd' habla directo con el micro (SOLO con el bridge apagado, Fases 2–3);
    topic='/scara/raw_cmd' pasa por el bridge (que reenvía con prioridad).
    """

    def __init__(self, name: str, topic: str = P.TOPIC_CMD):
        super().__init__(name)
        qos = P.QOS_CMD if topic == P.TOPIC_CMD else 10
        self.pub = self.create_publisher(Int32MultiArray, topic, qos)
        self.counts = None
        self.limits = 0
        self.samples = []       # (t, c1, c2, c3) mientras record=True
        self.record = False
        self.create_subscription(Int32MultiArray, P.TOPIC_ENC, self.on_counts, P.QOS_SENSOR)
        self.create_subscription(UInt8, P.TOPIC_LIMITS, self.on_limits, P.QOS_SENSOR)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_counts(self, msg):
        if len(msg.data) >= 3:
            self.counts = list(msg.data[:3])
            if self.record:
                self.samples.append((self.now(), *self.counts))

    def on_limits(self, msg):
        self.limits = msg.data

    def send(self, data):
        self.pub.publish(Int32MultiArray(data=[int(x) for x in data]))

    def hold(self, data, seconds: float, period: float = 0.02):
        """Repite un comando a 50 Hz (keepalive) durante 'seconds' procesando callbacks."""
        t_end = self.now() + seconds
        while self.now() < t_end and rclpy.ok():
            self.send(data)
            t_next = self.now() + period
            while self.now() < t_next:
                rclpy.spin_once(self, timeout_sec=max(0.0, t_next - self.now()))

    def wait_counts(self, timeout: float = 3.0) -> bool:
        t_end = self.now() + timeout
        while self.counts is None and self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.counts is not None

    def send_config(self, joint: int, kp=None, ki=None, kd=None, dmin=None, dmax=None):
        """Manda ganancias/duty a UNA junta (1..3), cada mensaje 3 veces (best effort)."""
        msgs = []
        if kp is not None or ki is not None:
            msgs.append([P.MODE_SET_GAINS, joint, int(kp or 0), int(ki or 0)])
        if kd is not None:
            msgs.append([P.MODE_SET_KD, joint, int(kd), 0])
        if dmin is not None or dmax is not None:
            dmax = 600 if dmax is None else dmax
            msgs.append([P.MODE_SET_DUTY, joint, int(dmin or 0), int(dmax)])
        for m in msgs:
            for _ in range(3):
                self.hold(m, 0.02)
