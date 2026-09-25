"""Contrato ESP32 <-> ROS 2 y conversiones cuentas <-> unidades SI.

Este módulo es el ÚNICO lugar de ROS que sabe cómo pasar de cuentas de encoder a
rad / m. Lo usan el bridge, el homing y las herramientas, así la calibración vive en
un solo sitio: los parámetros del YAML (scara_bringup/config/scara.yaml).
"""

import math
from dataclasses import dataclass

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

# ---------------- Tópicos ----------------
TOPIC_ENC = '/scara/enc_counts'
TOPIC_LIMITS = '/scara/limits'
TOPIC_CMD = '/scara/cmd'
TOPIC_RAW_CMD = '/scara/raw_cmd'
TOPIC_JOINT_GOAL = '/scara/joint_goal'
TOPIC_HOMED = '/scara/homed'
TOPIC_HOMING_STATE = '/scara/homing_state'
TOPIC_HOMING_REPORT = '/scara/homing_report'

# ---------------- Modos de /scara/cmd ----------------
MODE_STOP = 0
MODE_POSITION = 1
MODE_VELOCITY = 2
MODE_ZERO = 3
# Extensión: configuración en caliente, no mueve nada.
MODE_SET_GAINS = 4   # [4, j, kp_milli, ki_milli]
MODE_SET_KD = 5      # [5, j, kd_milli, 0]
MODE_SET_DUTY = 6    # [6, j, duty_min, duty_max]
CONFIG_MODES = (MODE_SET_GAINS, MODE_SET_KD, MODE_SET_DUTY)

# ---------------- Bits de /scara/limits ----------------
BIT_STALL = 1 << 6      # el ESP32 cortó alguna junta por bloqueo
BIT_WATCHDOG = 1 << 7   # el ESP32 está en STOP por falta de comandos
LIMIT_NAMES = ['FC1_MIN', 'FC1_MAX', 'FC2_MIN', 'FC2_MAX', 'FC3_MIN', 'FC3_MAX']


def limit_bit(joint_index: int, which: str) -> int:
    """Máscara del final MIN/MAX de la junta 0..2."""
    return 1 << (2 * joint_index + (1 if which.upper() == 'MAX' else 0))


# ---------------- QoS ----------------
# El ESP32 publica y se suscribe en BEST_EFFORT (ver uros.c): del lado ROS hay que
# suscribirse igual o no hay "match". Publicar RELIABLE hacia un sub BEST_EFFORT sí es válido.
QOS_SENSOR = qos_profile_sensor_data
QOS_CMD = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
# /scara/homed: el último valor le llega a quien se conecte tarde.
QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)

JOINT_TYPES = ('revolute', 'revolute', 'prismatic')


@dataclass
class JointConversion:
    """Cuentas de encoder <-> valor articular SI de UNA junta.

    counts_per_unit = cuentas por rad (rotacional) o por metro (prismática).
    sign = +1 si al subir las cuentas sube el valor articular, −1 si baja.
    """

    counts_per_unit: float
    sign: int

    def to_si(self, counts: float) -> float:
        return self.sign * counts / self.counts_per_unit

    def to_counts(self, value: float) -> int:
        return int(round(self.sign * value * self.counts_per_unit))


def declare_transmission_params(node) -> None:
    """Declara los parámetros de transmisión (todos los nodos los leen del mismo YAML)."""
    node.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
    # Juntas con motor instalado. Una junta deshabilitada: el bridge la mantiene frenada
    # (duty_max = 0 en el ESP32 y setpoint = donde está), el homing la salta y goto no la espera.
    node.declare_parameter('joints_enabled', [True, False, False])
    node.declare_parameter('counts_per_motor_rev', 64)
    node.declare_parameter('gear_ratio', [50.0, 50.0, 50.0])
    node.declare_parameter('transmission_ratio', [2.0, 2.0, 1.0])
    node.declare_parameter('lead_mm_per_rev', 1.0)
    node.declare_parameter('joint_sign', [1, 1, 1])
    # Calibración medida (calibrate_joint). > 0 reemplaza el cálculo cpr·reductora·transmisión.
    node.declare_parameter('counts_per_rad', [0.0, 0.0])   # θ1, θ2
    node.declare_parameter('counts_per_m', 0.0)            # Z
    node.declare_parameter('lower_limits', [-math.pi / 2, -2.4435, 0.0])
    node.declare_parameter('upper_limits', [math.pi / 2, 2.4435, 0.120])


def build_conversions(node) -> list:
    """Arma las 3 conversiones con los parámetros del nodo.

    Rotacional: cuentas/rad = cpr · reductora · transmisión / 2π
    Prismática: cuentas/m   = cpr · reductora · transmisión / (paso[m] por vuelta)
    Si hay calibración medida (counts_per_rad / counts_per_m > 0), se usa esa.
    """
    cpr = float(node.get_parameter('counts_per_motor_rev').value)
    gear = list(node.get_parameter('gear_ratio').value)
    trans = list(node.get_parameter('transmission_ratio').value)
    lead_m = float(node.get_parameter('lead_mm_per_rev').value) / 1000.0
    sign = list(node.get_parameter('joint_sign').value)
    measured = list(node.get_parameter('counts_per_rad').value) + \
        [float(node.get_parameter('counts_per_m').value)]
    convs = []
    for j, jtype in enumerate(JOINT_TYPES):
        counts_per_out_rev = cpr * float(gear[j]) * float(trans[j])
        if jtype == 'revolute':
            k = counts_per_out_rev / (2.0 * math.pi)
        else:
            k = counts_per_out_rev / lead_m
        if float(measured[j]) > 0.0:
            k = float(measured[j])   # calibración medida en el robot manda
        convs.append(JointConversion(counts_per_unit=k, sign=1 if int(sign[j]) >= 0 else -1))
    return convs


def joints_enabled(node) -> list:
    return [bool(x) for x in node.get_parameter('joints_enabled').value]


def joint_limits(node):
    return (list(node.get_parameter('lower_limits').value),
            list(node.get_parameter('upper_limits').value))


def fetch_enabled_from_bridge(node, timeout: float = 2.0) -> list:
    """Pregunta al bridge qué juntas están habilitadas (parámetro joints_enabled).

    Para herramientas de línea de comandos (goto, jog). Si el bridge no contesta,
    asume las 3 habilitadas (el bridge igual ignora las deshabilitadas).
    """
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    cli = AsyncParameterClient(node, 'scara_bridge')
    if not cli.wait_for_services(timeout_sec=timeout):
        return [True, True, True]
    fut = cli.get_parameters(['joints_enabled'])
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    try:
        vals = list(fut.result().values[0].bool_array_value)
        return vals if len(vals) == 3 else [True, True, True]
    except Exception:  # noqa: BLE001 — cualquier falla: asumir habilitadas
        return [True, True, True]
