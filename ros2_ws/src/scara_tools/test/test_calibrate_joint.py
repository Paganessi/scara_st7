"""Pruebas de la parte de cálculo y escritura del YAML de calibrate_joint (sin hardware)."""
import math
import shutil

import pytest
from scara_tools.calibrate_joint import compute, update_yaml

YAML = """/**:
  ros__parameters:
    joint_sign: [1, 1, 1]                    # TODO CONFIRMAR en Fase 1
    counts_per_rad: [0.0, 0.0]               # θ1, θ2 (cuentas por radián)
    counts_per_m: 0.0                        # Z (cuentas por metro)
"""


def test_compute_90_grados_teorico():
    # 64 cuentas × 50:1 × 2:1 = 6400 cuentas/vuelta → 1600 cuentas en 90°
    per_rad, sign = compute(1600, math.radians(90), prismatic=False)
    assert per_rad == pytest.approx(6400 / (2 * math.pi))
    assert sign == 1


def test_compute_signo_negativo():
    per_rad, sign = compute(-6400, 2 * math.pi, prismatic=False)
    assert per_rad == pytest.approx(1018.59, abs=0.01)
    assert sign == -1


def test_compute_sin_movimiento_falla():
    with pytest.raises(ValueError):
        compute(0, 1.0, prismatic=False)


def test_update_yaml_junta1_conserva_comentarios(tmp_path):
    p = tmp_path / 'scara.yaml'
    p.write_text(YAML)
    text = update_yaml(str(p), 1, 1018.5932, -1)
    assert 'counts_per_rad: [1018.59, 0.0]               # θ1, θ2 (cuentas por radián)' in text
    assert 'joint_sign: [-1, 1, 1]                    # TODO CONFIRMAR en Fase 1' in text
    assert 'counts_per_m: 0.0' in text


def test_update_yaml_z(tmp_path):
    p = tmp_path / 'scara.yaml'
    p.write_text(YAML)
    text = update_yaml(str(p), 3, 3200000.0, 1)
    assert 'counts_per_m: 3200000.0                        # Z' in text
    assert 'counts_per_rad: [0.0, 0.0]' in text


def test_update_yaml_real_del_repo(tmp_path):
    """El scara.yaml real tiene las claves con el formato que la herramienta espera."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    real = os.path.join(here, '..', '..', 'scara_bringup', 'config', 'scara.yaml')
    if not os.path.exists(real):
        pytest.skip('scara.yaml no disponible')
    p = tmp_path / 'scara.yaml'
    shutil.copy(real, p)
    text = update_yaml(str(p), 2, 1000.0, 1)
    assert 'counts_per_rad: [0.0, 1000.00]' in text
