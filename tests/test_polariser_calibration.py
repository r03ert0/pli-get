'''Tests for the polariser calibration fit (synthetic sweeps) and a dry run of
`pli_get --calibrate` on dummy hardware.'''

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

PLI_GET_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PLI_GET_DIR))

import polariser_calibration as pc  # noqa: E402

SWT = 54400          # xypli_large steps per polariser turn
MOTOR_REV = 6400


def sweep(P_true=SWT / 2, err_deg=0.0, Pm=MOTOR_REV, offset=1.1, noise=0.3,
          n_per_turn=72, n_turns=2, seed=0):
    '''A Malus-law sweep with an optional periodic drive-angle error.'''
    rng = np.random.default_rng(seed)
    s = np.array([int(i * SWT / n_per_turn) for i in range(n_per_turn * n_turns)], float)
    amp = err_deg / 180 * P_true

    def phys(s):
        return 2 * np.pi * (s + amp * np.cos(2 * np.pi * s / Pm + 0.7)) / P_true + offset

    # Like the real curve: strong harmonics, symmetric about extinction (as
    # Malus' law requires), and clipped at the camera's black level.
    u = phys(s)
    y = 30 + 27 * np.cos(u) + 7 * np.cos(2 * u) + 4 * np.cos(3 * u) \
        + rng.normal(0, noise, s.size)
    return s, np.clip(y, 0, None), phys


class TestFit:

    def test_recovers_steps_per_turn(self):
        s, y, _ = sweep(P_true=SWT / 2 * 1.004)
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        assert r["turn_error_pct"] == pytest.approx(0.4, abs=0.02)

    def test_lost_steps_are_flagged(self):
        '''A toothed drive can't slip, so a free-fit period well off the
        gearing means missed steps. 0.4 % must be flagged; exact must not.'''
        s, y, _ = sweep(P_true=SWT / 2 * 1.004)
        assert pc.summarise(s, y, SWT, MOTOR_REV)["lost_steps_suspected"]
        s, y, _ = sweep()
        assert not pc.summarise(s, y, SWT, MOTOR_REV)["lost_steps_suspected"]

    def test_period_is_taken_from_the_gearing(self):
        '''Extinction must be computed with the configured period, not the
        free fit: the fits returned for the final move carry P = SWT/2.'''
        s, y, _ = sweep()
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        assert r["fit_plain"]["P"] == SWT / 2
        assert r["fit_drive"]["P"] == SWT / 2

    def test_finds_the_pinion_period(self):
        '''An angle error repeating once per motor turn must be found at that
        period, with the right amplitude.'''
        s, y, _ = sweep(err_deg=0.4)
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        assert r["best_error_period_steps"] == pytest.approx(MOTOR_REV, rel=0.03)
        assert r["motor_period_error_deg"] == pytest.approx(0.4, abs=0.05)
        assert r["rms_with_motor_period_error"] < r["rms_plain"]

    def test_no_drive_error_reports_none(self):
        s, y, _ = sweep(err_deg=0.0)
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        assert r["motor_period_error_deg"] < 0.05

    @pytest.mark.parametrize("seed", range(4))
    def test_extinction_is_accurate(self, seed):
        '''The step returned must put the polariser at the true intensity
        minimum, second harmonic and drive error included.'''
        offset = np.random.default_rng(100 + seed).uniform(0, 2 * np.pi)
        # 0.05 deg drive error: realistic (0.012 deg was measured on the
        # machine). A large drive error breaks the dip's symmetry, which the
        # flank estimator relies on.
        s, y, phys = sweep(err_deg=0.05, offset=offset, seed=seed)
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        target = pc.next_extinction(r["fit_drive"], s[-1])
        assert target >= s[-1], "the final move must be forward"

        uu = np.linspace(0, 2 * np.pi, 200000)
        u_min = uu[np.argmin(27 * np.cos(uu) + 7 * np.cos(2 * uu) + 4 * np.cos(3 * uu))]
        err_deg = ((phys(target) - u_min + np.pi) % (2 * np.pi) - np.pi) * 90 / np.pi
        assert abs(err_deg) < 0.25

    def test_clipped_floor_is_excluded(self):
        s, y, _ = sweep()
        r = pc.summarise(s, y, SWT, MOTOR_REV)
        assert r["n_excluded"] >= (y <= 1.0).sum()
        assert r["rms_plain"] < 1.0, "harmonics + exclusion should fit to near the noise"

    def test_flat_signal_has_no_modulation(self):
        s = np.arange(144.0) * SWT / 72
        r = pc.summarise(s, np.full(s.size, 50.0) + 1e-3 * np.sin(s), SWT, MOTOR_REV)
        assert r["modulation"] < 0.01


class TestDryRun:

    def test_calibrate_on_dummy_hardware(self, tmp_path):
        '''The dummy camera returns black frames: the run must complete, write
        its results, and refuse to set a polariser zero.'''
        out = subprocess.run(
            [sys.executable, "pli_get.py", "--platform", "xypli_large",
             "--calibrate", "--calib_samples", "12", "--calib_turns", "1",
             "--base_path", str(tmp_path)],
            cwd=PLI_GET_DIR, env=dict(os.environ, PLI_DUMMY_HARDWARE="1"),
            capture_output=True, text=True, timeout=300)
        assert out.returncode == 0, out.stdout[-2000:] + out.stderr[-2000:]
        assert "zero was NOT changed" in out.stdout
        res = json.loads((tmp_path / "calibration.json").read_text())
        assert res["zero_set"] is False
        assert (tmp_path / "calibration.png").exists()
        assert (tmp_path / "sweep.csv").exists()
