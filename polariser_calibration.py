'''Polariser calibration: fit the intensity-vs-step curve of a polariser sweep.

With the polariser rotating and the analyser fixed, the transmitted intensity is
180-degree periodic in the polariser angle. Sampling it at many step positions
and fitting a periodic model gives:

  - the true number of motor steps per 180 degrees (P), and so per turn (2P) --
    a direct check of the gear ratio and microstepping in platform_profiles.py;
  - the step position of extinction (the intensity minimum), which becomes the
    polariser's zero, so every acquisition starts from the same physical angle;
  - any periodic ANGLE error in the drive. A pinion that is slightly eccentric,
    or a tight spot in the gear mesh, repeats once per motor revolution. On
    xypli_large the 170/20 gearing puts 8.5 motor revolutions in each polariser
    turn, so such an error lands differently on alternate turns.

The model, for step position s:

    u(s) = 2*pi * (s + e(s)) / P                       optical phase
    e(s) = a*cos(2*pi*s/Pm) + b*sin(2*pi*s/Pm)         drive angle error (steps)
    I(s) = A + sum_{k=1..H} Bk*cos(k*u) + Ck*sin(k*u)

The measured curve on xypli_large is far from a pure Malus cosine: with a
reference sheet in the slide holder it carries strong 2nd-4th harmonics and its
minimum is clipped at the camera's black level. H = 4 harmonics bring the fit
to the noise; fewer leave structure that the drive-error term then absorbs as a
spurious angle error.

Extinction is NOT taken from the model minimum, which moves by ~1 degree with
the number of harmonics. It is taken model-free: the midpoint between the two
flanks of each dip, measured at several intensity levels. The dip is symmetric
about extinction whatever its shape, and the clipped floor doesn't matter.
Clipped samples and the first frame (a first-grab artefact) are excluded
from all fits. For fixed (P, a, b) the model is linear in the
remaining coefficients, which are solved exactly at every step of the
non-linear fit (variable projection).

This module has no hardware dependencies, so it can be tested on synthetic data.
'''

import numpy as np
from scipy.optimize import least_squares


N_HARMONICS = 4


def _design(s, P, a, b, Pm, H=N_HARMONICS):
    e = 0.0
    if Pm:
        w = 2 * np.pi * s / Pm
        e = a * np.cos(w) + b * np.sin(w)
    u = 2 * np.pi * (s + e) / P
    cols = [np.ones_like(u)]
    for k in range(1, H + 1):
        cols += [np.cos(k * u), np.sin(k * u)]
    return np.column_stack(cols)


def _solve_linear(s, y, P, a=0.0, b=0.0, Pm=None):
    X = _design(s, P, a, b, Pm)
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        return coef, y - X @ coef


def fit_period(steps, values, nominal_period, span=0.1, n_grid=2001):
    '''Best period P (steps per 180 degrees) with no drive-error term.

    Grid search over nominal*(1 +/- span), then non-linear refinement. The
    linear coefficients are solved exactly for every candidate P.
    '''
    s = np.asarray(steps, float)
    y = np.asarray(values, float)
    grid = np.linspace(nominal_period * (1 - span),
                       nominal_period * (1 + span), n_grid)
    rss = [np.sum(_solve_linear(s, y, P)[1] ** 2) for P in grid]
    P0 = grid[int(np.argmin(rss))]

    res = least_squares(lambda p: _solve_linear(s, y, p[0])[1], [P0],
                        x_scale=[nominal_period * 1e-3])
    P = float(res.x[0])
    coef, r = _solve_linear(s, y, P)
    return {"P": P, "coef": coef, "rms": float(np.sqrt(np.mean(r ** 2)))}


def fit_with_drive_error(steps, values, P0, Pm, fix_P=False):
    '''Fit a sinusoidal drive-angle error of period Pm, and P unless fix_P.'''
    s = np.asarray(steps, float)
    y = np.asarray(values, float)

    if fix_P:
        res = least_squares(lambda p: _solve_linear(s, y, P0, p[0], p[1], Pm)[1],
                            [0.0, 0.0], x_scale=[10.0, 10.0])
        P, (a, b) = float(P0), (float(v) for v in res.x)
    else:
        res = least_squares(lambda p: _solve_linear(s, y, p[0], p[1], p[2], Pm)[1],
                            [P0, 0.0, 0.0], x_scale=[P0 * 1e-3, 10.0, 10.0])
        P, a, b = (float(v) for v in res.x)
    coef, r = _solve_linear(s, y, P, a, b, Pm)
    return {"P": P, "a": a, "b": b, "Pm": Pm, "coef": coef,
            "rms": float(np.sqrt(np.mean(r ** 2)))}


def fixed_period_fit(steps, values, P):
    '''Fit at a known period P (no free parameters beyond the linear ones).'''
    coef, r = _solve_linear(np.asarray(steps, float), np.asarray(values, float), P)
    return {"P": float(P), "coef": coef, "rms": float(np.sqrt(np.mean(r ** 2)))}


def scan_error_period(steps, values, P0, candidates):
    '''Which drive-error period explains the residuals best?

    Returns [(Pm, rms)], one per candidate. A clear minimum at one motor
    revolution points at the pinion; no minimum means no periodic drive error
    above the noise.
    '''
    return [(float(Pm), fit_with_drive_error(steps, values, P0, Pm, fix_P=True)["rms"])
            for Pm in candidates]


def model(steps, fit):
    '''Evaluate a fit (from fit_period or fit_with_drive_error).'''
    s = np.asarray(steps, float)
    X = _design(s, fit["P"], fit.get("a", 0.0), fit.get("b", 0.0),
                fit.get("Pm"))
    # macOS Accelerate can raise spurious FP warnings in small matmuls
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        return X @ fit["coef"]


def model_minimum_step(fit, n=4000):
    '''Step offset in [0, P) of the fitted curve's minimum (drive error ignored).

    Only used to locate the dip roughly; see flank_extinction_step().
    '''
    P = fit["P"]
    s = np.linspace(0, P, n, endpoint=False)
    y = model(s, {**fit, "Pm": None})
    return float(s[int(np.argmin(y))])


def flank_extinction_step(steps, values, P, rough, levels=(0.25, 0.35, 0.45)):
    '''Model-free extinction: midpoint of the dip's two flanks.

    Samples are folded onto one period (s mod P) around the rough minimum. At
    each level (a fraction of the dip depth) the falling and rising flanks are
    located by linear interpolation; the midpoints are averaged. Returns
    (offset in [0, P), spread of the per-level midpoints in steps).
    '''
    s = np.asarray(steps, float)
    y = np.asarray(values, float)
    d = (s - rough + P / 2) % P - P / 2          # position relative to the dip
    order = np.argsort(d)
    d, y = d[order], y[order]
    left, right = d < 0, d >= 0
    lo, hi = np.percentile(y, 2), np.percentile(y, 98)
    mids = []
    for f in levels:
        lvl = lo + f * (hi - lo)
        # falling flank: last crossing below lvl approaching the dip from the left
        dl, yl = d[left], y[left]
        dr, yr = d[right], y[right]
        try:
            i = np.where((yl[:-1] >= lvl) & (yl[1:] < lvl))[0][-1]
            j = np.where((yr[:-1] < lvl) & (yr[1:] >= lvl))[0][0]
        except IndexError:
            continue
        xl = dl[i] + (lvl - yl[i]) / (yl[i + 1] - yl[i]) * (dl[i + 1] - dl[i])
        xr = dr[j] + (lvl - yr[j]) / (yr[j + 1] - yr[j]) * (dr[j + 1] - dr[j])
        mids.append((xl + xr) / 2)
    if not mids:
        return rough % P, float("nan")
    return float((rough + np.mean(mids)) % P), float(np.ptp(mids))


def dip_extinctions(steps, values, P, rough, margin_frac=0.25):
    '''Extinction from each dip on its own, using only dips whose flanks both
    lie inside the sweep (at least margin_frac*P from either end).

    Returns a list of absolute step positions, one per usable dip. Estimating
    dips separately -- rather than folding the sweep onto one period -- keeps
    a dip that straddles the sweep's ends from contaminating the result, and
    the agreement between dips is a direct quality check.
    '''
    s = np.asarray(steps, float)
    y = np.asarray(values, float)
    smin, smax = s.min(), s.max()
    margin = margin_frac * P
    out = []
    k0 = int(np.floor((smin - rough) / P))
    for k in range(k0, k0 + int((smax - smin) / P) + 3):
        c = rough + k * P
        if c - margin < smin or c + margin > smax:
            continue
        near = np.abs(s - c) < P / 2
        off, _ = flank_extinction_step(s[near], y[near], P, c)
        out.append(c + ((off - c + P / 2) % P - P / 2))
    return out


def extinction_phase_step(fit, n=4000):
    '''Step offset in [0, P) of extinction: the model-free value if one was
    stored by summarise(), else the model minimum.'''
    if "s0" in fit:
        return fit["s0"]
    return model_minimum_step(fit, n)


def next_extinction(fit, at_or_after):
    '''Smallest step position >= at_or_after where the intensity is minimal.

    With a drive-error term, the commanded step s gives physical position
    s + e(s), so solve s + e(s) = target by fixed-point iteration.
    '''
    P = fit["P"]
    if fit.get("dips"):
        # Extrapolate from the last MEASURED dip, not from step 0: any error in
        # P then only acts over the short distance to the sweep's end.
        base = max(fit["dips"])
    else:
        base = extinction_phase_step(fit)
    k = np.ceil((at_or_after - base) / P)
    target = base + k * P
    s = target
    if fit.get("Pm"):
        for _ in range(20):
            w = 2 * np.pi * s / fit["Pm"]
            s = target - (fit["a"] * np.cos(w) + fit["b"] * np.sin(w))
        if s < at_or_after:
            return next_extinction(fit, at_or_after + P / 2)
    return int(round(s))


def channel_fit(steps, values, P, clip_level=1.0, settle_frames=2):
    '''Light per-channel analysis for plotting and comparison: a fixed-period
    fit and the model-free extinction, with the same exclusions as summarise().

    Returns (fit, extinction in degrees within the half-turn), or (None, nan)
    if too few samples survive.
    '''
    s = np.asarray(steps, float)
    y = np.asarray(values, float)
    keep = y > clip_level
    keep[:settle_frames] = False
    s, y = s[keep], y[keep]
    if keep.sum() < 4 * N_HARMONICS + 6 or np.ptp(y) <= 0:
        return None, float("nan")
    fit = fixed_period_fit(s, y, P)
    s0, _ = flank_extinction_step(s, y, P, model_minimum_step(fit))
    return fit, s0 / P * 180.0


def modulation(fit):
    '''Peak-to-peak of the fitted curve over its mean.'''
    y = model(np.linspace(0, fit["P"], 2000), {**fit, "Pm": None})
    return float((y.max() - y.min()) / max(abs(fit["coef"][0]), 1e-12))


# A toothed drive cannot slip: steps per turn is exact unless the stepper
# misses steps. A free-fit period further than this from the configured one
# is reported as possible lost steps.
LOST_STEPS_WARN_PCT = 0.1


def summarise(steps, values, steps_whole_turn, motor_steps_per_rev,
              n_scan=160, clip_level=1.0, settle_frames=2):
    '''Full analysis of one sweep. Returns a dict of results.

    Samples at or below `clip_level` (black-level clipped) and the first
    `settle_frames` frames are excluded from the fits.
    '''
    s_all = np.asarray(steps, float)
    y_all = np.asarray(values, float)
    keep = y_all > clip_level
    keep[:settle_frames] = False
    s, y = s_all[keep], y_all[keep]
    nominal = steps_whole_turn / 2.0

    if keep.sum() < 4 * N_HARMONICS + 6 or np.ptp(y) <= 0:
        # Nothing to fit: black frames (no light, or the dummy camera) or a
        # flat signal. Report zero modulation so the caller doesn't move.
        empty = {"P": nominal, "coef": np.zeros(2 * N_HARMONICS + 1),
                 "rms": float("nan")}
        nan = float("nan")
        return {"n_excluded": int((~keep).sum()), "n_samples": int(len(s_all)),
                "modulation": 0.0, "extinction_deg": nan,
                "extinction_model_deg": nan, "extinction_spread_deg": nan,
                "n_dips": 0, "dip_disagreement_deg": nan,
                "configured_steps_per_turn": float(steps_whole_turn),
                "measured_steps_per_turn": nan, "turn_error_pct": nan,
                "lost_steps_suspected": False,
                "rms_plain": nan, "rms_with_motor_period_error": nan,
                "motor_period_error_deg": nan, "best_error_period_steps": nan,
                "best_error_period_rms": nan, "best_error_amplitude_deg": nan,
                "motor_steps_per_rev": float(motor_steps_per_rev),
                "scan": [], "fit_plain": empty, "fit_drive": empty}

    # The period is known exactly from the gearing (a toothed drive can't slip),
    # so every estimate below uses it. The free fit is kept only as a check for
    # lost steps -- one sweep can estimate it to ~+/-0.03%, no better.
    free = fit_period(s, y, nominal)
    base = fixed_period_fit(s, y, nominal)
    spacing = np.median(np.diff(np.sort(s)))
    lo, hi = 2.2 * spacing, 2.0 * steps_whole_turn
    candidates = np.unique(np.concatenate([
        np.geomspace(lo, hi, n_scan), [motor_steps_per_rev]]))
    scan = scan_error_period(s, y, base["P"], candidates)
    best_Pm = min(scan, key=lambda t: t[1])[0]

    drive = fit_with_drive_error(s, y, nominal, motor_steps_per_rev, fix_P=True)
    best = fit_with_drive_error(s, y, nominal, best_Pm, fix_P=True)

    # model-free extinction, stored on both fits so next_extinction uses it
    rough = model_minimum_step(base)
    P = base["P"]
    dips = dip_extinctions(s, y, P, rough)
    _, spread = flank_extinction_step(s, y, P, rough)
    if dips:
        rel = [(d - rough + P / 2) % P - P / 2 for d in dips]
        s0 = (rough + float(np.mean(rel))) % P
        disagreement = float(np.ptp(rel)) / P * 180.0 if len(rel) > 1 else 0.0
    else:
        s0, spread = flank_extinction_step(s, y, P, rough)
        disagreement = float("nan")
    base["s0"] = s0
    drive["s0"] = s0
    base["dips"] = drive["dips"] = [float(d) for d in dips]
    deg_per_step = 180.0 / drive["P"]
    return {
        "n_excluded": int((~keep).sum()),
        "n_dips": len(dips),
        "dip_disagreement_deg": disagreement,
        "extinction_deg": s0 / base["P"] * 180.0,
        "extinction_model_deg": rough / base["P"] * 180.0,
        "extinction_spread_deg": spread / base["P"] * 180.0,
        "n_samples": int(len(s)),
        "configured_steps_per_turn": float(steps_whole_turn),
        "measured_steps_per_turn": 2 * free["P"],
        "turn_error_pct": 100 * (2 * free["P"] / steps_whole_turn - 1),
        "lost_steps_suspected": bool(
            abs(100 * (2 * free["P"] / steps_whole_turn - 1)) > LOST_STEPS_WARN_PCT),
        "modulation": modulation(base),
        "rms_plain": base["rms"],
        "rms_with_motor_period_error": drive["rms"],
        "motor_period_error_deg": float(np.hypot(drive["a"], drive["b"]) * deg_per_step),
        "best_error_period_steps": float(best_Pm),
        "best_error_period_rms": best["rms"],
        "best_error_amplitude_deg": float(np.hypot(best["a"], best["b"]) * 180.0 / best["P"]),
        "motor_steps_per_rev": float(motor_steps_per_rev),
        "scan": scan,
        "fit_plain": base,
        "fit_drive": drive,
    }
