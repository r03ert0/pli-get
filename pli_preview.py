'''Quick-look retardation and transmittance previews for a PLI run.

The raw FOVs are nearly featureless to the eye: the polarised signal is a few
percent modulation on top of a bright background, so a contact sheet of the raw
angles tells you very little about whether the acquisition actually captured
tissue. Fitting the PLI model and stretching the result makes it obvious.

The model is the usual Jones-calculus fit (see plipy/src/pli_fit.py):

    I(rho) = a0 + a1*sin(2*rho) + b1*cos(2*rho)

    transmittance = 2*a0
    retardation   = sqrt(a1^2 + b1^2) / a0
    azimuth       = in-plane fibre orientation from (a1, b1)

The pipeline is ok for previewing, but not for quantitative work:

  - No background/flat-field normalisation. plipy multiplies by a calibration
    matrix measured on an empty slide; without it, uneven illumination shows up
    as a smooth gradient across each FOV.
  - No inter-angle registration. plipy corrects the small translation the
    rotating polariser introduces.

IMPORTANT -- the polariser sweep. pli_get rotates `steps_whole_turn/n_angles`
per image, i.e. it covers a FULL 360 degrees, while the PLI signal itself is
only 180-degree periodic. So each optical state is acquired twice whenever
n_angles is even -- which is exactly what plipy's n_averages=2 and its
`{j*n_angles+i}` indexing are for.

This module handles that by folding phases modulo 180 and averaging the
repeats before fitting (a sqrt(2) noise reduction, worth having when the
modulation is only a few percent). An odd n_angles has no repeats: 9 steps of
40 degrees fold to nine distinct phases 20 degrees apart, and the stack is
used as-is.

Either way the fit uses the TRUE physical phase. Fitting a 360-degree
acquisition with plipy's 180-degree basis uses the wrong design matrix and
corrupts both retardation and azimuth. The default sweep here is 360; use
--sweep 180 for plipy-style data that was acquired over a half turn.

Usage:

    # retardation + transmittance mosaics for a whole run
    python pli_preview.py --base_path ~/Desktop/run

    # cheap version: transmittance only, no model fit
    python pli_preview.py --base_path ~/Desktop/run --mode transmittance

    # full resolution is rarely needed for a preview
    python pli_preview.py --base_path ~/Desktop/run --reduce 4
'''

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

FOV_RE = re.compile(r'^fov_(\d+)_(\d+)$')


def _open_reduced(path, reduce_factor, channel):
    '''Load one image, box-reducing it on the way in.

    PIL's reduce() is a cheap integer box filter, which keeps a 9-angle stack of
    4504x4504 frames from turning a preview into a memory problem.
    '''
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None

    im = Image.open(path)
    if reduce_factor > 1:
        im = im.reduce(reduce_factor)
    arr = np.asarray(im)

    if arr.ndim == 3:
        arr = arr[:, :, channel]
    if arr.dtype.kind == "f":
        arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    # Normalise to [0, 1] by dtype range so 8- and 12/16-bit runs behave alike.
    if arr.dtype == np.uint8:
        return arr.astype(np.float32) / 255.0
    if arr.dtype == np.uint16:
        return arr.astype(np.float32) / 65535.0
    return arr.astype(np.float32)


def load_fov_stack(fov_dir, reduce_factor=8, channel=1, ext="tif"):
    '''(h, w, n_angles) float stack for one FOV, angles in acquisition order.'''
    fov_dir = Path(fov_dir)
    frames = sorted(fov_dir.glob(f"*.{ext}"), key=lambda p: int(p.stem))
    if not frames:
        return None
    return np.stack(
        [_open_reduced(p, reduce_factor, channel) for p in frames], axis=-1)


def fold_duplicate_angles(im_vol, sweep_deg=360.0):
    '''Average images that sample the same optical state.

    The PLI signal is 180-degree periodic, but pli_get sweeps the polariser a
    full 360. Image i therefore sits at phase (i*sweep/n) mod 180, and for an
    EVEN number of angles images i and i+n/2 are the same state acquired twice.
    Averaging those pairs is what plipy does with n_averages=2, and it buys a
    sqrt(2) noise reduction -- worth having when the modulation is only a few
    percent.

    For an odd number of angles there are no collisions: 9 steps of 40 degrees
    fold to nine distinct phases 20 degrees apart. The stack comes back
    unchanged and the phases are still correct.

    Returns (folded_stack, rho_radians, n_repeats).
    '''
    n = im_vol.shape[2]
    phase = np.mod(np.arange(n) * sweep_deg / n, 180.0)
    uniq, inverse = np.unique(np.round(phase, 6), return_inverse=True)

    if len(uniq) == n:
        return im_vol, np.deg2rad(phase), 1

    folded = np.stack(
        [im_vol[:, :, inverse == k].mean(axis=2) for k in range(len(uniq))],
        axis=-1)
    return folded, np.deg2rad(uniq), n // len(uniq)


def fit_coefficients(im_vol, sweep_deg=360.0):
    '''Least-squares fit of I = a0 + a1*sin(2*rho) + b1*cos(2*rho).

    `sweep_deg` is the total polariser rotation across the stack: 360 for
    pli_get, 180 for plipy-style acquisitions. Duplicate optical states are
    folded and averaged first.

    Using the true physical phase is what matters here. Fitting a 360-degree
    acquisition with plipy's 180-degree basis uses the wrong design matrix and
    corrupts both retardation and azimuth.
    '''
    folded, rho, n_repeats = fold_duplicate_angles(im_vol, sweep_deg)
    h, w, n = folded.shape
    rho = rho.reshape(-1, 1)

    if n < 3:
        raise ValueError(
            f"need at least 3 distinct polariser phases to fit, got {n}")

    design = np.hstack((np.ones((n, 1)), np.sin(2 * rho), np.cos(2 * rho)))
    values = folded.reshape(h * w, n).T

    coeffs = np.linalg.lstsq(design, values, rcond=None)[0]
    a0 = coeffs[0].reshape(h, w)
    a1 = coeffs[1].reshape(h, w)
    b1 = coeffs[2].reshape(h, w)

    # Degenerate pixels (saturated or dead) can make lstsq return huge
    # coefficients; the residual is only a quality report, so don't let it warn.
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        residual = values - design @ coeffs
        rms = float(np.sqrt(np.nanmean(residual ** 2)))
    return a0, a1, b1, rms, n_repeats


def transmittance_map(a0):
    return 2.0 * a0


def retardation_map(a0, a1, b1):
    '''sqrt(a1^2 + b1^2) / a0, clipped to [0, 1].

    a0 is half the mean intensity, so it is only near zero where the image is
    black; the epsilon keeps those pixels finite rather than exploding.
    '''
    r = np.sqrt(a1 ** 2 + b1 ** 2) / (np.abs(a0) + 1e-10)
    return np.clip(r, 0.0, 1.0)


def normalise_tiles(vectors, global_bg=None):
    '''Per-tile background correction for the preview.

    `vectors` is {cell: (u, v)}, the normalised polarisation vector per pixel
    (u = a1/a0, v = b1/a0). On xypli_large alternate acquisitions show the same
    polariser angles but ~40 % less modulation, and a different background
    vector (docs/report-7 §4), which prints as bands across the mosaic.

    Each tile's background vector is estimated as the per-tile median (tissue
    is assumed to be the minority of pixels) and subtracted; the tile's noise
    floor (median of what remains) is removed; the result is then scaled so
    every tile's background magnitude matches the global median, undoing the
    contrast scaling. Returns {cell: 2D retardation-like map}.

    A preview aid only: it assumes background dominates each tile, and it hides
    the underlying problem rather than fixing the data.
    '''
    bg = {k: (np.median(u), np.median(v)) for k, (u, v) in vectors.items()}
    mags = {k: float(np.hypot(*b)) for k, b in bg.items()}
    ref = global_bg if global_bg is not None else float(np.median(list(mags.values())))
    out = {}
    for k, (u, v) in vectors.items():
        gain = ref / mags[k] if mags[k] > 1e-9 else 1.0
        m = np.hypot(u - bg[k][0], v - bg[k][1])
        # What remains in empty glass is noise, whose floor differs between
        # tiles; remove it before the gain, or the gain amplifies it into bands.
        out[k] = np.clip(m - np.median(m), 0, None) * gain
    return out


def azimuth_map(a1, b1):
    '''In-plane orientation in [0, pi).'''
    return 0.5 * np.mod(np.arctan2(-b1, a1), 2 * np.pi)


def stretch(img, low=2.0, high=98.0):
    '''Percentile contrast stretch to [0, 1].

    The whole point of the preview: raw retardation on unnormalised data sits
    in a narrow band just above zero and looks uniformly black otherwise.
    '''
    finite = img[np.isfinite(img)]
    if finite.size == 0:
        return np.zeros_like(img)
    lo, hi = np.percentile(finite, [low, high])
    if hi <= lo:
        return np.zeros_like(img)
    return np.clip((img - lo) / (hi - lo), 0.0, 1.0)


def _crop_to_pitch(tile, margin_pct):
    '''Trim the overlap svg_acquire builds into the grid.

    Adjacent FOVs share `fov_margin_pct` of their width at each edge, so the
    centre (1 - 2*margin) of each tile is what tiles without duplication.
    '''
    if margin_pct <= 0:
        return tile
    h, w = tile.shape[:2]
    dy, dx = int(round(h * margin_pct)), int(round(w * margin_pct))
    return tile[dy:h - dy, dx:w - dx]


def load_positions(roi_dir, fovs):
    '''Stage position (x, y) of each FOV, and the platform it was taken on.

    pli_get writes positions.json alongside the FOVs. Runs made before it did
    fall back to svg_acquire's convention -- ROIs with negative width and
    height, so the stage steps in -x as the column index grows and in -y as
    the row index grows. Only the ordering matters, so relative units do.
    '''
    f = Path(roi_dir) / "positions.json"
    if f.exists():
        data = json.loads(f.read_text())
        pos = {}
        for name, (x, y) in data["fovs"].items():
            m = FOV_RE.match(name)
            pos[(int(m.group(1)), int(m.group(2)))] = (x, y)
        return pos, data.get("platform")
    return {(r, c): (-float(c), -float(r)) for (r, c) in fovs}, None


def camera_axes_for(platform):
    '''(image right, image down) as stage axes, from the platform profile.'''
    if platform:
        try:
            from platform_profiles import PROFILES
            axes = PROFILES.get(platform, {}).get("camera_image_axes")
            if axes:
                return tuple(axes)
        except ImportError:
            pass
    return None


def image_grid(positions, axes):
    '''Map each FOV to its (row, col) cell as seen in the camera image.

    `axes` says which signed stage axis runs along image right and image down,
    e.g. ("-y", "+x"). Cells come from ranking the distinct coordinates, so no
    pixel scale is needed and any grid spacing works.
    '''
    def coord(spec, x, y):
        v = x if spec[1] == "x" else y
        return -v if spec[0] == "-" else v

    uv = {k: (coord(axes[0], x, y), coord(axes[1], x, y))
          for k, (x, y) in positions.items()}
    us = sorted({round(u, 3) for u, _ in uv.values()})
    vs = sorted({round(v, 3) for _, v in uv.values()})
    return {k: (vs.index(round(v, 3)), us.index(round(u, 3)))
            for k, (u, v) in uv.items()}


# Layout used when neither positions.json nor a profile says otherwise: the
# plain fov_row_col grid, which is what earlier versions produced.
LEGACY_AXES = ("-x", "-y")


def assemble_mosaic(tiles, margin_pct=0.1):
    '''Lay {(row, col): 2D array} out as a single image.

    Keys are cells in the IMAGE grid (see image_grid), not fov_row_col indices:
    on xypli_large the camera is rotated relative to the stage, and placing
    tiles by index scrambles their order.
    '''
    if not tiles:
        return None
    cropped = {k: _crop_to_pitch(v, margin_pct) for k, v in tiles.items()}
    th = min(t.shape[0] for t in cropped.values())
    tw = min(t.shape[1] for t in cropped.values())

    rows = max(r for r, _ in cropped) + 1
    cols = max(c for _, c in cropped) + 1
    out = np.full((rows * th, cols * tw), np.nan, dtype=np.float32)
    for (r, c), tile in cropped.items():
        out[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile[:th, :tw]
    return out


def save_image(arr, path, cmap=None):
    '''Write a float [0,1] array, optionally through a matplotlib colormap.'''
    from PIL import Image

    arr = np.nan_to_num(arr, nan=0.0)
    if cmap:
        try:
            from matplotlib import colormaps
            mapper = colormaps[cmap]
        except (ImportError, KeyError):  # matplotlib < 3.5
            import matplotlib.cm as mcm
            mapper = mcm.get_cmap(cmap)
        rgba = mapper(arr)
        img = Image.fromarray((rgba[:, :, :3] * 255).astype(np.uint8))
    else:
        img = Image.fromarray((arr * 255).astype(np.uint8))
    img.save(path)
    return path


def find_rois(base_path):
    '''{roi_label: {(row, col): fov_dir}} for a run directory.

    Handles both the labelled layout (roi_<label>/fov_r_c) and a bare
    fov_r_c directly under base_path.
    '''
    base = Path(base_path)
    rois = {}

    loose = {}
    for child in sorted(base.iterdir()):
        if not child.is_dir():
            continue
        m = FOV_RE.match(child.name)
        if m:
            loose[(int(m.group(1)), int(m.group(2)))] = child
        elif child.name.startswith("roi_"):
            fovs = {}
            for sub in sorted(child.iterdir()):
                m2 = FOV_RE.match(sub.name)
                if m2 and sub.is_dir():
                    fovs[(int(m2.group(1)), int(m2.group(2)))] = sub
            if fovs:
                rois[child.name[4:]] = fovs
    if loose:
        rois[""] = loose
    return rois


def preview_roi(fovs, mode="retardation", reduce_factor=8, channel=1,
                sweep_deg=360.0, margin_pct=0.1, verbose=True,
                platform=None, roi_dir=None, normalise=False):
    '''Compute preview mosaics for one ROI. Returns {name: 2D array}.'''
    retard, trans, vectors = {}, {}, {}

    positions, run_platform = load_positions(
        roi_dir or next(iter(fovs.values())).parent, fovs)
    axes = camera_axes_for(platform or run_platform)
    if axes is None:
        axes = LEGACY_AXES
        if verbose:
            print("    no camera orientation known for this platform: "
                  "laying tiles out by fov_row_col index")
    cell = image_grid(positions, axes)

    for (row, col), fov_dir in sorted(fovs.items()):
        stack = load_fov_stack(fov_dir, reduce_factor, channel)
        if stack is None:
            if verbose:
                print(f"    fov_{row}_{col}: no images, skipped")
            continue

        if mode == "transmittance":
            # Mean across distinct phases; no fit, so this stays cheap. Folding
            # first keeps an even-angle run from weighting phases unevenly.
            folded, _, _ = fold_duplicate_angles(stack, sweep_deg)
            t = folded.mean(axis=2)
            trans[(row, col)] = t
            if verbose:
                print(f"    fov_{row}_{col}: mean {t.mean():.4f}")
            continue

        a0, a1, b1, rms, n_repeats = fit_coefficients(stack, sweep_deg)
        r = retardation_map(a0, a1, b1)
        retard[(row, col)] = r
        with np.errstate(divide="ignore", invalid="ignore"):
            denom = np.abs(a0) + 1e-10
            vectors[(row, col)] = (a1 / denom, b1 / denom)
        trans[(row, col)] = transmittance_map(a0)
        if verbose:
            rep = f"  ({n_repeats}x averaged)" if n_repeats > 1 else ""
            print(f"    fov_{row}_{col}: median retardation {np.median(r):.4f}"
                  f"   fit RMS {rms:.5f}{rep}")

    out = {}
    if retard and normalise:
        norm = normalise_tiles(vectors)
        out["retardation"] = assemble_mosaic(
            {cell[k]: v for k, v in norm.items()}, margin_pct)
        out["retardation_raw"] = assemble_mosaic(
            {cell[k]: v for k, v in retard.items()}, margin_pct)
    elif retard:
        out["retardation"] = assemble_mosaic(
            {cell[k]: v for k, v in retard.items()}, margin_pct)
    if trans:
        out["transmittance"] = assemble_mosaic(
            {cell[k]: v for k, v in trans.items()}, margin_pct)
    return out


def preview_run(base_path, mode="retardation", reduce_factor=8, channel=1,
                sweep_deg=360.0, margin_pct=0.1, out_dir=None, verbose=True,
                platform=None, normalise=False):
    '''Write preview mosaics for every ROI in a run. Returns written paths.'''
    base = Path(base_path)
    out_dir = Path(out_dir) if out_dir else base
    out_dir.mkdir(parents=True, exist_ok=True)

    rois = find_rois(base)
    if not rois:
        print(f"No roi_*/fov_* or fov_* directories under {base}")
        return []

    written = []
    for label, fovs in sorted(rois.items()):
        name = f"roi_{label}" if label else "roi"
        if verbose:
            print(f"  {name}: {len(fovs)} FOVs")

        maps = preview_roi(fovs, mode, reduce_factor, channel, sweep_deg,
                           margin_pct, verbose, platform=platform,
                           normalise=normalise)
        for kind, mosaic in maps.items():
            if mosaic is None:
                continue
            cmap = "inferno" if kind.startswith("retardation") else None
            path = out_dir / f"preview_{name}_{kind}.png"
            save_image(stretch(mosaic), path, cmap=cmap)
            written.append(path)
            if verbose:
                finite = mosaic[np.isfinite(mosaic)]
                print(f"    -> {path.name}  "
                      f"[{finite.min():.4f}, {finite.max():.4f}] "
                      f"median {np.median(finite):.4f}")
    return written


def main(args):
    written = preview_run(
        args.base_path, mode=args.mode, reduce_factor=args.reduce,
        channel=args.channel, sweep_deg=args.sweep,
        margin_pct=args.fov_margin_pct, out_dir=args.out_dir,
        platform=args.platform, normalise=args.normalise)
    if not written:
        return 1
    print(f"\n{len(written)} preview image(s) written.")
    print("Note: no background normalisation or inter-angle registration -- "
          "preview only, not for quantitative use.")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base_path', required=True,
                        help='[required] run directory holding roi_*/fov_*_*')
    parser.add_argument('--mode', choices=['retardation', 'transmittance'],
                        default='retardation',
                        help='retardation fits the PLI model; transmittance is '
                             'just the mean across angles (much cheaper)')
    parser.add_argument('--reduce', type=int, default=8,
                        help='integer downsample factor applied on load '
                             '(default 8)')
    parser.add_argument('--channel', type=int, default=1,
                        help='colour channel for RGB input, 0=R 1=G 2=B '
                             '(default 1, green)')
    parser.add_argument('--sweep', type=float, default=360.0,
                        help='total polariser rotation across the stack in '
                             'degrees: 360 for pli_get, 180 for plipy data')
    parser.add_argument('--fov_margin_pct', type=float, default=0.1,
                        help='per-edge FOV overlap to trim when tiling '
                             '(default 0.1, matching svg_acquire)')
    parser.add_argument('--platform', default='xypli_large',
                        help='platform profile giving the camera orientation '
                             '(default xypli_large; overridden by the one '
                             'recorded in positions.json)')
    parser.add_argument('--normalise', action='store_true',
                        help='per-tile background subtraction and gain matching '
                             '(written as preview_*_retardation.png, with the raw '
                             'map as *_raw.png). Off by default: it was a workaround '
                             'for the white-balance alternation, now fixed at source')
    parser.add_argument('--out_dir', default=None,
                        help='where to write previews (default: base_path)')

    sys.exit(main(parser.parse_args()))
