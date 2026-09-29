'''Tests for the PLI quick-look preview.

The important one is test_fit_recovers_known_coefficients: it builds a synthetic
stack from known a0/a1/b1 at the exact polariser angles pli_get uses, then
checks the fit gets them back. That is what catches a wrong design matrix --
the failure mode where retardation and azimuth come out plausible but wrong.

Run with:  pytest pli_get/tests -v
'''

import sys
from pathlib import Path

import numpy as np
import pytest

PLI_GET_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PLI_GET_DIR))

from pli_preview import (  # noqa: E402
    fold_duplicate_angles, fit_coefficients, retardation_map,
    transmittance_map, azimuth_map, stretch, assemble_mosaic, find_rois)


def synth_stack(a0, a1, b1, n_angles, sweep_deg=360.0, shape=(4, 5)):
    '''Synthesise I = a0 + a1*sin(2rho) + b1*cos(2rho) at pli_get's angles.

    pli_get rotates steps_whole_turn/n_angles per image, so the physical
    polariser angle of image i is i*sweep/n -- a full turn by default.
    '''
    rho = np.deg2rad(np.arange(n_angles) * sweep_deg / n_angles)
    h, w = shape
    sig = a0 + a1 * np.sin(2 * rho) + b1 * np.cos(2 * rho)
    return np.broadcast_to(sig, (h, w, n_angles)).astype(np.float64).copy()


class TestAngleFolding:
    '''The PLI signal is 180-periodic but the polariser turns 360, so an even
    angle count acquires every optical state twice.'''

    @pytest.mark.parametrize("n,expected_distinct,expected_reps", [
        (9, 9, 1),     # odd: nine phases 20 deg apart, no repeats
        (18, 9, 2),    # even: each state twice
        (36, 18, 2),
        (72, 36, 2),
    ])
    def test_distinct_phase_count(self, n, expected_distinct, expected_reps):
        vol = np.zeros((2, 2, n), dtype=np.float32)
        folded, rho, reps = fold_duplicate_angles(vol, 360.0)
        assert folded.shape[2] == expected_distinct
        assert reps == expected_reps
        assert len(rho) == expected_distinct

    def test_phases_are_evenly_spaced_within_a_half_turn(self):
        vol = np.zeros((2, 2, 36), dtype=np.float32)
        _, rho, _ = fold_duplicate_angles(vol, 360.0)
        deg = np.rad2deg(rho)
        assert deg.min() == pytest.approx(0.0)
        assert deg.max() < 180.0
        assert np.allclose(np.diff(deg), deg[1] - deg[0])

    def test_repeats_are_averaged_not_dropped(self):
        '''Folding must average the duplicate acquisitions -- that is the
        sqrt(2) noise reduction. Dropping one would throw away half the data.'''
        vol = np.zeros((1, 1, 4), dtype=np.float64)
        # phases 0, 90, 180->0, 270->90 : pairs are (0,2) and (1,3)
        vol[0, 0, :] = [1.0, 5.0, 3.0, 9.0]
        folded, _, reps = fold_duplicate_angles(vol, 360.0)
        assert reps == 2
        assert folded[0, 0, 0] == pytest.approx(2.0)   # mean(1, 3)
        assert folded[0, 0, 1] == pytest.approx(7.0)   # mean(5, 9)

    def test_odd_count_passes_through_unchanged(self):
        vol = np.random.default_rng(0).random((3, 3, 9))
        folded, _, reps = fold_duplicate_angles(vol, 360.0)
        assert reps == 1
        assert np.array_equal(folded, vol)

    def test_half_turn_sweep_has_no_repeats(self):
        '''plipy-style data covers 180 degrees, so every image is distinct.'''
        vol = np.zeros((2, 2, 18), dtype=np.float32)
        folded, _, reps = fold_duplicate_angles(vol, 180.0)
        assert reps == 1
        assert folded.shape[2] == 18


class TestFit:

    @pytest.mark.parametrize("n_angles", [9, 18, 36, 72])
    def test_fit_recovers_known_coefficients(self, n_angles):
        a0, a1, b1 = 0.40, 0.03, -0.02
        vol = synth_stack(a0, a1, b1, n_angles)
        fa0, fa1, fb1, rms, _ = fit_coefficients(vol, sweep_deg=360.0)

        assert fa0.mean() == pytest.approx(a0, abs=1e-9)
        assert fa1.mean() == pytest.approx(a1, abs=1e-9)
        assert fb1.mean() == pytest.approx(b1, abs=1e-9)
        assert rms == pytest.approx(0.0, abs=1e-9)

    def test_wrong_sweep_corrupts_the_fit(self):
        '''Guards the bug this module exists to avoid: fitting a 360-degree
        acquisition with plipy's 180-degree basis.'''
        a0, a1, b1 = 0.40, 0.03, -0.02
        vol = synth_stack(a0, a1, b1, n_angles=9, sweep_deg=360.0)

        _, _, _, rms_right, _ = fit_coefficients(vol, sweep_deg=360.0)
        _, ba1, bb1, rms_wrong, _ = fit_coefficients(vol, sweep_deg=180.0)

        assert rms_right < 1e-9
        assert rms_wrong > 1e-4, "wrong basis should not fit cleanly"
        assert abs(ba1.mean() - a1) > 1e-3 or abs(bb1.mean() - b1) > 1e-3

    def test_retardation_of_known_signal(self):
        a0, a1, b1 = 0.5, 0.03, 0.04       # sqrt(a1^2+b1^2) = 0.05
        vol = synth_stack(a0, a1, b1, 18)
        fa0, fa1, fb1, _, _ = fit_coefficients(vol)
        r = retardation_map(fa0, fa1, fb1)
        assert r.mean() == pytest.approx(0.05 / 0.5, abs=1e-6)

    def test_retardation_is_bounded(self):
        '''Unnormalised data can push the ratio above 1; it must clip.'''
        a0 = np.array([[1e-12]])
        a1 = np.array([[1.0]])
        b1 = np.array([[1.0]])
        r = retardation_map(a0, a1, b1)
        assert np.all(np.isfinite(r))
        assert r.max() <= 1.0

    def test_transmittance_is_twice_a0(self):
        vol = synth_stack(0.4, 0.01, 0.01, 18)
        fa0, _, _, _, _ = fit_coefficients(vol)
        assert transmittance_map(fa0).mean() == pytest.approx(0.8, abs=1e-6)

    def test_azimuth_within_half_turn(self):
        rng = np.random.default_rng(1)
        a1 = rng.normal(size=(8, 8))
        b1 = rng.normal(size=(8, 8))
        az = azimuth_map(a1, b1)
        assert az.min() >= 0.0
        assert az.max() <= np.pi

    def test_too_few_phases_is_rejected(self):
        '''Three coefficients need three distinct phases; a 2-angle run folds
        to one and must fail loudly rather than return nonsense.'''
        vol = np.zeros((2, 2, 2), dtype=np.float32)
        with pytest.raises(ValueError, match="at least 3"):
            fit_coefficients(vol, sweep_deg=360.0)


class TestStretch:

    def test_maps_percentiles_to_unit_range(self):
        img = np.linspace(0, 1, 101).reshape(101, 1)
        out = stretch(img, 2, 98)
        assert out.min() == pytest.approx(0.0)
        assert out.max() == pytest.approx(1.0)

    def test_flat_image_does_not_divide_by_zero(self):
        out = stretch(np.full((4, 4), 0.3))
        assert np.all(np.isfinite(out))

    def test_ignores_nan(self):
        img = np.full((4, 4), 0.5)
        img[0, 0] = np.nan
        assert np.all(np.isfinite(stretch(img)))


class TestMosaic:

    def test_tiles_are_placed_by_row_and_col(self):
        tiles = {(r, c): np.full((10, 10), r * 2 + c, dtype=np.float32)
                 for r in range(2) for c in range(2)}
        m = assemble_mosaic(tiles, margin_pct=0)
        assert m.shape == (20, 20)
        assert m[0, 0] == 0 and m[0, 19] == 1
        assert m[19, 0] == 2 and m[19, 19] == 3

    def test_margin_crops_the_overlap(self):
        tiles = {(0, 0): np.zeros((10, 10), dtype=np.float32)}
        assert assemble_mosaic(tiles, margin_pct=0.1).shape == (8, 8)

    def test_missing_tiles_leave_gaps_not_errors(self):
        tiles = {(0, 0): np.zeros((4, 4), dtype=np.float32),
                 (1, 1): np.ones((4, 4), dtype=np.float32)}
        m = assemble_mosaic(tiles, margin_pct=0)
        assert m.shape == (8, 8)
        assert np.isnan(m[0, 4])


class TestFindRois:

    def test_labelled_layout(self, tmp_path):
        for r in range(2):
            for c in range(3):
                (tmp_path / "roi_22" / f"fov_{r}_{c}").mkdir(parents=True)
        rois = find_rois(tmp_path)
        assert set(rois) == {"22"}
        assert len(rois["22"]) == 6
        assert (0, 2) in rois["22"]

    def test_unlabelled_layout(self, tmp_path):
        for c in range(2):
            (tmp_path / f"fov_0_{c}").mkdir(parents=True)
        rois = find_rois(tmp_path)
        assert set(rois) == {""}
        assert len(rois[""]) == 2

    def test_multiple_rois(self, tmp_path):
        for label in ("18", "19"):
            (tmp_path / f"roi_{label}" / "fov_0_0").mkdir(parents=True)
        assert set(find_rois(tmp_path)) == {"18", "19"}

    def test_empty_directory(self, tmp_path):
        assert find_rois(tmp_path) == {}


class TestImageLayout:
    '''Tiles must be placed from the stage motion and the camera orientation,
    not from their fov_row_col names: on xypli_large the camera is rotated
    relative to the stage and index order scrambles the mosaic.'''

    def test_xypli_large_layout_matches_the_measured_overlaps(self):
        # 3 x 4 grid stepped the svg_acquire way (-x per column, -y per row).
        # Measured 2026-09-21: the next row sits to the RIGHT in the camera
        # image, the next column sits ABOVE.
        from pli_preview import load_positions, image_grid, camera_axes_for
        fovs = {(r, c): None for r in range(3) for c in range(4)}
        pos, _ = load_positions("/nonexistent", fovs)
        cell = image_grid(pos, camera_axes_for("xypli_large"))
        rows, cols = 4, 3
        seen = [[None] * cols for _ in range(rows)]
        for k, (ir, ic) in cell.items():
            seen[ir][ic] = k
        assert seen[0] == [(0, 3), (1, 3), (2, 3)]   # top row of the image
        assert seen[3] == [(0, 0), (1, 0), (2, 0)]   # bottom row

    def test_positions_json_is_used_when_present(self, tmp_path):
        import json
        from pli_preview import load_positions
        (tmp_path / "positions.json").write_text(json.dumps(
            {"platform": "xypli_large", "units": "mm",
             "fovs": {"fov_0_0": [25.0, 20.0], "fov_0_1": [21.0, 20.0]}}))
        pos, platform = load_positions(tmp_path, {(0, 0): None, (0, 1): None})
        assert platform == "xypli_large"
        assert pos[(0, 1)] == (21.0, 20.0)

    def test_grid_does_not_depend_on_step_sign(self):
        # Positive-extent ROIs step the other way; the image layout must follow
        # the positions, so the physically left-most tile stays left-most.
        from pli_preview import image_grid
        a = image_grid({(0, 0): (10, 20), (1, 0): (10, 16)}, ("-y", "+x"))
        b = image_grid({(0, 0): (10, 16), (1, 0): (10, 20)}, ("-y", "+x"))
        assert a[(1, 0)] == b[(0, 0)]      # the FOV at y=16 lands in the same cell

    def test_unknown_platform_falls_back_to_index_layout(self):
        from pli_preview import image_grid, load_positions, LEGACY_AXES
        fovs = {(r, c): None for r in range(2) for c in range(3)}
        pos, _ = load_positions("/nonexistent", fovs)
        cell = image_grid(pos, LEGACY_AXES)
        assert all(cell[k] == k for k in fovs)


class TestTileNormalisation:
    '''Alternate acquisitions carry a different background vector and ~40 %
    less modulation (report 7 §4). The preview's per-tile correction must
    make identical tissue look identical across such tiles.'''

    def test_offset_and_gain_differences_are_removed(self):
        from pli_preview import normalise_tiles
        rng = np.random.default_rng(0)
        tissue = np.zeros((40, 40)); tissue[15:25, 15:25] = 0.08   # a patch
        tiles = {}
        # Same physical background on both tiles, but tile (0, 1) sees the
        # whole polarisation signal at 0.6 contrast -- as measured (0.57-0.65).
        for cell, (bg_u, bg_v, gain) in {(0, 0): (0.05, 0.01, 1.0),
                                         (0, 1): (0.05, 0.01, 0.6)}.items():
            u = gain * (bg_u + tissue) + rng.normal(0, 1e-3, tissue.shape)
            v = gain * bg_v + rng.normal(0, 1e-3, tissue.shape)
            tiles[cell] = (u, v)
        out = normalise_tiles(tiles)
        a, b = out[(0, 0)], out[(0, 1)]
        # empty glass ~0 in both, and the patch equally bright in both
        assert np.median(a[:10]) < 5e-3 and np.median(b[:10]) < 5e-3
        assert np.median(b[15:25, 15:25]) == pytest.approx(
            np.median(a[15:25, 15:25]), rel=0.15)
