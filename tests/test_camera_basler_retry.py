'''Tests for camera_basler.Camera.grab_image()'s retry-on-failed-grab logic.

pypylon is not available/needed for this test: we install a fake `pypylon`
module (with a fake `pylon` submodule) into sys.modules before importing
camera_basler, so the module under test can be exercised without any Basler
hardware or the real pypylon package.
'''

import sys
import types
from pathlib import Path

import numpy as np
import pytest

PLI_GET_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PLI_GET_DIR))


class FakeGenericException(Exception):
    '''Stand-in for pypylon.pylon.GenericException.'''


class FakeGrabResult:
    '''A grab result that can be made to succeed or fail.

    On failure, records whether Release() was called and exposes a
    GetErrorCode/GetErrorDescription pair like a real pylon.GrabResult.
    '''
    def __init__(self, succeeded, array=None, error_code=0xE1000014,
                 error_desc="The buffer was incompletely grabbed."):
        self._succeeded = succeeded
        self._array = array
        self._error_code = error_code
        self._error_desc = error_desc
        self.released = False

    def GrabSucceeded(self):
        return self._succeeded

    def GetArray(self):
        return self._array

    def GetErrorCode(self):
        return self._error_code

    def GetErrorDescription(self):
        return self._error_desc

    def Release(self):
        self.released = True


def install_fake_pypylon():
    '''Insert a fake `pypylon` package (with `pylon` submodule) into
    sys.modules and return the fake `pylon` module, so camera_basler can be
    imported (and re-imported with a fresh fake) without real hardware.'''
    fake_pylon = types.ModuleType("pypylon.pylon")
    fake_pylon.GenericException = FakeGenericException
    fake_pylon.GrabStrategy_LatestImageOnly = object()
    fake_pylon.TimeoutHandling_ThrowException = object()

    class FakeTlFactory:
        @staticmethod
        def GetInstance():
            return FakeTlFactory()

        def CreateFirstDevice(self):
            return object()

    class FakeInstantCamera:
        '''Not used directly in these tests (we patch a bare object's
        .camera / .GrabOne instead), but provided so pylon.InstantCamera(...)
        would not blow up if some code path constructs one.'''
        def __init__(self, *args, **kwargs):
            pass

    fake_pylon.TlFactory = FakeTlFactory
    fake_pylon.InstantCamera = FakeInstantCamera

    fake_pypylon = types.ModuleType("pypylon")
    fake_pypylon.pylon = fake_pylon

    sys.modules["pypylon"] = fake_pypylon
    sys.modules["pypylon.pylon"] = fake_pylon
    return fake_pylon


@pytest.fixture
def camera_basler_module():
    '''Import (or re-import) camera_basler with a fresh fake pypylon.pylon
    installed, so each test sees a clean module and fake pylon.'''
    fake_pylon = install_fake_pypylon()
    sys.modules.pop("camera_basler", None)
    import camera_basler
    return camera_basler, fake_pylon


class FakeBaslerCamera:
    '''Stands in for camera_basler.Camera's underlying pylon InstantCamera
    for the purposes of grab_image(): IsOpen()/Open() bookkeeping plus a
    scripted sequence of GrabOne() outcomes.'''
    def __init__(self, outcomes):
        '''outcomes: list of either a FakeGrabResult or an Exception
        instance/class, consumed one per GrabOne() call.'''
        self._outcomes = list(outcomes)
        self._is_open = True
        self.grab_calls = 0

    def IsOpen(self):
        return self._is_open

    def Open(self):
        self._is_open = True

    def GrabOne(self, timeout_ms):
        self.grab_calls += 1
        outcome = self._outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        if isinstance(outcome, type) and issubclass(outcome, Exception):
            raise outcome("simulated pylon exception")
        return outcome


def make_camera(camera_basler_module, outcomes):
    '''Build a Camera instance (bypassing __init__/hardware discovery) with
    a FakeBaslerCamera wired in as .camera.'''
    camera_basler, _ = camera_basler_module
    cam = camera_basler.Camera.__new__(camera_basler.Camera)
    cam.camera = FakeBaslerCamera(outcomes)
    return cam


def test_grab_succeeds_first_try(camera_basler_module):
    array = np.arange(9, dtype=np.uint8).reshape(3, 3)
    cam = make_camera(camera_basler_module, [FakeGrabResult(True, array=array)])

    result = cam.grab_image()

    np.testing.assert_array_equal(result, array)
    assert cam.camera.grab_calls == 1


def test_grab_returns_a_copy_not_a_view(camera_basler_module):
    '''Regression test: the returned array must survive the underlying
    buffer being mutated/released after grab_image() returns.'''
    array = np.arange(9, dtype=np.uint8).reshape(3, 3)
    grab_result = FakeGrabResult(True, array=array)
    cam = make_camera(camera_basler_module, [grab_result])

    result = cam.grab_image()
    assert grab_result.released is True

    # Mutate the "buffer" after Release(): a view would reflect this change,
    # a copy would not.
    array[:] = 255
    assert not np.array_equal(result, array)


def test_grab_retries_on_failed_grab_then_succeeds(camera_basler_module, monkeypatch):
    camera_basler, _ = camera_basler_module
    monkeypatch.setattr(camera_basler.time, "sleep", lambda s: None)

    array = np.ones((2, 2), dtype=np.uint8)
    outcomes = [
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer"),
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer"),
        FakeGrabResult(True, array=array),
    ]
    cam = make_camera(camera_basler_module, outcomes)

    result = cam.grab_image()

    np.testing.assert_array_equal(result, array)
    assert cam.camera.grab_calls == 3
    for outcome in outcomes[:2]:
        assert outcome.released is True


def test_grab_retries_on_exception_then_succeeds(camera_basler_module, monkeypatch):
    camera_basler, fake_pylon = camera_basler_module
    monkeypatch.setattr(camera_basler.time, "sleep", lambda s: None)

    array = np.ones((2, 2), dtype=np.uint8)
    outcomes = [
        fake_pylon.GenericException("timeout waiting for frame"),
        FakeGrabResult(True, array=array),
    ]
    cam = make_camera(camera_basler_module, outcomes)

    result = cam.grab_image()

    np.testing.assert_array_equal(result, array)
    assert cam.camera.grab_calls == 2


def test_grab_raises_after_exhausting_attempts(camera_basler_module, monkeypatch):
    camera_basler, _ = camera_basler_module
    monkeypatch.setattr(camera_basler.time, "sleep", lambda s: None)

    outcomes = [
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer #1"),
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer #2"),
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer #3"),
        FakeGrabResult(False, error_code=0xE1000014, error_desc="Incomplete buffer #4"),
    ]
    cam = make_camera(camera_basler_module, outcomes)

    with pytest.raises(RuntimeError, match="Incomplete buffer #4"):
        cam.grab_image()

    assert cam.camera.grab_calls == 4
    for outcome in outcomes:
        assert outcome.released is True
