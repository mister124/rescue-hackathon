"""Webots 장치/검출만 대체한 main loop 테스트. 실제 시뮬레이터 테스트와 구별."""
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'rescue'))
import rescue as r
from slam import Slam


class Device:
    def __init__(self, name):
        self.name, self.velocities = name, []
    def getName(self): return self.name
    def enable(self, ts): pass
    def setPosition(self, x): pass
    def setVelocity(self, x): self.velocities.append(x)
    def getMaxVelocity(self): return 4.0
    def getValue(self): return 0.0
    def getHorizontalResolution(self): return 8
    def getFov(self): return 2*np.pi
    def getMaxRange(self): return 3.5
    def getNumberOfLayers(self): return 1
    def getRangeImage(self): return [np.inf]*8
    def getValues(self): return [0, 0, 0]


class Robot:
    def __init__(self, script, steps=7):
        names = (r.LEFT_MOTOR, r.RIGHT_MOTOR, r.LEFT_ENC, r.RIGHT_ENC, r.LIDAR_NAME, r.CAMERA_NAME, r.GYRO_NAME)
        self.devices = [Device(n) for n in names]
        self.script, self.steps, self.index, self.queue = script, steps, 0, []
    def getBasicTimeStep(self): return 1000
    def getTime(self): return 1000.0 + self.index
    def getNumberOfDevices(self): return len(self.devices)
    def getDeviceByIndex(self, i): return self.devices[i]
    def getKeyboard(self): return self
    def enable(self, ts): pass
    def getKey(self): return self.queue.pop(0) if self.queue else -1
    def step(self, ts):
        self.index += 1
        self.queue = list(self.script.get(self.index, []))
        return 0 if self.index <= self.steps else -1


class Book:
    def confirmed(self): return []
    def add(self, *args): pass


class Perception:
    def __init__(self, camera): pass
    def grab(self): return np.zeros((2, 2, 3), dtype=np.uint8)
    def detect(self, frame): return []


class ControllerTests(unittest.TestCase):
    def run_controller(self, robot, *, sensor_error=False):
        controller = types.SimpleNamespace(Robot=lambda: robot, Supervisor=lambda: robot)
        perception = types.SimpleNamespace(Perception=Perception, TargetBook=Book, localize=lambda *args: None)
        opencv = types.SimpleNamespace(error=RuntimeError)
        # 시작점에서 2m 떨어진 상태를 주입해 강제 복귀 이후 TIMEOUT까지 확인.
        slam = Slam(2, 0, 0)
        if sensor_error:
            robot.devices[2].getValue = lambda: np.nan
        records = []
        with contextlib.redirect_stdout(io.StringIO()), \
             patch.dict(sys.modules, controller=controller, perception=perception, cv2=opencv), \
             patch.multiple(r, TIME_LIMIT_S=5.0, RETURN_MARGIN_S=2.0, SHOW_WINDOWS=False, DETECT_EVERY=1), \
             patch.object(r, 'Slam', return_value=slam), \
             patch.object(r.nav, 'plan', return_value=[]), \
             patch.object(r.nav, 'pick_frontier', return_value=(None, [])), \
             patch.object(r, 'save_result', side_effect=lambda m,t,p: records.append(m.result(t,p))):
            if sensor_error:
                with self.assertRaisesRegex(RuntimeError, '엔코더'):
                    r.main()
            else:
                r.main()
        return records

    def test_manual_overridden_at_return_and_timeout_uses_elapsed(self):
        robot = Robot({1: [ord('M'), ord('W')], 2: [ord('W')], 3: [ord('M'), ord('W')]})
        records = self.run_controller(robot)
        self.assertIn(r.TELEOP_SPEED, robot.devices[0].velocities)
        self.assertEqual(records[-1]['state'], 'TIMEOUT')
        self.assertEqual(records[-1]['elapsed_s'], 5.0)
        self.assertEqual(robot.devices[0].velocities[-1], 0)
        # t=3부터 복귀 경로 없음 -> 정지. M+W도 재수동 전환 불가.
        self.assertEqual(robot.devices[0].velocities[3], 0)

    def test_emergency_stop_remains_locked_until_p(self):
        robot = Robot({1: [ord('M'), ord('W'), ord(' ')], 2: [ord('W')]}, steps=2)
        self.run_controller(robot)
        self.assertTrue(all(v == 0 for v in robot.devices[0].velocities))
        robot = Robot({1: [ord('M'), ord(' ')], 2: [ord('P'), ord('W')]}, steps=2)
        self.run_controller(robot)
        self.assertIn(r.TELEOP_SPEED, robot.devices[0].velocities)
        self.assertEqual(robot.devices[0].velocities[-1], 0)

    def test_sensor_error_stops_both_motors_and_records_failure(self):
        robot = Robot({})
        records = self.run_controller(robot, sensor_error=True)
        self.assertEqual(records[-1]['state'], 'ERROR')
        self.assertFalse(records[-1]['success'])
        self.assertEqual([d.velocities[-1] for d in robot.devices[:2]], [0, 0])

    def test_missing_camera_stops_initialised_motors(self):
        robot = Robot({})
        robot.devices = [d for d in robot.devices if d.name != r.CAMERA_NAME]
        with self.assertRaisesRegex(RuntimeError, '카메라'):
            self.run_controller(robot)
        self.assertEqual([d.velocities[-1] for d in robot.devices[:2]], [0, 0])


if __name__ == '__main__':
    unittest.main()
