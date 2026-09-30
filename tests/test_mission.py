"""requirements.txt 설치 후 실행: python -m unittest discover -s tests -v"""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'rescue'))
import rescue as r
from slam import Slam


class Book:
    def __init__(self, items=()):
        self.items = list(items)

    def confirmed(self):
        return self.items


def target(x=2.0, visited=False):
    return dict(x=x, y=0.0, cls='green', visited=visited)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.ranges = np.full(8, np.inf)
        self.angles = np.linspace(-np.pi, np.pi, 8, endpoint=False)
        self.grid = Slam(0, 0, 0).grid
        self.grid.lo[:] = -1
        self.planner = patch.object(r.nav, 'plan', side_effect=lambda grid, blocks, pose, goal: [(pose[0], pose[1]), goal])
        self.planner.start()
        self.frontier = patch.object(r.nav, 'pick_frontier', return_value=((3, 0), [(0, 0), (3, 0)]))
        self.frontier.start()
        self.follower = patch.object(r.nav.Follower, 'step', return_value=(0.1, 0.0, False))
        self.follower.start()
        self.addCleanup(self.output.__exit__, None, None, None)
        self.addCleanup(patch.stopall)

    def mission(self, items=()):
        return r.Mission(self.grid, Book(items))

    def step(self, m, elapsed, pose=(0, 0, 0), **kwargs):
        return m.step(elapsed, pose, self.ranges, self.angles, **kwargs)

    def test_b_backup_continues_without_path_and_then_replans(self):
        self.follower.stop()  # 실제 B Follower의 후진 연결 확인
        m = self.mission()
        m.state = 'EXPLORE'
        m.follower.backup_until = 4.0
        with patch.object(r.nav, 'make_blocks') as blocks:
            self.assertEqual(self.step(m, 2), (r.nav.BACKUP_V, 0.0))
            blocks.assert_not_called()
            self.assertEqual(self.step(m, 4), (0.0, 0.0))
        self.assertIsNone(m.follower.backup_until)
        self.assertEqual(m.last_plan, -float('inf'))
        self.follower.start()
        self.step(m, 4.1)
        self.assertTrue(m.path)

    def test_warmup_stops(self):
        m = self.mission()
        self.assertEqual(self.step(m, 0), (0, 0))
        self.assertEqual(m.state, 'WARMUP')
        self.step(m, r.WARMUP_S)
        self.assertEqual(m.state, 'EXPLORE')

    def test_manual_still_forces_return_at_boundary(self):
        m = self.mission()
        self.step(m, r.TIME_LIMIT_S-r.RETURN_MARGIN_S, (2, 0, 0), autonomous=False)
        self.assertEqual(m.state, 'RETURN')
        self.assertFalse(m.path)

    def test_timeout_stops_manual_and_is_latched(self):
        m = self.mission()
        self.assertEqual(self.step(m, r.TIME_LIMIT_S, autonomous=False), (0, 0))
        self.assertEqual(m.state, 'TIMEOUT')
        m.request_return('R key', r.TIME_LIMIT_S+1)
        self.assertEqual(self.step(m, r.TIME_LIMIT_S+2), (0, 0))
        self.assertEqual(m.state, 'TIMEOUT')

    def test_visit_then_return_success(self):
        t = target()
        m = self.mission([t])
        self.step(m, 1)
        self.assertEqual(m.state, 'GOTO')
        self.step(m, 2, (1.5, 0, 0))
        self.assertTrue(t['visited'])
        self.assertEqual(m.state, 'RETURN')
        self.step(m, 3)
        self.assertEqual(m.state, 'DONE')
        self.assertTrue(m.result(4, (0, 0, 0))['success'])
        self.assertEqual(m.result(4, (0, 0, 0))['elapsed_s'], 3)

    def test_return_without_targets_is_incomplete(self):
        m = self.mission()
        m.request_return('no frontier', 1)
        self.step(m, 2)
        self.assertEqual(m.state, 'DONE')
        self.assertFalse(m.result(2, (0, 0, 0))['success'])

    def test_unreachable_target_never_becomes_visited(self):
        t = target()
        m = self.mission([t])
        with patch.object(r.nav, 'plan', return_value=[]):
            self.step(m, 1)
            self.step(m, 1+r.GOTO_GIVEUP_S)
        self.assertFalse(t['visited'])
        self.assertEqual(m.state, 'EXPLORE')
        self.assertEqual(m.visited_count(), 0)
        self.step(m, 2+r.GOTO_GIVEUP_S)
        self.assertEqual(m.state, 'EXPLORE')

    def test_existing_path_without_progress_also_gives_up(self):
        t = target()
        m = self.mission([t])
        self.step(m, 1)
        self.step(m, 1+r.GOTO_GIVEUP_S)
        self.assertEqual(m.state, 'EXPLORE')
        self.assertFalse(t['visited'])

    def test_retry_limit(self):
        t = target()
        m = self.mission([t])
        m.attempts[id(t)] = r.TARGET_MAX_ATTEMPTS
        self.step(m, 1)
        self.assertEqual(m.state, 'EXPLORE')

    def test_nearest_target_selected(self):
        a, b = target(4), target(2)
        m = self.mission([a, b])
        self.step(m, 1)
        self.assertIs(m.target, b)

    def test_frontier_exhaustion_requests_return(self):
        m = self.mission()
        with patch.object(r.nav, 'pick_frontier', return_value=(None, [])):
            for i in range(r.EXPLORE_FAIL_LIMIT):
                self.step(m, 1+i, (1, 0, 0))
        self.assertEqual(m.state, 'RETURN')

    def test_return_no_path_stops_instead_of_spinning(self):
        m = self.mission()
        m.request_return('deadline', 1)
        with patch.object(r.nav, 'plan', return_value=[]):
            self.assertEqual(self.step(m, 2, (2, 0, 0)), (0, 0))
        self.assertEqual(m.state, 'RETURN')
        self.assertEqual(self.step(m, r.TIME_LIMIT_S, (2, 0, 0)), (0, 0))
        self.assertEqual(m.state, 'TIMEOUT')

    def test_stuck_stops_and_blacklists_frontier(self):
        m = self.mission()
        with patch.object(r.nav.Follower, 'step', return_value=(0.1, 0.2, True)):
            self.assertEqual(self.step(m, 1), (0, 0))
        self.assertIn((3, 0), m.blacklist)
        self.assertFalse(m.path)

    def test_manual_resume_discards_stale_path(self):
        m = self.mission()
        self.step(m, 1)
        old = m.follower
        m.resume(10)
        self.assertFalse(m.path)
        self.assertIsNone(m.goal)
        self.assertIsNot(m.follower, old)

    def test_real_nav_slam_interface_smoke(self):
        # Mock 없이 실제 A/B의 공개 함수 연결 확인; 물리/충돌 검증은 아님.
        patch.stopall()
        s = Slam(0, 0, 0)
        s.update(0, 0, None, None, self.ranges, self.angles, 3.5, 0.032)
        s.update(0.1, 0.1, None, None, self.ranges, self.angles, 3.5, 0.032)
        m = r.Mission(s.grid, Book())
        m.request_return('test', 1)
        v, w = m.step(2, (1, 0, 0), self.ranges, self.angles)
        self.assertTrue(np.isfinite([v, w]).all())
        self.assertEqual(m.state, 'RETURN')

    def test_lidar_inf_kept_and_invalid_beams_ignored(self):
        result = r.clean_ranges([np.inf, 0, -1, np.nan, 1], 5)
        self.assertTrue(np.isposinf(result[0]))
        self.assertTrue(np.isnan(result[1:4]).all())
        with self.assertRaises(RuntimeError):
            r.clean_ranges([0, np.nan], 2)
        with self.assertRaises(RuntimeError):
            r.clean_ranges([1], 2)
        self.assertEqual(r.lidar_angles(1, 1).tolist(), [0])

    def test_motor_ratio_and_hardware_limits(self):
        left, right = r.limited_wheels(9, -6, 3, 4)
        self.assertEqual((left, right), (3, -2))
        with self.assertRaises(ValueError):
            r.limited_wheels(np.nan, 0, 3, 3)

    def test_default_configuration(self):
        r.validate_config()
        with patch.object(r, 'RETURN_MARGIN_S', r.TIME_LIMIT_S):
            with self.assertRaises(ValueError):
                r.validate_config()


if __name__ == '__main__':
    unittest.main()
