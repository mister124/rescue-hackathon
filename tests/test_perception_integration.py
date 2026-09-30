"""실제 C 코드의 D 연결 검사. YOLO 추론만 대체하며 실제 모델 정확도 검사는 아님."""
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'rescue'))
import config
import perception as c
import rescue as r


class Array:
    def __init__(self, values): self.values = np.asarray(values)
    def cpu(self): return self
    def numpy(self): return self.values


class Camera:
    def getWidth(self): return 100
    def getHeight(self): return 100
    def getFov(self): return np.pi / 2


class PerceptionIntegrationTests(unittest.TestCase):
    def test_c_detection_localization_and_confirmation_interface(self):
        result = types.SimpleNamespace(
            boxes=types.SimpleNamespace(xyxy=Array([[40, 55, 60, 70]]), cls=Array([47])),
            plot=lambda: np.zeros((100, 100, 3), dtype=np.uint8))
        model = types.SimpleNamespace(names={47: 'apple'}, to=lambda device: None,
                                      predict=lambda **kwargs: [result])
        with patch.dict(sys.modules, ultralytics=types.SimpleNamespace(YOLO=lambda path: model)):
            detector = c.Perception(Camera())
        red = np.zeros((100, 100, 3), np.uint8)
        red[:, :, 2] = 255
        dets = detector.detect(red)
        self.assertEqual(len(dets), 1)
        self.assertEqual((dets[0]['cls'], dets[0]['color']), ('apple', 'red'))
        xy = c.localize(dets[0], np.full(360, np.inf),
                        np.linspace(-np.pi, np.pi, 360, endpoint=False), (0, 0, 0), 3.5)
        self.assertIsNotNone(xy)
        book = c.TargetBook()
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(config.CONFIRM_N): book.add(*xy, dets[0]['cls'])
        self.assertEqual(len(book.confirmed()), 1)
        self.assertFalse(book.confirmed()[0]['visited'])
        self.assertEqual(c.Perception.draw(red, dets).shape, red.shape)

    def test_c_requires_installation_message_and_model_path_is_absolute(self):
        with patch.dict(sys.modules, ultralytics=None):
            with self.assertRaises(ImportError): c.Perception(Camera())
        self.assertEqual(config.DETECTOR, 'yolo')
        self.assertTrue(Path(config.YOLO_MODEL).is_absolute())
        self.assertTrue(config.GOTO_GIVEUP_S > config.STUCK_S + config.BACKUP_S)


if __name__ == '__main__': unittest.main()
