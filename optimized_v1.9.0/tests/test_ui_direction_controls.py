import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from PyQt5.QtWidgets import QApplication

from main import NMEADataAnalyzer
from src.simulation.selftest import SelfTestRunner


class UIDirectionControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.window = NMEADataAnalyzer()

    @classmethod
    def tearDownClass(cls):
        cls.window.close()
        cls.window.deleteLater()
        cls.app.processEvents()

    def test_serial_stream_has_its_own_main_tab(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        self.assertIn("串口实时数据流", titles)
        stream_tab = self.window.tab_widget.widget(titles.index("串口实时数据流"))
        self.assertTrue(stream_tab.isAncestorOf(self.window.data_preview))

    def test_global_action_area_uses_readable_fonts(self):
        def rendered_size(widget):
            font = widget.font()
            return font.pointSizeF() if font.pointSizeF() > 0 else font.pixelSize()

        self.assertGreaterEqual(rendered_size(self.window.action_box), 14)
        for control in self.window.action_controls:
            self.assertGreaterEqual(rendered_size(control), 14)
            self.assertGreaterEqual(control.minimumHeight(), 32)

    def test_every_direction_has_four_independent_controls(self):
        expected_keys = {"start_btn", "pause_btn", "resume_btn", "terminate_btn"}
        for boxes in (self.window.dir_stat_boxes, self.window.dir_stat_boxes3):
            self.assertEqual(len(boxes), 4)
            for controls in boxes:
                self.assertTrue(expected_keys.issubset(controls))
                self.assertGreaterEqual(controls["box"].minimumHeight(), 208)
                self.assertTrue(controls["start_btn"].isEnabled())
                self.assertFalse(controls["pause_btn"].isEnabled())
                self.assertFalse(controls["resume_btn"].isEnabled())
                self.assertFalse(controls["terminate_btn"].isEnabled())

    def test_direction_buttons_follow_state_and_starting_another_pauses_current(self):
        self.window._start_direction_test(2, 0)
        first = self.window.dir_stat_boxes[0]
        self.assertTrue(self.window.direction_stats[0].is_active)
        self.assertFalse(first["start_btn"].isEnabled())
        self.assertTrue(first["pause_btn"].isEnabled())
        self.assertTrue(first["terminate_btn"].isEnabled())

        self.window._start_direction_test(3, 0)
        self.assertTrue(self.window.direction_stats[0].is_active)
        self.assertTrue(self.window.direction_stats3[0].is_active)

        self.window._start_direction_test(2, 1)
        self.assertFalse(self.window.direction_stats[0].is_active)
        self.assertTrue(first["resume_btn"].isEnabled())
        self.assertTrue(self.window.direction_stats[1].is_active)

        self.window._terminate_direction_test(2, 1)
        second = self.window.dir_stat_boxes[1]
        self.assertTrue(self.window.direction_stats[1].is_terminated)
        self.assertFalse(any(second[key].isEnabled() for key in (
            "start_btn", "pause_btn", "resume_btn", "terminate_btn"
        )))

        self.window._reset_direction_stats()
        self.window._reset_direction_stats3()
        for stats in (self.window.direction_stats, self.window.direction_stats3):
            for direction in stats:
                self.assertEqual(direction.state, "idle")
                self.assertEqual(direction.total_epochs, 0)
                self.assertEqual(direction.duration, 0.0)

    def test_gui_selftest_starts_direction_with_new_controls(self):
        class SimProgress:
            @staticmethod
            def progress():
                return 0.5

            @staticmethod
            def sim_time():
                return 151.0

        runner = SelfTestRunner(self.window, headless=True)
        runner._running = True
        runner._direction = True
        runner._sims = [SimProgress(), SimProgress(), SimProgress()]
        runner._wall0 = time.monotonic()
        runner._minutes = 5
        runner._speed = 100
        old_ref_ready = self.window.enu2_ref_ready
        self.window.enu2_ref_ready = True
        try:
            runner._on_monitor()
            self.assertTrue(runner._dir_started)
            self.assertTrue(self.window.direction_stats[0].is_active)
        finally:
            self.window.enu2_ref_ready = old_ref_ready
            self.window._reset_direction_stats()


if __name__ == "__main__":
    unittest.main()
