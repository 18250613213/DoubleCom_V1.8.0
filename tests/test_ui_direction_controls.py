import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from PyQt5.QtCore import QPoint
from PyQt5.QtWidgets import QApplication, QTableWidget

from main import APP_VERSION, NMEADataAnalyzer, SlidingWindowStd
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

    def test_release_version_is_v1_9_4_1(self):
        self.assertEqual(APP_VERSION, "1.9.4.1")

    def test_serial_stream_has_its_own_main_tab(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        self.assertIn("串口实时数据流", titles)
        stream_tab = self.window.tab_widget.widget(titles.index("串口实时数据流"))
        self.assertTrue(stream_tab.isAncestorOf(self.window.data_preview))

    def test_system_log_has_its_own_readable_main_tab(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        self.assertIn("系统运行日志", titles)
        log_tab = self.window.tab_widget.widget(titles.index("系统运行日志"))
        control_tab = self.window.tab_widget.widget(titles.index("功能与控制面板"))
        self.assertTrue(log_tab.isAncestorOf(self.window.log_window))
        self.assertFalse(control_tab.isAncestorOf(self.window.log_window))
        self.assertGreaterEqual(self.window.log_window.font().pixelSize(), 14)
        self.assertGreaterEqual(self.window.log_window.minimumHeight(), 400)

    def test_test_information_reports_and_selftest_share_a_separate_tab(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        self.assertIn("测试信息与报告", titles)
        report_tab = self.window.tab_widget.widget(titles.index("测试信息与报告"))
        control_tab = self.window.tab_widget.widget(titles.index("功能与控制面板"))
        for widget in (
            self.window.test_location_input,
            self.window.dut_model_input,
            self.window.export_report2_btn,
            self.window.export_pdf2_btn,
            self.window.export_report3_btn,
            self.window.export_pdf3_btn,
            self.window.selftest_btn,
        ):
            self.assertTrue(report_tab.isAncestorOf(widget))
            self.assertFalse(control_tab.isAncestorOf(widget))

        self.window.tab_widget.setCurrentWidget(report_tab)
        self.app.processEvents()
        selftest_pos = self.window.selftest_btn.mapTo(
            self.window.test_report_scroll.viewport(), QPoint(0, 0))
        self.assertLessEqual(
            selftest_pos.x() + self.window.selftest_btn.width(),
            self.window.test_report_scroll.viewport().width(),
        )

    def test_direction_page_has_only_per_direction_statistics_without_summary_tables(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        direction_tab = self.window.tab_widget.widget(titles.index("方向测试"))
        self.assertEqual(direction_tab.findChildren(QTableWidget), [])

    def test_per_direction_statistics_use_readable_fonts(self):
        for boxes in (self.window.dir_stat_boxes, self.window.dir_stat_boxes3):
            for controls in boxes:
                box_font = controls["box"].font()
                box_size = box_font.pointSizeF() if box_font.pointSizeF() > 0 else box_font.pixelSize()
                self.assertGreaterEqual(box_size, 16)
                for key in (
                    "duration", "epoch", "h", "v", "state",
                    "start_btn", "pause_btn", "resume_btn", "terminate_btn",
                ):
                    font = controls[key].font()
                    rendered_size = font.pointSizeF() if font.pointSizeF() > 0 else font.pixelSize()
                    self.assertGreaterEqual(rendered_size, 14)

    def test_serial_stream_uses_larger_readable_fonts(self):
        for widget in (
            self.window.preview_box,
            self.window.pause_preview_cb,
            self.window.preview_filter_combo,
            self.window.clear_preview_btn,
            self.window.data_preview,
        ):
            font = widget.font()
            rendered_size = font.pointSizeF() if font.pointSizeF() > 0 else font.pixelSize()
            self.assertGreaterEqual(rendered_size, 15)

    def test_global_action_area_uses_readable_fonts(self):
        def rendered_size(widget):
            font = widget.font()
            return font.pointSizeF() if font.pointSizeF() > 0 else font.pixelSize()

        self.assertGreaterEqual(rendered_size(self.window.action_box), 14)
        for control in self.window.action_controls:
            self.assertGreaterEqual(rendered_size(control), 14)
            self.assertGreaterEqual(control.minimumHeight(), 32)

    def test_data_and_log_buttons_have_no_icons_and_use_larger_fonts(self):
        buttons = (
            self.window.clear_data_btn,
            self.window.reset_stats_btn,
            self.window.clear_log_btn,
            self.window.save_all_log_btn,
        )
        self.assertEqual(
            [button.text() for button in buttons],
            ["清空数据", "重置统计", "清除日志", "保存全部日志"],
        )
        for button in buttons:
            font = button.font()
            rendered_size = font.pointSizeF() if font.pointSizeF() > 0 else font.pixelSize()
            self.assertGreaterEqual(rendered_size, 16)
            self.assertGreaterEqual(button.minimumHeight(), 40)

    def test_enu_outlier_filter_uses_higher_dual_threshold(self):
        unit_sigma = SlidingWindowStd(200)
        half_sigma = SlidingWindowStd(200)
        for index in range(30):
            unit_sigma.add(-1.0 if index % 2 == 0 else 1.0)
            half_sigma.add(-0.5 if index % 2 == 0 else 0.5)

        self.assertFalse(self.window._is_enu_outlier(unit_sigma, 9.5))
        self.assertTrue(self.window._is_enu_outlier(unit_sigma, 10.5))
        self.assertFalse(self.window._is_enu_outlier(half_sigma, 7.9))
        self.assertFalse(self.window._is_enu_outlier(half_sigma, 8.0))
        self.assertTrue(self.window._is_enu_outlier(half_sigma, 8.1))

    def test_plot_refresh_uses_a_separate_one_second_timer(self):
        self.assertEqual(self.window.update_timer.interval(), 250)
        self.assertEqual(self.window.plot_timer.interval(), 1000)
        self.assertIsNot(self.window.plot_timer, self.window.update_timer)

    def test_connecting_starts_both_runtime_and_plot_timers(self):
        manager = self.window.serial_port1
        original_connect = manager.connect
        self.window.update_timer.stop()
        self.window.plot_timer.stop()
        self.window.port1_combo.addItem("__TEST_PORT__")
        self.window.port1_combo.setCurrentText("__TEST_PORT__")
        manager.connect = lambda _port, _baud: True
        try:
            self.window.connect_serial(1)
            self.assertTrue(self.window.update_timer.isActive())
            self.assertTrue(self.window.plot_timer.isActive())
        finally:
            self.window.update_timer.stop()
            self.window.plot_timer.stop()
            manager.connect = original_connect
            index = self.window.port1_combo.findText("__TEST_PORT__")
            if index >= 0:
                self.window.port1_combo.removeItem(index)

    def test_live_enu_chart_refreshes_only_when_visible_and_dirty(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        control_tab = self.window.tab_widget.widget(titles.index("功能与控制面板"))
        self.window.tab_widget.setCurrentWidget(control_tab)
        self.app.processEvents()

        calls = []
        original_refresh = self.window._refresh_enu_charts
        self.window._refresh_enu_charts = lambda: calls.append("refresh")
        try:
            self.window._enu_plot_dirty = True
            self.window._refresh_visible_plots()
            self.assertEqual(calls, [])

            self.window.tab_widget.setCurrentWidget(self.window.enu_comp_tab)
            self.app.processEvents()
            self.assertEqual(calls, ["refresh"])

            self.window._refresh_visible_plots()
            self.assertEqual(calls, ["refresh"])
        finally:
            self.window._refresh_enu_charts = original_refresh
            self.window.tab_widget.setCurrentWidget(control_tab)

    def test_switching_to_clean_plot_tab_does_not_redraw(self):
        titles = [
            self.window.tab_widget.tabText(i)
            for i in range(self.window.tab_widget.count())
        ]
        control_tab = self.window.tab_widget.widget(titles.index("功能与控制面板"))
        self.window.tab_widget.setCurrentWidget(control_tab)
        self.app.processEvents()

        calls = []
        original_refresh = self.window._refresh_enu_charts
        self.window._refresh_enu_charts = lambda: calls.append("refresh")
        try:
            self.window._enu_plot_dirty = False
            self.window.tab_widget.setCurrentWidget(self.window.enu_comp_tab)
            self.app.processEvents()
            self.assertEqual(calls, [])
        finally:
            self.window._refresh_enu_charts = original_refresh
            self.window.tab_widget.setCurrentWidget(control_tab)

    def test_quarter_second_runtime_update_does_not_redraw_enu_curves(self):
        calls = []
        original_refresh = self.window._refresh_enu_charts
        self.window._refresh_enu_charts = lambda: calls.append("refresh")
        try:
            self.window.update_plots()
            self.assertEqual(calls, [])
        finally:
            self.window._refresh_enu_charts = original_refresh

    def test_quarter_second_runtime_update_does_not_redraw_direction_charts(self):
        calls = []
        original_summary2 = self.window._update_direction_charts
        original_summary3 = self.window._update_direction_charts3
        original_enu = self.window._update_direction_enu_charts
        self.window._update_direction_charts = lambda: calls.append("summary2")
        self.window._update_direction_charts3 = lambda: calls.append("summary3")
        self.window._update_direction_enu_charts = lambda: calls.append("enu")
        try:
            self.window.update_plots()
            self.assertEqual(calls, [])
        finally:
            self.window._update_direction_charts = original_summary2
            self.window._update_direction_charts3 = original_summary3
            self.window._update_direction_enu_charts = original_enu

    def test_direction_charts_refresh_only_on_their_visible_page(self):
        calls = []
        original_summary2 = self.window._update_direction_charts
        original_summary3 = self.window._update_direction_charts3
        original_enu = self.window._update_direction_enu_charts
        self.window._update_direction_charts = lambda: calls.append("summary2")
        self.window._update_direction_charts3 = lambda: calls.append("summary3")
        self.window._update_direction_enu_charts = lambda: calls.append("enu")
        try:
            self.window._direction_summary_dirty = True
            self.window.tab_widget.setCurrentWidget(self.window.direction_tab)
            self.app.processEvents()
            self.assertEqual(calls, ["summary2", "summary3"])

            self.window._refresh_visible_plots()
            self.assertEqual(calls, ["summary2", "summary3"])

            self.window._direction_enu_dirty = True
            self.window.tab_widget.setCurrentWidget(self.window.dir_enu_tab)
            self.app.processEvents()
            self.assertEqual(calls, ["summary2", "summary3", "enu"])
        finally:
            self.window._update_direction_charts = original_summary2
            self.window._update_direction_charts3 = original_summary3
            self.window._update_direction_enu_charts = original_enu

    def test_direction_data_marks_visible_charts_dirty(self):
        self.window._reset_direction_stats()
        self.window._start_direction_test(2, 0)
        try:
            self.window._direction_summary_dirty = False
            self.window._do_feed_direction_stats(
                1.0, 2.0, 3.0, 4,
                self.window.direction_stats,
                self.window.dir_stat_boxes,
                False, 0, 0,
            )
            self.assertTrue(self.window._direction_summary_dirty)

            self.window.enu1_times[:] = [0.0]
            self.window.enu1_east_data[:] = [0.1]
            self.window.enu1_north_data[:] = [0.2]
            self.window.enu1_up_data[:] = [0.3]
            self.window.enu2_times[:] = [0.0]
            self.window.enu2_east_data[:] = [1.1]
            self.window.enu2_north_data[:] = [1.2]
            self.window.enu2_up_data[:] = [1.3]
            self.window._direction_enu_dirty = False
            self.window._save_enu_for_direction(0)
            self.assertTrue(self.window._direction_enu_dirty)
        finally:
            self.window._reset_direction_stats()

    def test_reset_marks_direction_charts_dirty(self):
        self.window._direction_summary_dirty = False
        self.window._direction_enu_dirty = False
        self.window._reset_direction_stats()
        self.assertTrue(self.window._direction_summary_dirty)
        self.assertTrue(self.window._direction_enu_dirty)

    def test_accepted_enu_sample_marks_live_plot_dirty(self):
        self.window._enu_plot_dirty = False
        try:
            self.window._feed_fixed_enu1()
            self.assertTrue(self.window._enu_plot_dirty)
        finally:
            self.window._reset_enu_chart_data(1)

    def test_pausing_live_plot_keeps_enu_collection_running(self):
        self.assertTrue(
            self.window.enu_comp_tab.isAncestorOf(self.window.plot_pause_btn)
        )
        self.window.tab_widget.setCurrentWidget(self.window.enu_comp_tab)
        self.app.processEvents()

        calls = []
        original_refresh = self.window._refresh_enu_charts
        self.window._refresh_enu_charts = lambda: calls.append("refresh")
        before = len(self.window.enu1_times)
        try:
            self.window.plot_pause_btn.click()
            self.assertTrue(self.window._plot_paused)
            self.assertEqual(self.window.plot_pause_btn.text(), "继续实时绘图")

            self.window._feed_fixed_enu1()
            self.assertEqual(len(self.window.enu1_times), before + 1)
            self.window._refresh_visible_plots()
            self.assertEqual(calls, [])

            self.window.plot_pause_btn.click()
            self.assertFalse(self.window._plot_paused)
            self.assertEqual(self.window.plot_pause_btn.text(), "暂停实时绘图")
            self.assertEqual(calls, ["refresh"])
        finally:
            self.window._refresh_enu_charts = original_refresh
            if getattr(self.window, "_plot_paused", False):
                self.window.plot_pause_btn.click()
            self.window._reset_enu_chart_data(1)

    def test_live_enu_curves_clip_to_view_without_downsampling(self):
        curves = (
            self.window.enu1_east_curve,
            self.window.enu2_east_curve,
            self.window.enu1_north_curve,
            self.window.enu2_north_curve,
            self.window.enu1_up_curve,
            self.window.enu2_up_curve,
            self.window.enu1_13_east_curve,
            self.window.enu3_east_curve,
            self.window.enu1_13_north_curve,
            self.window.enu3_north_curve,
            self.window.enu1_13_up_curve,
            self.window.enu3_up_curve,
        )
        for curve in curves:
            self.assertTrue(curve.opts.get("clipToView"))
            self.assertFalse(curve.opts.get("autoDownsample"))
            self.assertEqual(curve.opts.get("downsample"), 1)

    def test_statistics_methods_are_visible_on_report_tab(self):
        self.assertTrue(hasattr(self.window, "statistics_method_box"))
        self.assertTrue(hasattr(self.window, "statistics_method_text"))
        self.assertTrue(
            self.window.test_report_tab.isAncestorOf(self.window.statistics_method_box)
        )
        self.assertTrue(self.window.statistics_method_text.isReadOnly())
        self.assertGreaterEqual(self.window.statistics_method_text.minimumHeight(), 260)

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
