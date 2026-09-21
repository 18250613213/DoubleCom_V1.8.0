import os
import sys
import time
import unittest

import pyqtgraph as pg

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from PyQt5.QtCore import QPoint
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QGroupBox, QLineEdit,
    QPushButton, QSpinBox, QTableWidget,
)

from main import NMEADataAnalyzer, SlidingWindowStd
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
        log_font = self.window.log_window.font()
        log_size = log_font.pointSizeF() if log_font.pointSizeF() > 0 else log_font.pixelSize()
        self.assertGreaterEqual(log_size, 14)
        self.assertGreaterEqual(self.window.log_window.minimumHeight(), 240)

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

    def test_global_typography_is_readable_on_a_laptop(self):
        self.assertGreaterEqual(self.window.font().pointSizeF(), 12)
        self.assertGreaterEqual(self.window.tab_widget.font().pointSizeF(), 13)
        self.assertGreaterEqual(self.window.snr_table.font().pointSizeF(), 12)
        self.assertGreaterEqual(self.window.data_preview.font().pointSizeF(), 14)
        self.assertGreaterEqual(self.window.log_window.font().pointSizeF(), 14)

        readable_classes = (QGroupBox, QPushButton, QComboBox, QLineEdit,
                            QSpinBox, QDoubleSpinBox)
        too_small = []
        for widget_class in readable_classes:
            for widget in self.window.findChildren(widget_class):
                if widget.font().pointSizeF() < 12:
                    too_small.append(
                        f"{widget.metaObject().className()}:{widget.objectName() or widget.toolTip()[:24]}"
                    )
        self.assertEqual([], too_small)

    def test_interactive_controls_have_laptop_friendly_touch_heights(self):
        compact_controls = (QPushButton, QComboBox, QLineEdit,
                            QSpinBox, QDoubleSpinBox)
        too_short = []
        for widget_class in compact_controls:
            for widget in self.window.findChildren(widget_class):
                if (widget_class is QLineEdit and
                        isinstance(widget.parentWidget(), (QSpinBox, QDoubleSpinBox))):
                    continue
                if widget.minimumHeight() < 32:
                    too_short.append(
                        f"{widget.metaObject().className()}:{widget.objectName() or widget.toolTip()[:24]}"
                    )
        self.assertEqual([], too_short)

    def test_all_plot_axes_use_readable_fonts(self):
        plots = self.window.findChildren(pg.PlotWidget)
        self.assertGreater(len(plots), 0)
        for plot in plots:
            for axis_name in ("left", "bottom"):
                axis = plot.getAxis(axis_name)
                tick_font = axis.style.get("tickFont")
                self.assertIsNotNone(tick_font)
                self.assertGreaterEqual(tick_font.pointSizeF(), 10)
                self.assertGreaterEqual(axis.label.font().pointSizeF(), 11)

    def test_primary_pages_fit_a_1366_by_768_laptop_without_horizontal_scroll(self):
        self.window.showNormal()
        self.window.resize(1366, 768)
        self.app.processEvents()
        try:
            self.assertLessEqual(
                self.window.width(), 1366,
                "content minimum width must not force a laptop window wider than 1366px",
            )
            self.assertFalse(self.window.tab_widget.usesScrollButtons())
            margins = self.window.centralWidget().layout().contentsMargins()
            self.assertLessEqual(margins.left(), 8)
            self.assertLessEqual(margins.right(), 8)

            for tab_name, scroll in (
                ("功能与控制面板", self.window.control_scroll),
                ("测试信息与报告", self.window.test_report_scroll),
            ):
                titles = [
                    self.window.tab_widget.tabText(i)
                    for i in range(self.window.tab_widget.count())
                ]
                self.window.tab_widget.setCurrentIndex(titles.index(tab_name))
                self.app.processEvents()
                self.assertEqual(
                    scroll.horizontalScrollBar().maximum(), 0,
                    f"{tab_name} should not require horizontal scrolling at 1366px",
                )
        finally:
            self.window.showMaximized()
            self.app.processEvents()

    def test_primary_pages_fit_a_125_percent_scaled_laptop_work_area(self):
        # 1366×768 at Windows 125% scaling, minus a typical taskbar, leaves
        # approximately 1093×580 logical pixels for the maximized application.
        self.window.showNormal()
        self.window.resize(1093, 580)
        self.app.processEvents()
        try:
            self.assertLessEqual(self.window.width(), 1093)
            self.assertLessEqual(self.window.height(), 580)

            titles = [
                self.window.tab_widget.tabText(i)
                for i in range(self.window.tab_widget.count())
            ]
            for tab_name in titles:
                self.window.tab_widget.setCurrentIndex(titles.index(tab_name))
                self.app.processEvents()
                self.assertLessEqual(self.window.width(), 1093, tab_name)
                self.assertLessEqual(self.window.height(), 580, tab_name)

            for scroll in (self.window.control_scroll, self.window.test_report_scroll):
                self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)

            self.assertGreaterEqual(self.window.data_preview.height(), 240)
            self.assertGreaterEqual(self.window.log_window.height(), 240)
        finally:
            self.window.showMaximized()
            self.app.processEvents()

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
