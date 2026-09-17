import os
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5.QtCore import QCoreApplication

from src.communication.serial_manager import SerialManager
from src.nmea.nmea_parser import NMEAParser
from src.positioning.direction_statistics import DirectionStatistics
from src.protocol.ubx_parser import extract_ubx_frames_from_buffer
from src.simulation.nmea_generator import _ubx_nav_status


class OptimizedCoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_reconnect_timer_stops_after_success(self):
        manager = SerialManager(1)
        manager.last_port = "COM_TEST"
        manager.last_settings = (9600, 8, 1, "None")
        manager.reconnect_enabled = True
        manager.connect = lambda *args, **kwargs: True
        manager.reconnect_timer.start(1000)
        manager.try_reconnect()
        self.assertFalse(manager.reconnect_timer.isActive())
        self.assertTrue(manager.reconnect_timer.isSingleShot())

    def test_false_ubx_sync_does_not_swallow_nested_valid_frame(self):
        valid = _ubx_nav_status(7200000, 35000)
        payload = b"noise" + valid + b"tail"
        false_frame = (b"\xb5\x62\x99\x01" + len(payload).to_bytes(2, "little")
                       + payload + b"\x00\x00")
        frames, remaining = extract_ubx_frames_from_buffer(false_frame)
        self.assertEqual(frames, [valid])
        self.assertIn(b"noise", remaining)
        self.assertIn(b"tail", remaining)

    def test_gsv_snapshot_replaces_missing_satellites_and_keeps_signals(self):
        parser = NMEAParser()
        parser.parse_gpgsv("$GPGSV,1,1,2,01,10,100,40,02,20,200,41,1")
        self.assertIn(("GP", 1, 1), parser.satellite_info)
        self.assertIn(("GP", 1, 2), parser.satellite_info)

        parser.parse_gpgsv("$GPGSV,1,1,1,02,20,200,42,1")
        self.assertNotIn(("GP", 1, 1), parser.satellite_info)
        self.assertEqual(parser.satellite_info[("GP", 1, 2)]["snr"], 42)

        parser.parse_gpgsv("$GPGSV,1,1,1,02,20,200,35,5")
        self.assertIn(("GP", 1, 2), parser.satellite_info)
        self.assertIn(("GP", 5, 2), parser.satellite_info)

    def test_rmc_tow_includes_day_of_week(self):
        parser = NMEAParser()
        # 2026-08-17 is Monday: GPS day index 1.
        parser.parse_gprmc("$GNRMC,020000.00,A,2811.000,N,11256.000,E,0,0,170826,,,A")
        self.assertTrue(parser.last_gps_time_valid)
        self.assertEqual(parser.last_gps_tow, 86400 + 7200)

    def test_direction_pause_resume_accumulates_and_invalid_fix_has_no_error(self):
        stats = DirectionStatistics()
        stats.start()
        stats.add_epoch(3.0, 4.0, 2.0, 1)
        time.sleep(0.01)
        stats.stop()
        first_duration = stats.duration
        stats.add_epoch(None, None, None, 0)
        self.assertEqual(stats.h_max, 5.0)
        self.assertEqual(stats.successful_epochs, 1)
        self.assertEqual(stats.total_epochs, 2)

        stats.start()
        time.sleep(0.01)
        stats.stop()
        self.assertGreater(stats.duration, first_duration)
        self.assertEqual(stats.successful_epochs, 1)


if __name__ == "__main__":
    unittest.main()
