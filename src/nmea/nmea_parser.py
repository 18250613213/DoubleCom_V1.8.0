import re
from collections import deque

class NMEAParser:
    """Single-threaded NMEA sentence parser.

    This class is NOT thread-safe. It must only be used from one thread
    at a time. The caller is responsible for serializing all access.
    """

    def __init__(self):
        self.clear()

    def clear(self):
        self.total_sentences = 0
        self.unknown_sentences = 0
        self.gpgga_data = deque(maxlen=1)
        self.gprmc_data = deque(maxlen=1)
        self.gpgsv_data = deque(maxlen=50)
        self.gpgsa_data = deque(maxlen=1)
        self.obsvha_data = deque(maxlen=1)
        self.obsvma_data = deque(maxlen=1)
        self.satellite_info = {}
        self.position_fixes = deque(maxlen=1)
        self.dop_values = deque(maxlen=1)
        self.errors = deque(maxlen=100)
        self.last_gps_tow = 0.0
        self.last_gps_time_valid = False

    def decode_line(self, data):
        try:
            if isinstance(data, bytes):
                return data.decode("utf-8", errors="ignore").strip()
            return str(data).strip()
        except Exception:
            return None

    @staticmethod
    def _compute_xor_checksum(data_part):
        """Compute the NMEA XOR checksum over characters after '$'."""
        calculated = 0
        for ch in data_part[1:]:  # skip leading '$'
            calculated ^= ord(ch)
        return calculated

    def parse(self, sentence):
        if not sentence:
            return

        self.total_sentences += 1

        try:
            # --- Strip and validate NMEA checksum BEFORE dispatching ---
            if "*" in sentence:
                data_part, checksum_hex = sentence.split("*", 1)
                calculated = self._compute_xor_checksum(data_part)
                try:
                    expected = int(checksum_hex[:2], 16)
                except ValueError:
                    expected = None
                if expected is None or calculated != expected:
                    self.errors.append(
                        f"Checksum mismatch: {sentence[:50]}..."
                    )
                    return
            else:
                data_part = sentence

            # Dispatch clean sentence (no trailing *XX)
            if (data_part.startswith("$GPGGA") or data_part.startswith("$BDGGA") or
                    data_part.startswith("$GLGGA") or data_part.startswith("$GNGGA") or
                    data_part.startswith("$GAGGA") or data_part.startswith("$GBGGA")):
                self.parse_gpgga(data_part)
            elif (data_part.startswith("$GPRMC") or data_part.startswith("$BDRMC") or
                    data_part.startswith("$GLRMC") or data_part.startswith("$GNRMC") or
                    data_part.startswith("$GARMC") or data_part.startswith("$GBRMC")):
                self.parse_gprmc(data_part)
            elif (data_part.startswith("$GPGSV") or data_part.startswith("$BDGSV") or
                    data_part.startswith("$GLGSV") or data_part.startswith("$GAGSV") or
                    data_part.startswith("$GBGSV") or data_part.startswith("$GNGSV")):
                self.parse_gpgsv(data_part)
            elif (data_part.startswith("$GPGSA") or data_part.startswith("$BDGSA") or
                    data_part.startswith("$GLGSA") or data_part.startswith("$GNGSA") or
                    data_part.startswith("$GAGSA") or data_part.startswith("$GBGSA")):
                self.parse_gpgsa(data_part)
            elif data_part.startswith("#OBSVHA"):
                self.parse_obsvha(data_part)
            elif data_part.startswith("#OBSVMA"):
                self.parse_obsvma(data_part)
            else:
                self.unknown_sentences += 1

        except Exception as e:
            self.errors.append(f"Parse error: {sentence} - {str(e)}")

    # Expected field counts per NMEA sentence type
    _EXPECTED_FIELDS = {"GGA": 15, "RMC": 13, "GSA": 18}

    @staticmethod
    def _normalize_fields(parts, expected_count):
        """Pad *parts* in-place to *expected_count* with empty strings."""
        short = expected_count - len(parts)
        if short > 0:
            parts.extend([""] * short)
        return parts

    def parse_gpgga(self, sentence):
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 15)

        data = {
            "timestamp": parts[1],
            "latitude": parts[2],
            "lat_dir": parts[3],
            "longitude": parts[4],
            "lon_dir": parts[5],
            "fix_quality": int(parts[6]) if parts[6] else 0,
            "satellites_used": int(parts[7]) if parts[7] else 0,
            "hdop": float(parts[8]) if parts[8] else 0.0,
            "altitude": float(parts[9]) if parts[9] else 0.0,
            "altitude_units": parts[10],
            "geoidal_separation": float(parts[11]) if parts[11] else 0.0,
            "age_of_diff": parts[13],
            "diff_station": parts[14],
        }

        self.gpgga_data.append(data)

        if data["fix_quality"] > 0:
            lat = self.convert_coordinate(data["latitude"], data["lat_dir"])
            lon = self.convert_coordinate(data["longitude"], data["lon_dir"])
            self.position_fixes.append({
                "timestamp": data["timestamp"],
                "latitude": lat,
                "longitude": lon,
                "fix_quality": data["fix_quality"],
                "satellites": data["satellites_used"],
                "hdop": data["hdop"],
            })

    def parse_gprmc(self, sentence):
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 13)

        data = {
            "timestamp": parts[1],
            "status": parts[2],
            "latitude": parts[3],
            "lat_dir": parts[4],
            "longitude": parts[5],
            "lon_dir": parts[6],
            "speed": float(parts[7]) if parts[7] else 0.0,
            "course": float(parts[8]) if parts[8] else 0.0,
            "date": parts[9],
            "magnetic_variation": float(parts[10]) if parts[10] else 0.0,
            "variation_dir": parts[11],
            "mode": parts[12] if parts[12] else "",
        }

        self.gprmc_data.append(data)

        if data["date"] and data["timestamp"]:
            try:
                day = int(data["date"][:2])
                month = int(data["date"][2:4])
                year = 2000 + int(data["date"][4:6])

                hours = int(data["timestamp"][:2])
                minutes = int(data["timestamp"][2:4])
                seconds = float(data["timestamp"][4:])

                seconds_of_day = hours * 3600 + minutes * 60 + seconds

                self.last_gps_tow = seconds_of_day % 604800
                self.last_gps_time_valid = True
            except Exception:
                pass

    def parse_gpgsv(self, sentence):
        # Strip trailing checksum BEFORE splitting (defensive -- parse()
        # already strips it, but this method may be called directly).
        if "*" in sentence:
            sentence, _ = sentence.split("*", 1)

        parts = sentence.split(",")
        if len(parts) < 8:
            return

        talker_id = parts[0][1:3]  # "GP", "BD", "GL", "GA", "GB"
        total_messages = int(parts[1]) if parts[1] else 0
        message_number = int(parts[2]) if parts[2] else 0
        total_satellites = int(parts[3]) if parts[3] else 0

        # --- GSV sequence reset ---
        if message_number == 1:
            # Clear satellite_info entries belonging to this talker so that
            # a fresh GSV cycle replaces stale data from the previous one.
            self.satellite_info = {
                prn: info
                for prn, info in self.satellite_info.items()
                if info.get("talker_id") != talker_id
            }

        extra_fields = len(parts) - 4
        signal_id = 0
        if extra_fields % 4 == 1:
            signal_id = int(parts[-1]) if parts[-1] else 0
            sat_count = extra_fields // 4
        else:
            sat_count = extra_fields // 4

        satellites = []

        for i in range(sat_count):
            idx = 4 + i * 4
            if idx + 3 >= len(parts):
                break

            prn = int(parts[idx]) if parts[idx] else 0
            elevation = int(parts[idx + 1]) if parts[idx + 1] else 0
            azimuth = int(parts[idx + 2]) if parts[idx + 2] else 0
            raw_snr = int(parts[idx + 3]) if parts[idx + 3] else 0
            snr = raw_snr if 0 <= raw_snr <= 99 else 0

            satellites.append({
                "prn": prn,
                "elevation": elevation,
                "azimuth": azimuth,
                "snr": snr,
                "signal_id": signal_id,
            })

            self.satellite_info[prn] = {
                "elevation": elevation,
                "azimuth": azimuth,
                "snr": snr,
                "signal_id": signal_id,
                "talker_id": talker_id,
            }

        self.gpgsv_data.append({
            "talker_id": talker_id,
            "total_messages": total_messages,
            "message_number": message_number,
            "total_satellites": total_satellites,
            "signal_id": signal_id,
            "satellites": satellites,
        })

    def parse_gpgsa(self, sentence):
        parts = sentence.split(",")
        if len(parts) < 2:
            return
        parts = self._normalize_fields(parts, 18)

        data = {
            "mode": parts[1],
            "fix_type": int(parts[2]) if parts[2] else 0,
            "satellites": [int(p) for p in parts[3:15] if p],
            "pdop": float(parts[15]) if parts[15] else 0.0,
            "hdop": float(parts[16]) if parts[16] else 0.0,
            "vdop": float(parts[17]) if parts[17] else 0.0,
        }

        self.gpgsa_data.append(data)
        self.dop_values.append({
            "pdop": data["pdop"],
            "hdop": data["hdop"],
            "vdop": data["vdop"],
        })

    def parse_obsvha(self, sentence):
        try:
            if "*" in sentence:
                main_part, checksum = sentence.split("*", 1)
            else:
                main_part = sentence
                checksum = None

            parts = main_part.split(",")
            if len(parts) < 10:
                return

            data = {
                "version": int(parts[1]) if parts[1] else 0,
                "gnss": parts[2],
                "mode": parts[3],
                "rcv_tow": int(parts[4]) if parts[4] else 0,
                "rcv_wkn": int(parts[5]) if parts[5] else 0,
                "num_sat": int(parts[8]) if parts[8] else 0,
                "satellites": [],
                "checksum": checksum,
            }

            prn_block = parts[9] if len(parts) > 9 else ""
            prn_list = [int(p) for p in prn_block.split(";") if p.strip().isdigit()]

            def safe_int(s, default=0):
                try:
                    return int(float(s))
                except Exception:
                    return default

            def safe_float(s, default=0.0):
                try:
                    return float(s)
                except Exception:
                    return default

            obs_data = parts[10:] if len(parts) > 10 else []

            num_satellites = len(obs_data) // 11
            for sat_idx in range(num_satellites):
                start = sat_idx * 11
                fields = obs_data[start:start + 11]

                sat = {
                    "prn": safe_int(fields[1]),
                    "gnss_id": safe_int(fields[0]),
                    "sv_id": safe_int(fields[1]),
                    "pseudorange": safe_float(fields[2]),
                    "carrier_phase": safe_float(fields[3]),
                    "doppler": safe_float(fields[4]),
                    "cn0": safe_int(fields[5]),
                    "locktime": safe_int(fields[6]),
                    "flags": safe_int(fields[7]),
                    "reserved": safe_int(fields[8]),
                    "pseudorange_rate": safe_float(fields[9]),
                    "timestamp": data["rcv_tow"] / 1000.0,
                }
                data["satellites"].append(sat)

            self.obsvha_data.append(data)

        except Exception as e:
            self.errors.append(
                f"Parse OBSVHA failed: {sentence[:50]}... - {str(e)}"
            )

    def parse_obsvma(self, sentence):
        try:
            if "*" in sentence:
                main_part, checksum = sentence.split("*", 1)
            else:
                main_part = sentence
                checksum = None

            parts = main_part.split(",")
            if len(parts) < 10:
                return

            data = {
                "version": int(parts[1]) if parts[1] else 0,
                "gnss": parts[2],
                "mode": parts[3],
                "rcv_tow": int(parts[4]) if parts[4] else 0,
                "rcv_wkn": int(parts[5]) if parts[5] else 0,
                "num_sat": int(parts[8]) if parts[8] else 0,
                "satellites": [],
                "checksum": checksum,
            }

            prn_list_str = parts[9] if len(parts) > 9 else ""
            prn_list = [
                int(p) for p in prn_list_str.split(";") if p.strip().isdigit()
            ]

            obs_data = parts[10:] if len(parts) > 10 else []

            def safe_int(s, default=0):
                try:
                    return int(float(s))
                except Exception:
                    return default

            def safe_float(s, default=0.0):
                try:
                    return float(s)
                except Exception:
                    return default

            num_satellites = len(obs_data) // 11

            for sat_idx in range(num_satellites):
                start = sat_idx * 11
                fields = obs_data[start:start + 11]

                prn = prn_list[sat_idx] if sat_idx < len(prn_list) else 0

                sat = {
                    "prn": prn,
                    "gnss_id": safe_int(fields[0]),
                    "sv_id": safe_int(fields[1]),
                    "pseudorange": safe_float(fields[2]) / 100.0,
                    "carrier_phase": safe_float(fields[3]),
                    "doppler": safe_float(fields[4]),
                    "cn0": safe_int(fields[5]),
                    "locktime": safe_int(fields[6]),
                    "flags": safe_int(fields[7], 16),
                    "reserved": safe_int(fields[8]),
                    "freq_idx": safe_int(fields[9]),
                    "timestamp": data["rcv_tow"] / 1000.0,
                    "X": None,
                    "Y": None,
                    "Z": None,
                    "has_sat_pos": False,
                }

                data["satellites"].append(sat)

            self.obsvma_data.append(data)

        except Exception as e:
            self.errors.append(
                f"Parse OBSVMA failed: {sentence[:50]}... - {str(e)}"
            )

    def convert_coordinate(self, coord_str, direction):
        if not coord_str:
            return 0.0

        try:
            dmm = float(coord_str)
            degrees = int(dmm // 100)
            minutes = dmm % 100
            value = degrees + minutes / 60.0

            if direction in ["S", "W"]:
                value = -value

            return round(value, 6)
        except Exception:
            return 0.0

    def get_statistics(self):
        obsv_sat_info = {}
        for obsv in list(self.obsvha_data) + list(self.obsvma_data):
            for sat in obsv["satellites"]:
                prn = sat["prn"]
                if prn not in obsv_sat_info:
                    obsv_sat_info[prn] = []
                obsv_sat_info[prn].append({
                    "cn0": sat["cn0"] / 10.0,
                    "doppler": sat["doppler"],
                    "locktime": sat["locktime"],
                })

        stats = {
            "total_sentences": self.total_sentences,
            "unknown_sentences": self.unknown_sentences,
            "gpgga_count": len(self.gpgga_data),
            "gprmc_count": len(self.gprmc_data),
            "gpgsv_count": len(self.gpgsv_data),
            "gpgsa_count": len(self.gpgsa_data),
            "obsvha_count": len(self.obsvha_data),
            "obsvma_count": len(self.obsvma_data),
            "valid_fixes": len(self.position_fixes),
            "total_satellites": len(self.satellite_info),
            "obsv_satellites": len(obsv_sat_info),
            "obsv_sat_info": obsv_sat_info,
            "errors": len(self.errors),
        }

        if self.position_fixes:
            stats["avg_satellites"] = (
                sum(f["satellites"] for f in self.position_fixes)
                / len(self.position_fixes)
            )
            stats["avg_hdop"] = (
                sum(f["hdop"] for f in self.position_fixes)
                / len(self.position_fixes)
            )
        else:
            stats["avg_satellites"] = 0
            stats["avg_hdop"] = 0

        if self.dop_values:
            stats["min_pdop"] = min(d["pdop"] for d in self.dop_values)
            stats["max_pdop"] = max(d["pdop"] for d in self.dop_values)
            stats["avg_pdop"] = (
                sum(d["pdop"] for d in self.dop_values) / len(self.dop_values)
            )
            stats["min_hdop"] = min(d["hdop"] for d in self.dop_values)
            stats["max_hdop"] = max(d["hdop"] for d in self.dop_values)
            stats["min_vdop"] = min(d["vdop"] for d in self.dop_values)
            stats["max_vdop"] = max(d["vdop"] for d in self.dop_values)
        else:
            stats.update({
                "min_pdop": 0,
                "max_pdop": 0,
                "avg_pdop": 0,
                "min_hdop": 0,
                "max_hdop": 0,
                "min_vdop": 0,
                "max_vdop": 0,
            })

        return stats