import re

class NMEAGGAParser:
    """NMEA GGA sentence parser module"""

    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.quality = 0
        self.num_sat = 0
        self.hdop = 0.0
        self.geoid_height = 0.0
        self.timestamp = ""

    def parse(self, sentence):
        """Parse a GGA sentence."""
        if not sentence.startswith("$"):
            return False

        if not self._validate_checksum(sentence):
            return False

        parts = sentence.split(",")
        if len(parts) < 15:
            return False

        if not parts[0].endswith("GGA"):
            return False

        try:
            self.timestamp = parts[1]

            if parts[2]:
                self.lat = self._dmm_to_dd(float(parts[2]))
                if parts[3] == "S":
                    self.lat = -self.lat
            else:
                self.lat = 0.0

            if parts[4]:
                self.lon = self._dmm_to_dd(float(parts[4]))
                if parts[5] == "W":
                    self.lon = -self.lon
            else:
                self.lon = 0.0

            self.quality = int(parts[6]) if parts[6] else 0
            self.num_sat = int(parts[7]) if parts[7] else 0
            self.hdop = float(parts[8]) if parts[8] else 0.0
            self.alt = float(parts[9]) if parts[9] else 0.0
            self.geoid_height = float(parts[11]) if parts[11] else 0.0

            return True
        except Exception:
            return False

    def _validate_checksum(self, sentence):
        """Validate the NMEA XOR checksum."""
        if "*" not in sentence:
            return False
        try:
            data_part, checksum = sentence.split("*")
            calculated = 0
            for char in data_part[1:]:  # skip "$"
                calculated ^= ord(char)
            return calculated == int(checksum, 16)
        except Exception:
            return False

    def _dmm_to_dd(self, dmm):
        """Convert degrees-minutes format to decimal degrees."""
        degrees = int(dmm // 100)
        minutes = dmm % 100
        return degrees + minutes / 60.0

    def get_data(self):
        """Return parsed result."""
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "quality": self.quality,
            "num_sat": self.num_sat,
            "hdop": self.hdop,
            "geoid_height": self.geoid_height,
            "timestamp": self.timestamp,
        }

    def clear(self):
        """Clear parsed result."""
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.quality = 0
        self.num_sat = 0
        self.hdop = 0.0
        self.geoid_height = 0.0
        self.timestamp = ""