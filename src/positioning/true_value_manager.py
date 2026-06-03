from PyQt5.QtCore import QObject, pyqtSignal

class TrueValueManager(QObject):
    """真值管理器 - 负责存储、校验、变更通知"""
    
    value_changed = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._alt_type = 'ellipsoid'  # 'ellipsoid' 或 'geoid'
        self._geoid_offset = 0.0
    
    @property
    def lat(self):
        return self._lat
    
    @lat.setter
    def lat(self, value):
        if -90.0 <= value <= 90.0:
            self._lat = value
            self.value_changed.emit()
        else:
            raise ValueError(f"Latitude {value} out of range [-90.0, 90.0]")
    
    @property
    def lon(self):
        return self._lon
    
    @lon.setter
    def lon(self, value):
        if -180.0 <= value <= 180.0:
            self._lon = value
            self.value_changed.emit()
        else:
            raise ValueError(f"Longitude {value} out of range [-180.0, 180.0]")
    
    @property
    def alt(self):
        return self._alt
    
    @alt.setter
    def alt(self, value):
        if -500.0 <= value <= 100000.0:
            self._alt = value
            self.value_changed.emit()
        else:
            raise ValueError(f"Altitude {value} out of range [-500.0, 100000.0]")
    
    @property
    def alt_type(self):
        return self._alt_type
    
    @alt_type.setter
    def alt_type(self, value):
        if value in ['ellipsoid', 'geoid']:
            self._alt_type = value
            self.value_changed.emit()
    
    @property
    def geoid_offset(self):
        return self._geoid_offset
    
    @geoid_offset.setter
    def geoid_offset(self, value):
        if -100.0 <= value <= 100.0:
            self._geoid_offset = value
            self.value_changed.emit()
        else:
            raise ValueError(f"Geoid offset {value} out of range [-100.0, 100.0]")
    
    def get_ellipsoid_alt(self):
        """获取椭球高（用于与 GGA 匹配）"""
        if self._alt_type == 'ellipsoid':
            return self._alt
        else:
            return self._alt + self._geoid_offset
    
    def set_values(self, lat, lon, alt):
        """同时设置三个值"""
        if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
            self._lat = lat
            self._lon = lon
            self._alt = alt
            self.value_changed.emit()
    
    def get_values(self):
        """获取所有真值"""
        return {
            'lat': self._lat,
            'lon': self._lon,
            'alt': self._alt,
            'alt_type': self._alt_type,
            'geoid_offset': self._geoid_offset,
            'ellipsoid_alt': self.get_ellipsoid_alt()
        }
    
    def validate(self):
        """校验真值有效性"""
        errors = []
        if not (-90.0 <= self._lat <= 90.0):
            errors.append("纬度必须在 -90 到 90 之间")
        if not (-180.0 <= self._lon <= 180.0):
            errors.append("经度必须在 -180 到 180 之间")
        return errors
    
    def reset(self):
        """重置为默认值"""
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._alt_type = 'ellipsoid'
        self._geoid_offset = 0.0
        self.value_changed.emit()
