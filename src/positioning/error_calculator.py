import math

class ErrorCalculator:
    """误差计算模块（无状态）"""
    
    R_EARTH = 6371000.0  # 地球半径（米）
    
    @staticmethod
    def calculate_errors(measured, true):
        """
        计算水平距离误差和高程误差
        
        参数：
        measured: {'lat': float, 'lon': float, 'alt': float}
        true: {'lat': float, 'lon': float, 'alt': float}
        
        返回：
        {'horizontal_error': float, 'vertical_error': float}
        """
        # 水平距离误差（Haversine 公式）
        h_error = ErrorCalculator._haversine_distance(
            measured['lat'], measured['lon'],
            true['lat'], true['lon']
        )
        
        # 高程误差（带符号）
        v_error = measured['alt'] - true['alt']
        
        return {
            'horizontal_error': h_error,
            'vertical_error': v_error
        }
    
    @staticmethod
    def _haversine_distance(lat1, lon1, lat2, lon2):
        """使用 Haversine 公式计算两点间球面距离"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        d_lat = lat2_rad - lat1_rad
        d_lon = lon2_rad - lon1_rad
        
        a = math.sin(d_lat / 2) ** 2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * \
            math.sin(d_lon / 2) ** 2
        a = min(max(a, 0.0), 1.0)  # Clamp for floating-point safety

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return ErrorCalculator.R_EARTH * c
    
    @staticmethod
    def calculate_plane_distance(lat1, lon1, lat2, lon2):
        """平面近似距离计算（适用于近距离）"""
        # 将经纬度差转换为米
        d_lat = (lat2 - lat1) * 111139.0  # 纬度每度约 111.139 公里
        mid_lat = (lat1 + lat2) / 2.0
        d_lon = (lon2 - lon1) * 111139.0 * math.cos(math.radians(mid_lat))
        
        return math.sqrt(d_lat ** 2 + d_lon ** 2)
