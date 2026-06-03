import math

class StatisticsAccumulator:
    """单维度统计累加器"""
    
    def __init__(self):
        self.n = 0
        self.sum = 0.0
        self.sum_sq = 0.0
    
    def add(self, value):
        """添加新值"""
        self.n += 1
        self.sum += value
        self.sum_sq += value ** 2
    
    def reset(self):
        """重置累加器"""
        self.n = 0
        self.sum = 0.0
        self.sum_sq = 0.0
    
    @property
    def mean(self):
        """计算均值"""
        if self.n == 0:
            return 0.0
        return self.sum / self.n
    
    @property
    def rms(self):
        """计算 RMS（均方根）"""
        if self.n == 0:
            return 0.0
        return math.sqrt(self.sum_sq / self.n)
    
    @property
    def variance(self):
        """计算方差（样本方差，n-1）"""
        if self.n < 2:
            return 0.0
        return (self.sum_sq - self.sum ** 2 / self.n) / (self.n - 1)
    
    @property
    def std(self):
        """计算标准差（样本标准差）"""
        return math.sqrt(self.variance)
    
    def get_stats(self):
        """获取所有统计值"""
        return {
            'count': self.n,
            'mean': self.mean,
            'rms': self.rms,
            'variance': self.variance,
            'std': self.std
        }

class MultiDimensionStatistics:
    """多维统计模块（水平 + 垂直独立）"""
    
    def __init__(self):
        self.horizontal = StatisticsAccumulator()  # 水平误差（恒正）
        self.vertical = StatisticsAccumulator()    # 高程误差（带符号）
    
    def add_errors(self, horizontal_error, vertical_error):
        """添加误差值"""
        # 水平误差使用绝对值
        self.horizontal.add(abs(horizontal_error))
        # 高程误差使用带符号值
        self.vertical.add(vertical_error)
    
    def reset(self):
        """重置所有统计"""
        self.horizontal.reset()
        self.vertical.reset()
    
    def get_stats(self):
        """获取所有统计值"""
        return {
            'horizontal': self.horizontal.get_stats(),
            'vertical': self.vertical.get_stats()
        }
    
    def get_combined_stats(self):
        """获取合并的统计摘要"""
        h_stats = self.horizontal.get_stats()
        v_stats = self.vertical.get_stats()
        
        return {
            'h_count': h_stats['count'],
            'h_mean': h_stats['mean'],
            'h_rms': h_stats['rms'],
            'h_std': h_stats['std'],
            'v_count': v_stats['count'],
            'v_mean': v_stats['mean'],
            'v_rms': v_stats['rms'],
            'v_std': v_stats['std']
        }
