r"""
定位误差在线统计累加器模块

本模块提供低开销、高精度的在线流式统计计算功能，针对 GNSS 定位误差数据进行实时分析。
通过增量维护样本数 n、一阶和 sum(x) 以及二阶平方和 sum(x^2)，
实现 O(1) 时间复杂度与 O(1) 空间复杂度的均值、均方根 (RMS)、无偏样本方差和标准差计算，
避免在内存中缓存全部历史点，适用于长时间测试与海量历元统计。
"""

import math


class StatisticsAccumulator:
    r"""
    单维度在线统计累加器类。

    维护单个变量（如水平误差或天向误差）的流式统计量。

    数学公式推导:
      - 样本量: n
      - 样本均值 (Mean): \mu = \sum x_i / n
      - 均方根误差 (RMS): RMS = \sqrt{\sum x_i^2 / n}
      - 样本方差 (Sample Variance, n-1 自由度无偏估计):
        s^2 = \sum (x_i - \mu)^2 / (n - 1) = (\sum x_i^2 - (\sum x_i)^2 / n) / (n - 1)
      - 样本标准差 (Sample Std): s = \sqrt{s^2}
    """
    
    def __init__(self):
        """初始化累加器状态。"""
        self.n = 0          # 样本总点数
        self.sum = 0.0      # 样本一阶累加和
        self.sum_sq = 0.0   # 样本平方累加和
    
    def add(self, value):
        """
        向累加器输入一个新样本值并更新内部统计和。

        参数:
            value (float): 新输入的误差测量值。
        """
        self.n += 1
        self.sum += value
        self.sum_sq += value ** 2
    
    def reset(self):
        """重置累加器计数与各阶累加和为零。"""
        self.n = 0
        self.sum = 0.0
        self.sum_sq = 0.0
    
    @property
    def mean(self):
        """
        计算样本算术平均值 (Mean)。

        返回:
            float: 样本均值；若样本数为空则返回 0.0。
        """
        if self.n == 0:
            return 0.0
        return self.sum / self.n
    
    @property
    def rms(self):
        """
        计算均方根误差 (Root Mean Square, RMS)。

        在 GNSS 定位误差评估中，RMS 反映了误差相对于零点的综合发散程度。

        返回:
            float: RMS 误差值；若样本数为空则返回 0.0。
        """
        if self.n == 0:
            return 0.0
        return math.sqrt(self.sum_sq / self.n)
    
    @property
    def variance(self):
        """
        计算样本无偏方差（采用 Bessel 修正，分母为 n - 1）。

        返回:
            float: 样本方差；若样本数少于 2 则返回 0.0。
        """
        if self.n < 2:
            return 0.0
        return (self.sum_sq - self.sum ** 2 / self.n) / (self.n - 1)
    
    @property
    def std(self):
        """
        计算样本标准差 (Standard Deviation, 1-Sigma)。

        返回:
            float: 样本标准差；若样本数少于 2 则返回 0.0。
        """
        return math.sqrt(self.variance)
    
    def get_stats(self):
        """
        获取包含当前维度全部统计指标的字典。

        返回:
            dict: 包含 count, mean, rms, variance, std 的字典。
        """
        return {
            'count': self.n,
            'mean': self.mean,
            'rms': self.rms,
            'variance': self.variance,
            'std': self.std
        }


class MultiDimensionStatistics:
    """
    多维定位误差联合统计模块。

    独立维护水平方向（Radial 2D，恒正绝对值）与垂直方向（Vertical Up，带符号）的统计量。
    """
    
    def __init__(self):
        """初始化水平与垂直统计累加器。"""
        self.horizontal = StatisticsAccumulator()  # 水平径向误差累加器（恒为正）
        self.vertical = StatisticsAccumulator()    # 天向高程误差累加器（带符号，可为正负）
    
    def add_errors(self, horizontal_error, vertical_error):
        """
        添加一个历元的水平误差与高程误差。

        参数:
            horizontal_error (float): 水平径向误差 sqrt(E^2 + N^2)。
            vertical_error (float): 天向高程误差 (Up)。
        """
        # 水平径向误差使用绝对值录入
        self.horizontal.add(abs(horizontal_error))
        # 高程误差保留原始正负符号录入，以便评估高程偏置 (Bias)
        self.vertical.add(vertical_error)
    
    def reset(self):
        """重置水平与垂直统计累加器。"""
        self.horizontal.reset()
        self.vertical.reset()
    
    def get_stats(self):
        """
        获取水平和垂直分维度的统计字典。

        返回:
            dict: {'horizontal': {...}, 'vertical': {...}}
        """
        return {
            'horizontal': self.horizontal.get_stats(),
            'vertical': self.vertical.get_stats()
        }
    
    def get_combined_stats(self):
        """
        获取扁平化合并的水平与高程统计摘要，便于报表生成或表格直接绑定。

        返回:
            dict: 包含 h_count, h_mean, h_rms, h_std, v_count, v_mean, v_rms, v_std 的字典。
        """
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

