from PyQt5.QtCore import QObject, pyqtSignal

class WarningModule(QObject):
    """预警模块 - 含防抖逻辑"""
    
    horizontal_warning = pyqtSignal(bool)
    vertical_warning = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.h_threshold = 10.0   # 水平容差（米）
        self.v_threshold = 5.0    # 高程容差（米）
        self.h_count = 0          # 水平超限计数
        self.v_count = 0          # 高程超限计数
        self.debounce_limit = 3   # 连续超限次数阈值
        self.h_active = False
        self.v_active = False
    
    def set_thresholds(self, horizontal, vertical):
        """设置阈值"""
        self.h_threshold = horizontal
        self.v_threshold = vertical
    
    def update(self, horizontal_error, vertical_error):
        """更新预警状态"""
        # 水平预警（使用绝对值）
        if abs(horizontal_error) > self.h_threshold:
            self.h_count += 1
            if self.h_count >= self.debounce_limit and not self.h_active:
                self.h_active = True
                self.horizontal_warning.emit(True)
        else:
            self.h_count = max(0, self.h_count - 1)
            if self.h_count == 0 and self.h_active:
                self.h_active = False
                self.horizontal_warning.emit(False)
        
        # 垂直预警（使用绝对值）
        if abs(vertical_error) > self.v_threshold:
            self.v_count += 1
            if self.v_count >= self.debounce_limit and not self.v_active:
                self.v_active = True
                self.vertical_warning.emit(True)
        else:
            self.v_count = max(0, self.v_count - 1)
            if self.v_count == 0 and self.v_active:
                self.v_active = False
                self.vertical_warning.emit(False)
    
    def reset(self):
        """重置预警状态"""
        self.h_count = 0
        self.v_count = 0
        self.h_active = False
        self.v_active = False
        self.horizontal_warning.emit(False)
        self.vertical_warning.emit(False)
    
    def get_status(self):
        """获取当前预警状态"""
        return {
            'horizontal': {
                'active': self.h_active,
                'count': self.h_count,
                'threshold': self.h_threshold
            },
            'vertical': {
                'active': self.v_active,
                'count': self.v_count,
                'threshold': self.v_threshold
            }
        }
