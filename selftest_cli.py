"""
一键自检命令行 (CLI) 入口脚本

提供无 GUI 界面的离屏 (offscreen) 自动化执行模式，便于 CI/CD 自动化集成、远程服务器测试或脚本化回归验证。

参数说明:
    --minutes: 模拟数据生成的时长 (分钟)，默认 60。
    --speed:   仿真时钟回放倍率 (例如 10 表示 10 倍速，60 表示 60 倍速)，默认 10。
    --seed:    伪随机数发生器种子，默认 42。
    --no-direction: 是否跳过方向测试检验。

退出状态码 (Exit Code):
    0: 自检项目全部通过 (ALL PASS)
    1: 存在未通过的失败检查项 (FAIL)
    2: 启动被拒绝（例如检测到已有物理串口被占用）
"""

import argparse
import sys


def main():
    """CLI 主执行入口函数。"""
    # 1. 命令行参数解析
    ap = argparse.ArgumentParser(description="DoubleCom 一键自检(无串口模拟测试)")
    ap.add_argument('--minutes', type=int, default=60, help='模拟时长(分钟), 默认60')
    ap.add_argument('--speed', type=int, default=10, help='加速倍率, 默认10')
    ap.add_argument('--seed', type=int, default=42, help='随机种子, 默认42')
    ap.add_argument('--no-direction', action='store_true', help='跳过方向测试')
    args = ap.parse_args()

    # 2. 启动无界面 Qt 应用程序实例 (使用 offscreen 渲染后端)
    from PyQt5.QtWidgets import QApplication
    app = QApplication(['selftest_cli', '-platform', 'offscreen'])

    from main import NMEADataAnalyzer
    from src.simulation.selftest import SelfTestRunner

    # 3. 实例化主窗口 (不调用 show()) 并关闭默认日志句柄
    window = NMEADataAnalyzer()
    window._close_auto_log_files()

    # 4. 创建无头模式自检运行器
    runner = SelfTestRunner(window, headless=True)

    def on_finished(ok, results):
        """自检完成回调：打印各检查项的 PASS/FAIL 详情并退出 Qt 事件循环。"""
        for name, rok, detail in results:
            print("[{}] {} - {}".format('PASS' if rok else 'FAIL', name, detail))
        print("RESULT: {}".format('PASS' if ok else 'FAIL'))
        print("REPORT: {}".format(runner.report_dir))
        app.exit(0 if ok else 1)

    # 5. 连接完成信号并启动测试
    runner.finished.connect(on_finished)
    if not runner.start(args.minutes, args.speed, args.seed, not args.no_direction):
        sys.exit(2)
        
    # 6. 进入 Qt 主事件循环直至自检结束退出
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

