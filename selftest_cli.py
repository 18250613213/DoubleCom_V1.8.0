"""一键自检CLI入口 - offscreen模式无界面运行, 适合脚本化回归验证。

用法:
    python selftest_cli.py                      # 60分钟@10x, seed=42, 含方向测试
    python selftest_cli.py --minutes 6 --speed 60 --no-direction
退出码: 0=全部通过, 1=存在失败项, 2=启动被拒绝
"""
import argparse
import sys


def main():
    ap = argparse.ArgumentParser(description="DoubleCom 一键自检(无串口模拟测试)")
    ap.add_argument('--minutes', type=int, default=60, help='模拟时长(分钟), 默认60')
    ap.add_argument('--speed', type=int, default=10, help='加速倍率, 默认10')
    ap.add_argument('--seed', type=int, default=42, help='随机种子, 默认42')
    ap.add_argument('--no-direction', action='store_true', help='跳过方向测试')
    args = ap.parse_args()

    from PyQt5.QtWidgets import QApplication
    app = QApplication(['selftest_cli', '-platform', 'offscreen'])

    from main import NMEADataAnalyzer
    from src.simulation.selftest import SelfTestRunner

    window = NMEADataAnalyzer()
    window._close_auto_log_files()

    runner = SelfTestRunner(window, headless=True)

    def on_finished(ok, results):
        for name, rok, detail in results:
            print("[{}] {} - {}".format('PASS' if rok else 'FAIL', name, detail))
        print("RESULT: {}".format('PASS' if ok else 'FAIL'))
        print("REPORT: {}".format(runner.report_dir))
        app.exit(0 if ok else 1)

    runner.finished.connect(on_finished)
    if not runner.start(args.minutes, args.speed, args.seed, not args.no_direction):
        sys.exit(2)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
