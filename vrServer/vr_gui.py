import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QCheckBox, QLineEdit, QLabel, QTextEdit, QMessageBox
)
from PyQt5.QtCore import QThread, pyqtSignal
import threading

class Worker(QThread):
    log_signal = pyqtSignal(str)
    def __init__(self, cmd):
        super().__init__()
        self.cmd = cmd
        self.process = None

    def run(self):
        self.process = subprocess.Popen(
            self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        for line in self.process.stdout:
            self.log_signal.emit(line)
        self.process.wait()

    def stop(self):
        if self.process:
            self.process.terminate()
        # 避免线程自己 join 自己
        if self is not threading.current_thread():
            self.wait()

class VRGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VR服务器控制台")
        self.worker = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # 运行模式
        self.video_cb = QCheckBox("视频")
        self.command_cb = QCheckBox("指令")
        self.audio_cb = QCheckBox("音频")
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(self.video_cb)
        mode_layout.addWidget(self.command_cb)
        mode_layout.addWidget(self.audio_cb)
        layout.addLayout(mode_layout)

        # IP 和 repo_id
        self.ip_edit = QLineEdit("17.16.2.88")
        self.repo_edit = QLineEdit("dual_arm/test_dp")
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("IP:"))
        ip_layout.addWidget(self.ip_edit)
        ip_layout.addWidget(QLabel("repo_id:"))
        ip_layout.addWidget(self.repo_edit)
        layout.addLayout(ip_layout)

        # 启动/停止按钮
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("启动")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        layout.addLayout(btn_layout)

        # 日志输出
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        layout.addWidget(self.log_edit)

        self.setLayout(layout)

        # 事件绑定
        self.start_btn.clicked.connect(self.start_server)
        self.stop_btn.clicked.connect(self.stop_server)

    def start_server(self):
        modes = []
        if self.video_cb.isChecked():
            modes.append("video")
        if self.command_cb.isChecked():
            modes.append("command")
        if self.audio_cb.isChecked():
            modes.append("audio")
        if not modes:
            QMessageBox.warning(self, "提示", "请至少选择一个运行模式")
            return

        ip = self.ip_edit.text()
        repo_id = self.repo_edit.text()
        cmd = [
            sys.executable, "vrServer/main.py",
            "--mode", *modes,
            "--ip", ip,
            "--repo_id", repo_id
        ]
        self.worker = Worker(cmd)
        self.worker.log_signal.connect(self.append_log)
        self.worker.start()
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.append_log("[UI] 服务已启动...\n")

    def stop_server(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.append_log("[UI] 服务已停止\n")

    def append_log(self, text):
        self.log_edit.append(text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = VRGui()
    gui.show()
    sys.exit(app.exec_())