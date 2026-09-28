import json
import sys

import pymysql
from PyQt5.QtCore import QTimer
from PyQt5.QtNetwork import QAbstractSocket, QTcpSocket
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

BRIDGE_HOST = "localhost"
BRIDGE_PORT = 9090
RECONNECT_MS = 3000

DB_CONFIG = {
    "host": "localhost",
    "port": 3306,
    "user": "root",
    "password": "1234",
    "database": "rosdb",
}


def log(text):
    print(text, flush=True)


def save_pose(x, y, theta):
    conn = pymysql.connect(**DB_CONFIG)
    try:
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO turtlepos (x, y, theta, `time`) VALUES (%s, %s, %s, NOW())",
                (str(x), str(y), str(theta)),
            )
        conn.commit()
    finally:
        conn.close()


class TurtleClient(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Turtle Client")

        self.status = QLabel("연결 중...")
        self.action_label = QLabel("동작: -")

        pad = QGridLayout()
        arrows = {
            "↑": (0, 1, 1.0, 0.0),
            "←": (1, 0, 0.0, 1.0),
            "↓": (1, 1, -1.0, 0.0),
            "→": (1, 2, 0.0, -1.0),
        }
        for text, (row, col, linear_x, angular_z) in arrows.items():
            btn = QPushButton(text)
            btn.setFixedSize(64, 48)
            btn.clicked.connect(lambda _=False, lx=linear_x, az=angular_z: self.move(lx, az))
            pad.addWidget(btn, row, col)

        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(lambda: self.send({"type": "reset"}))
        save_btn = QPushButton("Pose DB 저장")
        save_btn.clicked.connect(lambda: self.send({"type": "pose"}))

        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addLayout(pad)
        layout.addWidget(reset_btn)
        layout.addWidget(save_btn)
        layout.addWidget(self.action_label)

        self.pending = b""
        self.sock = QTcpSocket(self)
        self.sock.connected.connect(self.on_connected)
        self.sock.disconnected.connect(lambda: self.schedule_reconnect("연결 끊김"))
        self.sock.errorOccurred.connect(lambda _: self.schedule_reconnect(f"연결 실패: {self.sock.errorString()}"))
        self.sock.readyRead.connect(self.on_ready_read)

        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.setSingleShot(True)
        self.reconnect_timer.timeout.connect(self.connect_bridge)

        self.connect_bridge()

    def connect_bridge(self):
        if self.sock.state() == QAbstractSocket.UnconnectedState:
            self.pending = b""
            self.sock.connectToHost(BRIDGE_HOST, BRIDGE_PORT)

    def on_connected(self):
        self.reconnect_timer.stop()
        self.status.setText(f"연결됨 ({BRIDGE_HOST}:{BRIDGE_PORT})")

    def schedule_reconnect(self, reason):
        self.status.setText(f"{reason} — {RECONNECT_MS // 1000}초 후 재접속")
        self.reconnect_timer.start(RECONNECT_MS)

    def send(self, msg):
        if self.sock.state() != QAbstractSocket.ConnectedState:
            log("브리지에 연결되어 있지 않음")
            return
        self.sock.write((json.dumps(msg) + "\n").encode())

    def move(self, linear_x, angular_z):
        self.send({"type": "move", "linear_x": linear_x, "angular_z": angular_z})

    def on_ready_read(self):
        self.pending += bytes(self.sock.readAll())
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            try:
                self.handle_message(json.loads(line))
            except json.JSONDecodeError:
                log(f"잘못된 JSON: {line!r}")

    def handle_message(self, msg):
        kind = msg.get("type")
        action = msg.get("action", "")

        if kind == "goal_response":
            self.action_label.setText(f"동작: {action} {'수락' if msg.get('accepted') else '거부'}")
        elif kind == "feedback":
            remain = msg.get("remained_dist", msg.get("remaining"))
            self.action_label.setText(f"동작: {action} 진행 중 (남은 값 {remain:.2f})")
        elif kind == "result":
            self.action_label.setText(f"동작: {action} {msg.get('status')}")
            log(json.dumps(msg, ensure_ascii=False))
        elif kind == "reset":
            self.action_label.setText("동작: reset 완료")
        elif kind == "pose":
            self.on_pose(msg["x"], msg["y"], msg["theta"])
        elif kind == "error":
            log(f"오류 [{msg.get('code')}] {msg.get('message')}")
        else:
            log(json.dumps(msg, ensure_ascii=False))

    def on_pose(self, x, y, theta):
        try:
            save_pose(x, y, theta)
            log(f"DB 저장: x={x:.2f} y={y:.2f} theta={theta:.2f}")
        except pymysql.MySQLError as e:
            log(f"DB 저장 실패: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = TurtleClient()
    w.resize(320, 360)
    w.show()
    sys.exit(app.exec_())
