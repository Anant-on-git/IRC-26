from __future__ import annotations

import sys
# import threading  # Commented out - not needed for mock mode
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

# ROS2 backend commented out for testing
# try:
#     import numpy as np
# except ImportError:  # pragma: no cover - numpy is optional for camera conversion
#     np = None

# try:
#     import rclpy
#     from rclpy.node import Node
#     from rclpy.qos import QoSProfile, QoSReliabilityPolicy
#     from sensor_msgs.msg import Image
#     from std_msgs.msg import Float32
# except ImportError:  # pragma: no cover - keep GUI usable without ROS2
#     rclpy = None
#     Node = object  # type: ignore[misc, assignment]
#     QoSProfile = QoSReliabilityPolicy = object  # type: ignore[misc, assignment]
#     Float32 = Image = object  # type: ignore[misc, assignment]

np = None
rclpy = None


@dataclass
class TopicConfig:
    atm_temperature: str = "/bme280/atm_temperature"
    atm_humidity: str = "/bme280/atm_humidity"
    atm_pressure: str = "/bme280/atm_pressure"
    soil_temperature: str = "/soil/ds18b20/temperature"
    soil_moisture: str = "/soil/capacitive/moisture"
    camera_primary: str = "/camera/primary/image_raw"
    camera_secondary: str = "/camera/secondary/image_raw"


# ROS2 SensorNode class commented out for testing
# class SensorNode(Node):
#     """ROS2 node that subscribes to the requested sensor topics."""
# 
#     def __init__(self, bridge: "Ros2Bridge", topics: TopicConfig) -> None:
#         super().__init__("vishwa_gui_bridge")
#         qos = QoSProfile(depth=5)
#         if hasattr(qos, "reliability"):
#             qos.reliability = QoSReliabilityPolicy.BEST_EFFORT  # type: ignore[attr-defined]
# 
#         self.bridge = bridge
#         self.create_subscription(Float32, topics.atm_temperature, self._temperature_cb, qos)
#         self.create_subscription(Float32, topics.atm_humidity, self._humidity_cb, qos)
#         self.create_subscription(Float32, topics.atm_pressure, self._pressure_cb, qos)
#         self.create_subscription(Float32, topics.soil_temperature, self._soil_temp_cb, qos)
#         self.create_subscription(Float32, topics.soil_moisture, self._soil_moisture_cb, qos)
#         self.create_subscription(Image, topics.camera_primary, self._primary_camera_cb, qos)
#         self.create_subscription(Image, topics.camera_secondary, self._secondary_camera_cb, qos)
# 
#     def _temperature_cb(self, msg: Float32) -> None:
#         self.bridge.sensor_value.emit("atm_temperature", float(msg.data))
# 
#     def _humidity_cb(self, msg: Float32) -> None:
#         self.bridge.sensor_value.emit("atm_humidity", float(msg.data))
# 
#     def _pressure_cb(self, msg: Float32) -> None:
#         self.bridge.sensor_value.emit("atm_pressure", float(msg.data))
# 
#     def _soil_temp_cb(self, msg: Float32) -> None:
#         self.bridge.sensor_value.emit("soil_temperature", float(msg.data))
# 
#     def _soil_moisture_cb(self, msg: Float32) -> None:
#         self.bridge.sensor_value.emit("soil_moisture", float(msg.data))
# 
#     def _primary_camera_cb(self, msg: Image) -> None:
#         frame = self._convert_image(msg)
#         if frame is not None:
#             self.bridge.camera_frame.emit("primary", frame)
# 
#     def _secondary_camera_cb(self, msg: Image) -> None:
#         frame = self._convert_image(msg)
#         if frame is not None:
#             self.bridge.camera_frame.emit("secondary", frame)
# 
#     @staticmethod
#     def _convert_image(msg: Image) -> Optional[QImage]:
#         if np is None:
#             return None
#         if msg.encoding not in ("rgb8", "bgr8", "mono8"):
#             return None
# 
#         if msg.encoding == "mono8":
#             array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
#             qimg = QImage(array.data, msg.width, msg.height, msg.step, QImage.Format.Format_Grayscale8)
#             return qimg.copy()
# 
#         channels = 3
#         array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
#         if msg.encoding == "bgr8":
#             array = array[:, :, ::-1]
#         qimg = QImage(array.data, msg.width, msg.height, msg.step, QImage.Format.Format_RGB888)
#         return qimg.copy()


class Ros2Bridge(QObject):
    """Mock bridge for testing - generates fake sensor data."""

    sensor_value = pyqtSignal(str, float)
    camera_frame = pyqtSignal(str, QImage)
    status_changed = pyqtSignal(str)

    def __init__(self, topics: TopicConfig) -> None:
        super().__init__()
        self._topics = topics
        self._timer: Optional[QTimer] = None
        self._running = False
        self._counter = 0
        
        # Base values for mock data
        self._base_values = {
            "atm_temperature": 20.0,
            "atm_humidity": 55.0,
            "atm_pressure": 101.325,
            "soil_temperature": 18.5,
            "soil_moisture": 45.0,
        }

    def start(self) -> None:
        if self._running:
            return
        
        self._running = True
        self.status_changed.emit("Mock mode (testing)")
        
        # Create timer to emit mock data every 500ms
        self._timer = QTimer()
        self._timer.timeout.connect(self._emit_mock_data)
        self._timer.start(500)  # Update every 500ms

    def stop(self) -> None:
        if not self._running:
            return
        if self._timer is not None:
            self._timer.stop()
            self._timer = None
        self._running = False
        self.status_changed.emit("stopped")

    def _emit_mock_data(self) -> None:
        """Generate and emit mock sensor data."""
        import random
        import math
        
        self._counter += 1
        
        # Generate slightly varying values with sine waves for realistic behavior
        t = self._counter * 0.1
        
        # Atmospheric temperature: 20°C ± 2°C
        atm_temp = self._base_values["atm_temperature"] + 2 * math.sin(t * 0.5) + random.uniform(-0.5, 0.5)
        self.sensor_value.emit("atm_temperature", atm_temp)
        
        # Atmospheric humidity: 55% ± 5%
        atm_hum = self._base_values["atm_humidity"] + 5 * math.sin(t * 0.3) + random.uniform(-1, 1)
        self.sensor_value.emit("atm_humidity", max(0, min(100, atm_hum)))
        
        # Atmospheric pressure: 101.325 kPa ± 2 kPa
        atm_press = self._base_values["atm_pressure"] + 2 * math.sin(t * 0.2) + random.uniform(-0.3, 0.3)
        self.sensor_value.emit("atm_pressure", atm_press)
        
        # Soil temperature: 18.5°C ± 1.5°C
        soil_temp = self._base_values["soil_temperature"] + 1.5 * math.sin(t * 0.4) + random.uniform(-0.3, 0.3)
        self.sensor_value.emit("soil_temperature", soil_temp)
        
        # Soil moisture: 45% ± 10%
        soil_moist = self._base_values["soil_moisture"] + 10 * math.sin(t * 0.25) + random.uniform(-2, 2)
        self.sensor_value.emit("soil_moisture", max(0, min(100, soil_moist)))
        
        # Generate mock camera images (simple colored rectangles)
        if self._counter % 10 == 0:  # Update cameras less frequently
            img1 = QImage(640, 480, QImage.Format.Format_RGB32)
            img1.fill(QColor(100, 150, 200))  # Blue-ish
            self.camera_frame.emit("primary", img1)
            
            img2 = QImage(640, 480, QImage.Format.Format_RGB32)
            img2.fill(QColor(200, 150, 100))  # Orange-ish
            self.camera_frame.emit("secondary", img2)


class SensorValueCard(QFrame):
    def __init__(self, title: str, unit: str) -> None:
        super().__init__()
        self._unit = unit
        self.setObjectName("valueCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(4)

        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")

        self.value_label = QLabel("--")
        self.value_label.setObjectName("cardValue")

        unit_label = QLabel(unit)
        unit_label.setObjectName("cardUnit")

        layout.addWidget(self.value_label, alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(unit_label, alignment=Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignCenter)

    def update_value(self, value: float) -> None:
        if self._unit.upper() == "KPA":
            text = f"{value:0.3f}"
        else:
            text = f"{value:0.1f}"
        self.value_label.setText(text)


class SensorChart(QWidget):
    def __init__(self, title: str, unit: str, color: str = "#8ab4ff") -> None:
        super().__init__()
        self.setObjectName("chartCell")
        self.setStyleSheet(
            "background-color: #0c1221; border-radius: 18px; border: 1px solid rgba(255,255,255,0.25);"
        )
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
        from matplotlib.figure import Figure

        self._data: Deque[float] = deque(maxlen=300)
        self._color = color
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.fig = Figure(facecolor="#111726", figsize=(4, 3))
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.canvas.setMinimumHeight(200)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#111726")
        self.ax.tick_params(colors="#c8d1ff", labelsize=9)
        self.ax.spines["bottom"].set_color("#c8d1ff")
        self.ax.spines["left"].set_color("#c8d1ff")
        self.ax.set_title(f"{title} ({unit})", color="#c8d1ff", fontsize=11, pad=8)
        self.ax.set_xlabel("Samples", color="#c8d1ff", fontsize=10)
        self.ax.set_ylabel(unit, color="#c8d1ff", fontsize=10)
        self.ax.grid(True, alpha=0.2, color="#c8d1ff")
        self.line, = self.ax.plot([], [], color=color, linewidth=2)
        layout.addWidget(self.canvas)

    def append(self, value: float) -> None:
        self._data.append(value)
        xs = range(len(self._data))
        self.line.set_data(xs, list(self._data))
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()


class MetricPanel(QWidget):
    def __init__(self, card: SensorValueCard, chart: SensorChart) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)
        layout.addWidget(card, stretch=0)
        layout.addWidget(chart, stretch=1)


class CameraFeedWidget(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setObjectName("cameraCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet("background-color: #050912; border-radius: 12px;")
        layout.addWidget(self.image_label)

    def update_image(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image).scaled(
            self.image_label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
        )
        self.image_label.setPixmap(pixmap)


class SensorDashboardPage(QWidget):
    def __init__(self, bridge: Ros2Bridge) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 6, 12, 12)
        layout.setSpacing(8)

        self.status_label = QLabel("ROS2: Mock mode (testing)")
        self.status_label.setObjectName("statusLabel")
        layout.addWidget(self.status_label, alignment=Qt.AlignmentFlag.AlignRight)

        config_frame = QFrame()
        config_frame.setObjectName("configFrame")
        config_layout = QGridLayout(config_frame)
        config_layout.setSpacing(8)
        config_layout.setContentsMargins(16, 16, 16, 16)

        self.start_button = QPushButton("ROSCORE")
        self.shutdown_button = QPushButton("SHUTDOWN")
        for button in (self.start_button, self.shutdown_button):
            button.setObjectName("primaryButton")
            button.setFixedHeight(48)

        config_layout.addWidget(self.start_button, 0, 0, 1, 1)
        config_layout.addWidget(self.shutdown_button, 1, 0, 1, 1)

        control_labels = ["DRIVE control", "ARM control", "AUTONOMOUS control"]
        for idx, label in enumerate(control_labels):
            btn = QPushButton(label)
            btn.setObjectName("secondaryButton")
            btn.setEnabled(False)
            config_layout.addWidget(btn, idx, 1)

        layout.addWidget(config_frame)

        metrics_frame = QFrame()
        metrics_frame.setObjectName("metricsFrame")
        metrics_layout = QGridLayout(metrics_frame)
        metrics_layout.setSpacing(6)
        metrics_layout.setContentsMargins(6, 6, 6, 6)

        self.cards: Dict[str, SensorValueCard] = {
            "atm_temperature": SensorValueCard("Temperature", "°C"),
            "atm_humidity": SensorValueCard("Humidity", "%"),
            "atm_pressure": SensorValueCard("Pressure", "kPa"),
            "soil_temperature": SensorValueCard("Soil Temperature", "°C"),
            "soil_moisture": SensorValueCard("Soil Moisture", "%"),
        }

        self.charts = {
            "atm_temperature": SensorChart("Temperature", "°C", "#81a2ff"),
            "atm_humidity": SensorChart("Humidity", "%", "#a78bfa"),
            "atm_pressure": SensorChart("Pressure", "kPa", "#7dd3fc"),
        }

        metric_panels = {
            key: MetricPanel(self.cards[key], self.charts[key]) for key in self.charts.keys()
        }

        metric_positions = [
            (0, 0, metric_panels["atm_temperature"]),
            (0, 1, metric_panels["atm_humidity"]),
            (0, 2, metric_panels["atm_pressure"]),
        ]

        for row, col, widget in metric_positions:
            metrics_layout.addWidget(widget, row, col)

        metrics_layout.addWidget(self.cards["soil_temperature"], 1, 0)
        metrics_layout.addWidget(self.cards["soil_moisture"], 1, 1)
        metrics_layout.setColumnStretch(0, 1)
        metrics_layout.setColumnStretch(1, 1)
        metrics_layout.setColumnStretch(2, 1)

        layout.addWidget(metrics_frame)

        bridge.sensor_value.connect(self._update_value)
        bridge.status_changed.connect(self._update_status)
        self.start_button.clicked.connect(bridge.start)
        self.shutdown_button.clicked.connect(bridge.stop)

    def _update_value(self, key: str, value: float) -> None:
        if key in self.cards:
            self.cards[key].update_value(value)
        if key in self.charts:
            self.charts[key].append(value)

    def _update_status(self, text: str) -> None:
        self.status_label.setText(f"ROS2: {text}")


class CameraPage(QWidget):
    def __init__(self, bridge: Ros2Bridge) -> None:
        super().__init__()
        layout = QGridLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(24)

        self.primary = CameraFeedWidget("Front Camera")
        self.secondary = CameraFeedWidget("Rear Camera")
        layout.addWidget(self.primary, 0, 0)
        layout.addWidget(self.secondary, 0, 1)

        bridge.camera_frame.connect(self._update_frame)

    def _update_frame(self, channel: str, image: QImage) -> None:
        if channel == "primary":
            self.primary.update_image(image)
        else:
            self.secondary.update_image(image)


class NavigationButton(QPushButton):
    def __init__(self, text: str, page_index: int, stack: QStackedWidget) -> None:
        super().__init__(text)
        self.page_index = page_index
        self.stack = stack
        self.setCheckable(True)
        self.clicked.connect(self._handle_click)
        self.setObjectName("navButton")

    def _handle_click(self) -> None:
        self.stack.setCurrentIndex(self.page_index)
        self.setChecked(True)
        parent = self.parent()
        if parent:
            for child in parent.findChildren(NavigationButton):
                if child is not self:
                    child.setChecked(False)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("VISHWA Control Panel")
        self.resize(1400, 900)

        topics = TopicConfig()
        self.bridge = Ros2Bridge(topics)

        container = QWidget()
        root_layout = QVBoxLayout(container)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        nav_bar = QFrame()
        nav_bar.setObjectName("navBar")
        nav_layout = QGridLayout(nav_bar)
        nav_layout.setContentsMargins(20, 8, 20, 8)
        nav_layout.setHorizontalSpacing(8)

        self.stack = QStackedWidget()
        self.sensor_page = SensorDashboardPage(self.bridge)
        self.camera_page = CameraPage(self.bridge)
        self.stack.addWidget(self.sensor_page)
        self.stack.addWidget(self.camera_page)

        nav_buttons = [
            NavigationButton("Infometrics", 0, self.stack),
            NavigationButton("Cameras", 1, self.stack),
            NavigationButton("MAP", 0, self.stack),
        ]

        for idx, button in enumerate(nav_buttons):
            nav_layout.addWidget(button, 0, idx)

        nav_buttons[0].setChecked(True)

        root_layout.addWidget(nav_bar)
        root_layout.addWidget(self.stack)
        self.setCentralWidget(container)

        self.bridge.start()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.bridge.stop()
        super().closeEvent(event)


def apply_dark_palette(app: QApplication) -> None:
    app.setStyleSheet(
        """
        QWidget {
            background-color: #050912;
            color: #e5e5fb;
            font-family: 'Segoe UI', 'Inter', sans-serif;
        }
        #navBar {
            background-color: #7f5db3;
        }
        #navButton {
            background-color: transparent;
            border: none;
            color: #eee9ff;
            padding: 8px 20px;
            border-radius: 18px;
        }
        #navButton:checked {
            background-color: rgba(255,255,255,0.15);
            font-weight: 600;
        }
        #configFrame, #cameraCard, #valueCard, #metricsFrame {
            background-color: #0c1221;
            border-radius: 18px;
            border: 1px solid rgba(255,255,255,0.25);
        }
        #primaryButton {
            background-color: #a78bfa;
            border: none;
            color: #050912;
            border-radius: 28px;
            font-size: 16px;
            font-weight: 600;
        }
        #primaryButton:pressed {
            background-color: #8b6bd3;
        }
        #secondaryButton {
            background-color: transparent;
            border: 2px solid rgba(255,255,255,0.2);
            color: rgba(255,255,255,0.6);
            border-radius: 24px;
            font-weight: 600;
        }
        #cardTitle {
            font-size: 14px;
            color: rgba(255,255,255,0.7);
        }
        #cardValue {
            font-size: 32px;
            font-weight: bold;
        }
        #cardUnit {
            font-size: 14px;
            color: rgba(255,255,255,0.6);
        }
        #statusLabel {
            color: #9aa4ec;
            font-weight: 600;
        }
        """
    )


def main() -> None:
    app = QApplication(sys.argv)
    apply_dark_palette(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

