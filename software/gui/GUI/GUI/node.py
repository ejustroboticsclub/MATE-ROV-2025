import sys
import rclpy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QGroupBox,
    QVBoxLayout, QLabel, QScrollArea
)
from PyQt5.QtCore import QTimer
from GUI.ros_logic import main_ros  # Import ROS logic

class DashboardWindow(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.setWindowTitle("System Dashboard - All Data")
        self.resize(1200, 800)

        # Create a scroll area to handle many fields.
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        container = QWidget()
        scroll.setWidget(container)
        self.setCentralWidget(scroll)

        # Create a grid layout.
        self.grid_layout = QGridLayout(container)
        self.field_widgets = {}


        fields = [
            "id", "valid",
            "imu_acc_x", "imu_acc_y", "imu_acc_z", "imu_roll", "imu_pitch", "imu_yaw",
            "thruster_current_1", "thruster_current_2", "thruster_current_3",
            "thruster_current_4", "thruster_current_5", "thruster_current_6",
            "thruster_pwm_1", "thruster_pwm_2", "thruster_pwm_3",
            "thruster_pwm_4", "thruster_pwm_5", "thruster_pwm_6",
            "indicator_1", "indicator_2", "indicator_3",
            "indicator_4", "indicator_5", "indicator_6",
            "heartbeat_1", "heartbeat_2", "heartbeat_3", "heartbeat_4",
            "connection_percentage_1", "connection_percentage_2",
            "connection_percentage_3", "connection_percentage_4",
            "arm_1", "arm_2", "arm_3", "arm_4",
            "rov_depth"
        ]

        # Arrange fields in a grid (e.g., 4 columns).
        cols = 4
        row = 0
        col = 0
        for field in fields:
            box = QGroupBox(field)
            vbox = QVBoxLayout()
            label = QLabel("N/A")
            vbox.addWidget(label)
            box.setLayout(vbox)
            self.field_widgets[field] = label
            self.grid_layout.addWidget(box, row, col)
            col += 1
            if col >= cols:
                col = 0
                row += 1

        # Timer to update the dashboard.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(500)

    def update_display(self):
        # Process ROS callbacks.
        rclpy.spin_once(self.ros_interface, timeout_sec=0)
        if self.ros_interface.latest_msg:
            msg = self.ros_interface.latest_msg
            for field, label in self.field_widgets.items():
                try:
                    # If field should be read from the embedded Imu message.
                    if field in ["acc_x", "acc_y", "acc_z", "imu_roll", "imu_pitch", "imu_yaw"]:
                        subfield = field[4:]
                        if hasattr(msg.imu, subfield):
                            value = getattr(msg.imu, subfield)
                        else:
                            value = getattr(msg.imu, field)
                    elif field.startswith("imu_"):
                        
                        subfield = field[4:]
                        if hasattr(msg.imu, subfield):
                            value = getattr(msg.imu, subfield)
                        else:
                            value = getattr(msg.imu, field)
                    else:
                        value = getattr(msg, field)
                    label.setText(str(value))
                except AttributeError as e:
                    label.setText("N/A")
                    self.ros_interface.get_logger().error(f"Error updating {field}: {e}")
            self.ros_interface.latest_msg = None

def main_gui():
    ros_interface = main_ros()
    app = QApplication(sys.argv)
    window = DashboardWindow(ros_interface)
    window.show()
    ret = app.exec_()
    ros_interface.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main_gui()
