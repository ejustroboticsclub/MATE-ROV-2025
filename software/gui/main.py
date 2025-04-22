import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QDialog
from PyQt5.QtGui import QFont
from landingPage import LandingPageUi
from pilot import PilotUi
from copilot import CopilotUi, CAM_PORTS
from engineer import EngineerUi
from Float import FloatUi
from utils import VideoCaptureThread, scale, ROSInterface

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        #add the ip , username,password as args in line 34 and unhash line 24 in copilot.py 
        # I Used stacked widget here for navigation between pages
        self.ros_interface = ROSInterface().node
        self.stacked_widget = QStackedWidget()

        self.landing_page = QDialog()
        self.pilot_page = QDialog()
        self.co_pilot_page = QDialog()
        self.engineer_page = QDialog()
        self.float_page = QDialog()

        # code for Setting up the UI for each page
        self.landing_page_ui = LandingPageUi()
        self.landing_page_ui.setupUi(self.landing_page)

        self.pilot_ui = PilotUi()
        self.pilot_ui.setupUi(self.pilot_page)

        self.float_ui = FloatUi()
        self.float_ui.setupUi(self.float_page)

        self.co_pilot_ui = CopilotUi("192.168.191.175","suzuki","samir2023", self.ros_interface)
        self.co_pilot_ui.setupUi(self.co_pilot_page)

        self.engineer_ui = EngineerUi()
        self.engineer_ui.setupUi(self.engineer_page)

        
        self.stacked_widget.addWidget(self.landing_page)
        self.stacked_widget.addWidget(self.pilot_page)
        self.stacked_widget.addWidget(self.co_pilot_page)
        self.stacked_widget.addWidget(self.engineer_page)
        self.stacked_widget.addWidget(self.float_page)

        
        self.setCentralWidget(self.stacked_widget)

        
        self.landing_page_ui.PilotButton.clicked.connect(self.show_pilot_page)
        self.landing_page_ui.CoButton.clicked.connect(self.show_co_pilot_page)
        self.landing_page_ui.EngButton.clicked.connect(self.show_engineer_page)
        self.landing_page_ui.FloatButton.clicked.connect(self.show_float_page)

        self.pilot_ui.BackButton.clicked.connect(self.show_landing_page)
        self.co_pilot_ui.back_button.clicked.connect(self.show_landing_page)
        self.engineer_ui.BackButton.clicked.connect(self.show_landing_page)
        self.float_ui.back_button.clicked.connect(self.show_landing_page)

        
        self.video_thread = VideoCaptureThread()

        
        self.engineer_ui.RecButton.clicked.connect(self.start_recording)
        self.engineer_ui.StopButton.clicked.connect(self.stop_recording)

        self._connect_ros_signals()


    def show_landing_page(self):
        self.stacked_widget.setCurrentWidget(self.landing_page)

    def show_pilot_page(self):
        self.stacked_widget.setCurrentWidget(self.pilot_page)

    def show_co_pilot_page(self):
        self.stacked_widget.setCurrentWidget(self.co_pilot_page)

    def show_engineer_page(self):
        self.stacked_widget.setCurrentWidget(self.engineer_page)

    def show_float_page(self):
        self.stacked_widget.setCurrentWidget(self.float_page)

    def start_recording(self):
        self.video_thread.start_recording()

    def stop_recording(self):
        self.video_thread.stop_recording()

    def _connect_ros_signals(self):
        """Connect ROS signals to UI updates"""
        self.ros_interface.signal_emitter.float_signal.connect(self.float_ui.update_float)
        self.ros_interface.signal_emitter.depth_signal.connect(self.co_pilot_ui.update_actual_depth)
        self.ros_interface.signal_emitter.gripper_r_signal.connect(self.co_pilot_ui.update_gripper_r)
        self.ros_interface.signal_emitter.gripper_l_signal.connect(self.co_pilot_ui.update_gripper_l)
        self.ros_interface.signal_emitter.thrusters_signal.connect(self.co_pilot_ui.update_thrusters)
        self.ros_interface.signal_emitter.imu_signal.connect(self.co_pilot_ui.update_imu)
        self.ros_interface.signal_emitter.indicators_signal.connect(self.co_pilot_ui.update_indicators)
        self.ros_interface.signal_emitter.desired_signal.connect(self.co_pilot_ui.update_desired_values)
        self.ros_interface.signal_emitter.angles_signal.connect(self.co_pilot_ui.update_angles)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont = QFont()
    window = MainWindow()
    window.setWindowTitle("Mate ROV 2025")
    window.resize(scale(930), scale(600))
    window.show()
    sys.exit(app.exec_())