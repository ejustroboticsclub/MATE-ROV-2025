import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QDialog
from landingPage import LandingPageUi
from pilot import PilotUi
from copilot import CopilotUi
from engineer import EngineerUi
from utils import VideoCaptureThread 

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # I Used stacked widget here for navigation between pages
        self.stacked_widget = QStackedWidget()

        self.landing_page = QDialog()
        self.pilot_page = QDialog()
        self.co_pilot_page = QDialog()
        self.engineer_page = QDialog()

        # code for Setting up the UI for each page
        self.landing_page_ui = LandingPageUi()
        self.landing_page_ui.setupUi(self.landing_page)

        self.pilot_ui = PilotUi()
        self.pilot_ui.setupUi(self.pilot_page)

        self.co_pilot_ui = CopilotUi()
        self.co_pilot_ui.setupUi(self.co_pilot_page)

        self.engineer_ui = EngineerUi()
        self.engineer_ui.setupUi(self.engineer_page)

        
        self.stacked_widget.addWidget(self.landing_page)
        self.stacked_widget.addWidget(self.pilot_page)
        self.stacked_widget.addWidget(self.co_pilot_page)
        self.stacked_widget.addWidget(self.engineer_page)

        
        self.setCentralWidget(self.stacked_widget)

        
        self.landing_page_ui.PilotButton.clicked.connect(self.show_pilot_page)
        self.landing_page_ui.CoButton.clicked.connect(self.show_co_pilot_page)
        self.landing_page_ui.EngButton.clicked.connect(self.show_engineer_page)

        self.pilot_ui.BackButton.clicked.connect(self.show_landing_page)
        self.co_pilot_ui.back_button.clicked.connect(self.show_landing_page)
        self.engineer_ui.BackButton.clicked.connect(self.show_landing_page)

        
        self.video_thread = VideoCaptureThread()

        
        self.engineer_ui.RecButton.clicked.connect(self.start_recording)
        self.engineer_ui.StopButton.clicked.connect(self.stop_recording)

    def show_landing_page(self):
        self.stacked_widget.setCurrentWidget(self.landing_page)

    def show_pilot_page(self):
        self.stacked_widget.setCurrentWidget(self.pilot_page)

    def show_co_pilot_page(self):
        self.stacked_widget.setCurrentWidget(self.co_pilot_page)

    def show_engineer_page(self):
        self.stacked_widget.setCurrentWidget(self.engineer_page)

    def start_recording(self):
        self.video_thread.start_recording()

    def stop_recording(self):
        self.video_thread.stop_recording()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle("Mate ROV 2025")
    window.resize(930, 600)
    window.show()
    sys.exit(app.exec_())
