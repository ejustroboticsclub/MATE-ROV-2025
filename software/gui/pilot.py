from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PyQt5.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PyQt5.QtWidgets import (QApplication, QDialog, QLabel, QPushButton,
    QSizePolicy, QWidget)

from stylesheet import Engineer_buttons_st , back_st
import os
from utils import CameraStreamer , BG_path ,scale  # Added scale import

class PilotUi(object):
    def setupUi(self, Dialog):
        # Loading font
        script_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.join(script_dir, "GillSans.ttf")
        id = QFontDatabase.addApplicationFont(font_path)
        if id == -1:
            print("Failed to load font!")
        
        families = QFontDatabase.applicationFontFamilies(id)
        # Scaled font sizes
        font = QFont(families[0], 22)
        Afont = QFont(families[0], 11)

        Dialog.setObjectName("Dialog")
        Dialog.resize(scale(928), scale(596))  # Scaled window size

        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("Background")
        self.Bg_label.setGeometry(QRect(scale(-3), scale(-5), scale(945), scale(607)))
        self.Bg_label.setPixmap(QPixmap(BG_path))
        self.Bg_label.setScaledContents(True)

        # Back button
        self.BackButton = QPushButton(Dialog)
        self.BackButton.setObjectName("Back button")
        self.BackButton.setGeometry(QRect(scale(10), scale(10), scale(61), scale(41)))
        icon = QIcon()
        icon = QIcon.fromTheme("go-previous")  
        self.BackButton.setIcon(icon)
        self.BackButton.setStyleSheet(back_st)
        self.BackButton.setFont(Afont)

        # Camera system button
        self.CamButton = QPushButton(Dialog)
        self.CamButton.setObjectName("Launching the Camera system button")
        self.CamButton.setGeometry(QRect(scale(290), scale(240), scale(351), scale(81)))
        self.CamButton.setStyleSheet(Engineer_buttons_st)
        self.CamButton.setFont(font)


        # Change to station's IP
        ip_pilot = "192.168.1.101"

        # Change to PI's IP
        ip_rasp = "192.168.1.100"
 
        # IPs passed for cameraStreamer class
        IPS = [
            f"rtsp://{ip_rasp}:5002/unicast", # Top left (Net)
            f"rtsp://{ip_rasp}:5001/unicast", # Top right (Side)
            f"rtsp://{ip_rasp}:5003/unicast", # Bottom left (Gripper)
            f"rtsp://{ip_rasp}:5004/unicast", # Bottom right (Jelly)
            f"rtsp://{ip_pilot}:8554/videofeed",  # Middle
        ]
        
        self.camera_6feeds = CameraStreamer(IPS)
        self.CamButton.clicked.connect(self.camera_6feeds.run)

        self.setText(Dialog)
        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.BackButton.setText(QCoreApplication.translate("Dialog", "Back", None))
        self.CamButton.setText(QCoreApplication.translate("Dialog", "Launch Camera System", None))